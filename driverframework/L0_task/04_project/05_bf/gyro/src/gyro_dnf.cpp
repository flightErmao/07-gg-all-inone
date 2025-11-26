#include "gyro_dnf.h"

// Support both USE_DYN_NOTCH_FILTER and CONFIG_USE_DYN_NOTCH_FILTER
#if defined(USE_DYN_NOTCH_FILTER) || defined(CONFIG_USE_DYN_NOTCH_FILTER)

extern "C" {
#include <rtthread.h>
#include <rtconfig.h>
#include <math.h>
#define LOG_TAG "gyro_dnf"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>
#include "filter.h"  // For pt1FilterGain, biquadFilterInit, biquadFilterUpdate, biquadFilterApplyDF1
#include "maths.h"   // For MAX, MIN, constrainf, lrintf
// sdft.h is already included in gyro_dnf.h, no need to include again
}

#include <cstring>
#include <algorithm>

GyroDynNotch::GyroDynNotch()
    : q_(120.0f / 100.0f),  // 默认 Q = 1.2
      min_hz_(250.0f),
      max_hz_(550.0f),
      count_(0),
      looptime_us_(0),
      max_center_freq_(0),
      sample_index_(0),
      sample_count_(0),
      sample_count_rcp_(0.0f),
      state_tick_(0),
      state_step_(STEP_WINDOW),
      state_axis_(0),
      sdft_sample_rate_hz_(0.0f),
      sdft_resolution_hz_(0.0f),
      sdft_start_bin_(0),
      sdft_end_bin_(0),
      sdft_noise_threshold_(0.0f),
      pt1_looptime_s_(0.0f) {
  std::memset(center_freq_, 0, sizeof(center_freq_));
  std::memset(notch_, 0, sizeof(notch_));
  std::memset(sample_accumulator_, 0, sizeof(sample_accumulator_));
  std::memset(sample_avg_, 0, sizeof(sample_avg_));
  // Initialize SDFT buffers (value-initialize to avoid warnings with std::complex)
  for (auto& sdft_obj : sdft_) {
    sdft_obj = {};
  }
  std::memset(sdft_data_, 0, sizeof(sdft_data_));
  std::memset(peaks_, 0, sizeof(peaks_));
}

void GyroDynNotch::init(uint16_t dyn_notch_q, uint16_t dyn_notch_min_hz, uint16_t dyn_notch_max_hz,
                        uint8_t dyn_notch_count, uint32_t targetLooptimeUs) {
  // dynNotchUpdate() is running at looprateHz (which is the PID looprate aka. 1e6f / targetLooptimeUs)
  const float looprateHz = 1e6f / targetLooptimeUs;
  const float nyquistHz = looprateHz / 2.0f;

  // Disable dynamic notch if dynNotchUpdate() would run at less than 2kHz
  if (looprateHz < DYN_NOTCH_UPDATE_MIN_HZ) {
    count_ = 0;
    LOG_W("Dynamic notch disabled: looprate %.1f Hz < %.1f Hz", looprateHz, (float)DYN_NOTCH_UPDATE_MIN_HZ);
    return;
  }

  // If dynamic notch is available, initialise so it can be activated at any time
  q_ = dyn_notch_q / 100.0f;
  min_hz_ = dyn_notch_min_hz;
  max_hz_ = MAX(min_hz_, dyn_notch_max_hz);
  max_hz_ = MIN(max_hz_, nyquistHz);  // Ensure to not go above the nyquist limit
  count_ = (dyn_notch_count > DYN_NOTCH_COUNT_MAX) ? DYN_NOTCH_COUNT_MAX : dyn_notch_count;
  looptime_us_ = targetLooptimeUs;
  max_center_freq_ = 0;

  sample_count_ = MAX(1, (int)(nyquistHz / max_hz_));  // maxHz = 600 & looprateHz = 8000 -> sampleCount = 6
  sample_count_rcp_ = 1.0f / sample_count_;

  sdft_sample_rate_hz_ = looprateHz / sample_count_;
  // eg 8k, user max 600hz, int(4000/600) = 6 (6.666), sdftSampleRateHz = 1333hz, range 666Hz
  // eg 4k, user max 600hz, int(2000/600) = 3 (3.333), sdftSampleRateHz = 1333hz, range 666Hz
  // eg 2k, user max 600hz, int(1000/600) = 1 (1.666)  sdftSampleRateHz = 2000hz, range 1000Hz
  // eg 2k, user max 400hz, int(1000/400) = 2 (2.5)    sdftSampleRateHz = 1000hz, range 500Hz
  // eg 1k, user max 600hz, int(500/500)  = 1 (1.0)    sdftSampleRateHz = 1000hz, range 500Hz
  // The upper limit of DN is always going to be the Nyquist frequency (= sampleRate / 2)

  sdft_resolution_hz_ = sdft_sample_rate_hz_ / SDFT_SAMPLE_SIZE;  // 18.5hz per bin at 8k and 600Hz maxHz
  sdft_start_bin_ = MAX(1, (int)lrintf(min_hz_ / sdft_resolution_hz_));  // can't use bin 0 because it is DC.
  sdft_end_bin_ = MIN(SDFT_BIN_COUNT - 1, (int)lrintf(max_hz_ / sdft_resolution_hz_));  // can't use more than SDFT_BIN_COUNT bins.
  pt1_looptime_s_ = DYN_NOTCH_CALC_TICKS / looprateHz;

  for (int axis = 0; axis < 3; axis++) {
    sdftInit(&sdft_[axis], sdft_start_bin_, sdft_end_bin_, sample_count_);
  }

  for (int axis = 0; axis < 3; axis++) {
    for (int p = 0; p < count_; p++) {
      // any init value is fine, but evenly spreading centerFreqs across frequency range makes notches stick to peaks quicker
      center_freq_[axis][p] = (p + 0.5f) * (max_hz_ - min_hz_) / (float)count_ + min_hz_;
      biquadFilterInit(&notch_[axis][p], center_freq_[axis][p], looptime_us_, q_, FILTER_NOTCH, 1.0f);
    }
  }

  sample_index_ = 0;
  state_tick_ = 0;
  state_step_ = STEP_WINDOW;
  state_axis_ = 0;

  LOG_I("Dynamic notch initialized: q=%.2f, min=%.1f Hz, max=%.1f Hz, count=%d, sampleCount=%d, sdftRate=%.1f Hz",
        q_, min_hz_, max_hz_, count_, sample_count_, sdft_sample_rate_hz_);
}

// Collect gyro data, to be downsampled and analysed in update() function
void GyroDynNotch::push(int axis, float sample) {
  if (axis < 0 || axis >= 3) {
    return;
  }
  sample_accumulator_[axis] += sample;
}

// Downsample and analyse gyro data
void GyroDynNotch::update() {
  // samples should have been pushed by `push`
  // if gyro sampling is > 1kHz, accumulate and average multiple gyro samples
  if (sample_index_ == sample_count_) {
    sample_index_ = 0;

    // calculate mean value of accumulated samples
    for (int axis = 0; axis < 3; axis++) {
      sample_avg_[axis] = sample_accumulator_[axis] * sample_count_rcp_;
      sample_accumulator_[axis] = 0;
    }

    // We need DYN_NOTCH_CALC_TICKS ticks to update all axes with newly sampled value
    // recalculation of filters takes 4 calls per axis => each filter gets updated every DYN_NOTCH_CALC_TICKS calls
    // at 8kHz PID loop rate this means 8kHz / 4 / 3 = 666Hz => update every 1.5ms
    // at 4kHz PID loop rate this means 4kHz / 4 / 3 = 333Hz => update every 3ms
    state_tick_ = DYN_NOTCH_CALC_TICKS;
  }

  // SDFT processing in batches to synchronize with incoming downsampled data
  for (int axis = 0; axis < 3; axis++) {
    sdftPushBatch(&sdft_[axis], sample_avg_[axis], sample_index_);
  }
  sample_index_++;

  // Find frequency peaks and update filters
  if (state_tick_ > 0) {
    process();
    --state_tick_;
  }
}

// Find frequency peaks and update filters
void GyroDynNotch::process() {
  switch (state_step_) {
    case STEP_WINDOW:  // 4.1us (3-6us) @ F722
    {
      sdftWinSq(&sdft_[state_axis_], sdft_data_);

      // Get total vibrational power in dyn notch range for noise floor estimate in STEP_CALC_FREQUENCIES
      sdft_noise_threshold_ = 0.0f;
      for (int bin = sdft_start_bin_; bin <= sdft_end_bin_; bin++) {
        sdft_noise_threshold_ += sdft_data_[bin];  // sdftData contains power spectral density
      }

      break;
    }
    case STEP_DETECT_PEAKS:  // 5.5us (4-7us) @ F722
    {
      // Get memory ready for new peak data on current axis
      for (int p = 0; p < count_; p++) {
        peaks_[p].bin = 0;
        peaks_[p].value = 0.0f;
      }

      // Search for N biggest peaks in frequency spectrum
      for (int bin = (sdft_start_bin_ + 1); bin < sdft_end_bin_; bin++) {
        // Check if bin is peak
        if ((sdft_data_[bin] > sdft_data_[bin - 1]) && (sdft_data_[bin] > sdft_data_[bin + 1])) {
          // Check if peak is big enough to be one of N biggest peaks.
          // If so, insert peak and sort peaks in descending height order
          for (int p = 0; p < count_; p++) {
            if (sdft_data_[bin] > peaks_[p].value) {
              for (int k = count_ - 1; k > p; k--) {
                peaks_[k] = peaks_[k - 1];
              }
              peaks_[p].bin = bin;
              peaks_[p].value = sdft_data_[bin];
              break;
            }
          }
          bin++;  // If bin is peak, next bin can't be peak => skip it
        }
      }

      // Sort N biggest peaks in ascending bin order (example: 3, 8, 25, 0, 0, ..., 0)
      for (int p = count_ - 1; p > 0; p--) {
        for (int k = 0; k < p; k++) {
          // Swap peaks but ignore swapping void peaks (bin = 0). This leaves
          // void peaks at the end of peaks array without moving them
          if (peaks_[k].bin > peaks_[k + 1].bin && peaks_[k + 1].bin != 0) {
            dynNotchPeak_t temp = peaks_[k];
            peaks_[k] = peaks_[k + 1];
            peaks_[k + 1] = temp;
          }
        }
      }

      break;
    }
    case STEP_CALC_FREQUENCIES:  // 4.0us (2-7us) @ F722
    {
      // Approximate noise floor (= average power spectral density in dyn notch range, excluding peaks)
      int peakCount = 0;
      for (int p = 0; p < count_; p++) {
        if (peaks_[p].bin != 0) {
          sdft_noise_threshold_ -= 0.75f * sdft_data_[peaks_[p].bin - 1];
          sdft_noise_threshold_ -= sdft_data_[peaks_[p].bin];
          sdft_noise_threshold_ -= 0.75f * sdft_data_[peaks_[p].bin + 1];
          peakCount++;
        }
      }
      sdft_noise_threshold_ /= sdft_end_bin_ - sdft_start_bin_ - peakCount + 1;

      // A noise threshold 2 times the noise floor prevents peak tracking being too sensitive to noise
      sdft_noise_threshold_ *= 2.0f;

      for (int p = 0; p < count_; p++) {
        // Only update centerFreq if there is a peak (ignore void peaks) and if peak is above noise floor
        if (peaks_[p].bin != 0 && peaks_[p].value > sdft_noise_threshold_) {
          float meanBin = peaks_[p].bin;

          // Height of peak bin (y1) and shoulder bins (y0, y2)
          const float y0 = sdft_data_[peaks_[p].bin - 1];
          const float y1 = sdft_data_[peaks_[p].bin];
          const float y2 = sdft_data_[peaks_[p].bin + 1];

          // Estimate true peak position aka. meanBin (fit parabola y(x) over y0, y1 and y2, solve dy/dx=0 for x)
          const float denom = 2.0f * (y0 - 2 * y1 + y2);
          if (denom != 0.0f) {
            meanBin += (y0 - y2) / denom;
          }

          // Convert bin to frequency: freq = bin * binResolution (bin 0 is 0Hz)
          const float centerFreq = constrainf(meanBin * sdft_resolution_hz_, min_hz_, max_hz_);

          // PT1 style smoothing moves notch center freqs rapidly towards big peaks and slowly away, up to 10x faster
          const float cutoffMult = constrainf(peaks_[p].value / sdft_noise_threshold_, 1.0f, 10.0f);
          const float gain = pt1FilterGain(DYN_NOTCH_SMOOTH_HZ * cutoffMult, pt1_looptime_s_);  // dynamic PT1 k value

          // Finally update notch center frequency p on current axis
          center_freq_[state_axis_][p] += gain * (centerFreq - center_freq_[state_axis_][p]);
        }
      }

      // Update max center frequency for debugging (when throttle > 20%)
      // Note: throttle check is done in motor task, we just track max here
      for (int p = 0; p < count_; p++) {
        max_center_freq_ = MAX(max_center_freq_, (int)lrintf(center_freq_[state_axis_][p]));
      }

      break;
    }
    case STEP_UPDATE_FILTERS:  // 5.4us (2-9us) @ F722
    {
      for (int p = 0; p < count_; p++) {
        // Only update notch filter coefficients if the corresponding peak got its center frequency updated in the previous step
        if (peaks_[p].bin != 0 && peaks_[p].value > sdft_noise_threshold_) {
          biquadFilterUpdate(&notch_[state_axis_][p], center_freq_[state_axis_][p], looptime_us_, q_, FILTER_NOTCH, 1.0f);
        }
      }

      state_axis_ = (state_axis_ + 1) % 3;
      break;
    }
    case STEP_COUNT:
    default:
      // No action needed; guard to keep -Wswitch satisfied
      break;
  }

  state_step_ = (dynNotchStep_e)((state_step_ + 1) % STEP_COUNT);
}

// Apply dynamic notch filter
float GyroDynNotch::filter(int axis, float value) {
  if (axis < 0 || axis >= 3) {
    return value;
  }

  for (int p = 0; p < count_; p++) {
    value = biquadFilterApplyDF1(&notch_[axis][p], value);
  }

  return value;
}

#endif  // USE_DYN_NOTCH_FILTER || CONFIG_USE_DYN_NOTCH_FILTER

