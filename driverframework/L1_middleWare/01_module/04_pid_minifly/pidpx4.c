#include "pidpx4.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static float rcpx4_constrain(float v, float min_v, float max_v) {
  if (v < min_v) return min_v;
  if (v > max_v) return max_v;
  return v;
}

static float rcpx4_maxf(float a, float b) { return (a > b) ? a : b; }
static float rcpx4_minf(float a, float b) { return (a < b) ? a : b; }

void rcpx4_init(RateControlPX4 *rc) {
  if (!rc) return;
  memset(rc, 0, sizeof(*rc));
}

void rcpx4_set_pid(RateControlPX4 *rc, const float P[3], const float I[3], const float D[3]) {
  if (!rc) return;
  memcpy(rc->gain_p, P, sizeof(rc->gain_p));
  memcpy(rc->gain_i, I, sizeof(rc->gain_i));
  memcpy(rc->gain_d, D, sizeof(rc->gain_d));
}

void rcpx4_set_integrator_limit(RateControlPX4 *rc, const float lim_int[3]) {
  if (!rc) return;
  memcpy(rc->lim_int, lim_int, sizeof(rc->lim_int));
}

void rcpx4_set_ff(RateControlPX4 *rc, const float FF[3]) {
  if (!rc) return;
  memcpy(rc->gain_ff, FF, sizeof(rc->gain_ff));
}

void rcpx4_set_saturation(RateControlPX4 *rc, const uint8_t sat_pos[3], const uint8_t sat_neg[3]) {
  if (!rc) return;
  memcpy(rc->sat_pos, sat_pos, sizeof(rc->sat_pos));
  memcpy(rc->sat_neg, sat_neg, sizeof(rc->sat_neg));
}

void rcpx4_set_pos_sat_axis(RateControlPX4 *rc, size_t axis, uint8_t is_saturated) {
  if (!rc) return;
  if (axis < 3) rc->sat_pos[axis] = is_saturated;
}

void rcpx4_set_neg_sat_axis(RateControlPX4 *rc, size_t axis, uint8_t is_saturated) {
  if (!rc) return;
  if (axis < 3) rc->sat_neg[axis] = is_saturated;
}

void rcpx4_reset_integral(RateControlPX4 *rc) {
  if (!rc) return;
  rc->rate_int[0] = 0.f;
  rc->rate_int[1] = 0.f;
  rc->rate_int[2] = 0.f;
}

void rcpx4_reset_integral_axis(RateControlPX4 *rc, size_t axis) {
  if (!rc) return;
  if (axis < 3) rc->rate_int[axis] = 0.f;
}

void rcpx4_update(RateControlPX4 *rc,
                  const float rate[3],
                  const float rate_sp[3],
                  const float angular_accel[3],
                  float dt,
                  uint8_t landed,
                  float torque[3]) {
  if (!rc || !rate || !rate_sp || !angular_accel || !torque) return;

  float rate_error[3];
  rate_error[0] = rate_sp[0] - rate[0];
  rate_error[1] = rate_sp[1] - rate[1];
  rate_error[2] = rate_sp[2] - rate[2];

  /* P + I - D(angular_accel) + FF * rate_sp */
  for (int i = 0; i < 3; i++) {
    torque[i] = rc->gain_p[i] * rate_error[i] + rc->rate_int[i] - rc->gain_d[i] * angular_accel[i] + rc->gain_ff[i] * rate_sp[i];
  }

  if (!landed) {
    for (int i = 0; i < 3; i++) {
      /* 防止饱和方向继续积分 */
      if (rc->sat_pos[i]) {
        rate_error[i] = rcpx4_minf(rate_error[i], 0.f);
      }
      if (rc->sat_neg[i]) {
        rate_error[i] = rcpx4_maxf(rate_error[i], 0.f);
      }

      /* I 因子随误差二次衰减，参考 PX4 i_factor 实现。
         400 deg 的比例：i_factor = max(0, 1 - (err / rad(400))^2) */
      const float rad400 = (400.0f * (float)M_PI) / 180.0f;
      float i_factor = rate_error[i] / rad400;
      i_factor = rcpx4_maxf(0.0f, 1.0f - i_factor * i_factor);

      float rate_i = rc->rate_int[i] + i_factor * rc->gain_i[i] * rate_error[i] * dt;
      /* 约束积分 */
      rc->rate_int[i] = rcpx4_constrain(rate_i, -rc->lim_int[i], rc->lim_int[i]);
    }
  }
}


