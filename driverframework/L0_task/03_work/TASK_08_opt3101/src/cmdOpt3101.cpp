#include <rtthread.h>
#include <rtdevice.h>
#include <finsh.h>

#include <cstring>
#include <cstdlib>

#include "hostController.h"
#include "OPT3101device.h"
#include "definitions.h"

#if defined(WORK_TASK_OPT3101_PARAM_SAVE_EN) && defined(PROJECT_MINIFLY_TASK05_PARAM_EN) && \
    defined(PROJECT_MINIFLY_TASK05_PARAM_OPT3101_EN)
#define OPT3101_PARAM_FEATURE 1
#include "opt3101Param.h"
#include "param.h"
static constexpr const char* OPT3101_PARAM_NAME = "opt3101_calibration";
#else
#define OPT3101_PARAM_FEATURE 0
#endif

namespace {

struct PrepareOptions {
  bool reset = false;
  bool init = false;
  bool load_param = false;
  bool check_comm = false;
};

OPT3101::device& opt3101_device() {
  static OPT3101::device device;
  return device;
}

bool opt3101_check_connection(OPT3101::device& dev) {
  bool ok = true;
  if (!dev.validateI2C()) {
    host.printf("[OPT3101][ERR] I2C handshake failed\r\n");
    ok = false;
  }
  if (!dev.validateDesignID()) {
    host.printf("[OPT3101][ERR] Design ID mismatch\r\n");
    ok = false;
  }
  return ok;
}

#if OPT3101_PARAM_FEATURE

rt_err_t opt3101_capture_registers(OPT3101::device& dev, opt3101_param_blob_t* blob) {
  if (blob == RT_NULL) {
    return -RT_ERROR;
  }

  const OPT3101::calibrationC& cal = dev.calibration[0];
  uint16_t count = cal.registerAddressListSize;
  if (count > OPT3101_PARAM_MAX_REG_COUNT) {
    host.printf("[OPT3101][ERR] register snapshot truncated %u -> %u\r\n", count, OPT3101_PARAM_MAX_REG_COUNT);
    count = OPT3101_PARAM_MAX_REG_COUNT;
  }

  opt3101_param_reset(blob);
  blob->count = count;

  for (uint16_t idx = 0; idx < count; ++idx) {
    uint8_t address = cal.registerAddressList[idx];
    blob->address[idx] = address;
    blob->value[idx] = host.readI2C(address);
  }

  return RT_EOK;
}

rt_err_t opt3101_save_registers_to_param(OPT3101::device& dev, rt_bool_t verbose) {
  opt3101_param_blob_t blob;
  opt3101_param_reset(&blob);

  rt_err_t ret = opt3101_capture_registers(dev, &blob);
  if (ret != RT_EOK) {
    return ret;
  }

  ret = setParam(OPT3101_PARAM_NAME, &blob, sizeof(blob));
  if (ret != RT_EOK) {
    host.printf("[OPT3101][ERR] setParam %s failed (%d)\r\n", OPT3101_PARAM_NAME, ret);
    return ret;
  }

  if (verbose) {
    host.printf("[OPT3101] saved %u registers to param\r\n", blob.count);
  }
  return RT_EOK;
}

rt_err_t opt3101_load_registers_from_param(OPT3101::device& dev, rt_bool_t verbose) {
  opt3101_param_blob_t blob;
  rt_err_t ret = getParam(OPT3101_PARAM_NAME, &blob, sizeof(blob));
  if (ret != RT_EOK) {
    if (verbose) {
      host.printf("[OPT3101][WARN] getParam %s failed (%d)\r\n", OPT3101_PARAM_NAME, ret);
    }
    return ret;
  }

  if (blob.magic != OPT3101_PARAM_MAGIC) {
    if (verbose) {
      host.printf("[OPT3101][WARN] param magic mismatch (0x%08x)\r\n", blob.magic);
    }
    return -RT_ERROR;
  }

  uint16_t count = blob.count;
  if (count > OPT3101_PARAM_MAX_REG_COUNT) {
    count = OPT3101_PARAM_MAX_REG_COUNT;
  }

  for (uint16_t idx = 0; idx < count; ++idx) {
    host.writeI2C(blob.address[idx], blob.value[idx]);
  }

  if (verbose) {
    host.printf("[OPT3101] loaded %u registers from param\r\n", count);
  }
  return RT_EOK;
}

void opt3101_print_param_summary(void) {
  opt3101_param_blob_t blob;
  if (getParam(OPT3101_PARAM_NAME, &blob, sizeof(blob)) != RT_EOK) {
    host.printf("[OPT3101][INFO] no param snapshot stored\r\n");
    return;
  }
  host.printf("[OPT3101][INFO] param magic 0x%08x version %u count %u\r\n",
              blob.magic, blob.version, blob.count);
}

#else

rt_err_t opt3101_save_registers_to_param(OPT3101::device&, rt_bool_t) {
  host.printf("[OPT3101][INFO] parameter storage disabled\r\n");
  return -RT_ENOSYS;
}

rt_err_t opt3101_load_registers_from_param(OPT3101::device&, rt_bool_t verbose) {
  if (verbose) {
    host.printf("[OPT3101][INFO] parameter storage disabled\r\n");
  }
  return -RT_ENOSYS;
}

void opt3101_print_param_summary(void) {
  host.printf("[OPT3101][INFO] parameter storage disabled\r\n");
}

#endif

bool opt3101_prepare_device(const PrepareOptions& options) {
  OPT3101::device& dev = opt3101_device();
  host.initialize();

  if (options.reset) {
    host.printf("[OPT3101] resetting device\r\n");
    dev.reset();
    host.sleep(WORK_TASK_OPT3101_RESET_RELEASE_MS);
  }

  if (options.init) {
    host.printf("[OPT3101] initializing device\r\n");
    dev.initialize();
  }

  if (options.load_param) {
    opt3101_load_registers_from_param(dev, RT_TRUE);
  }

  if (options.check_comm) {
    return opt3101_check_connection(dev);
  }
  return true;
}

int result_from_bool(bool status) {
  return status ? 0 : -RT_ERROR;
}

}  // namespace

static int cmd_opt3101_ping(int argc, char** argv) {
  RT_UNUSED(argc);
  RT_UNUSED(argv);

  PrepareOptions options{true, true, false, false};
  opt3101_prepare_device(options);
  bool ok = opt3101_check_connection(opt3101_device());
  host.printf("[OPT3101] ping %s\r\n", ok ? "OK" : "FAIL");
  return result_from_bool(ok);
}
MSH_CMD_EXPORT_ALIAS(cmd_opt3101_ping, opt3101_ping, "Check OPT3101 I2C and design ID");

static int cmd_opt3101_reset(int argc, char** argv) {
  RT_UNUSED(argc);
  RT_UNUSED(argv);
  PrepareOptions options{true, false, false, false};
  opt3101_prepare_device(options);
  host.printf("[OPT3101] reset done\r\n");
  return 0;
}
MSH_CMD_EXPORT_ALIAS(cmd_opt3101_reset, opt3101_reset, "Reset OPT3101 via hardware line");

static int cmd_opt3101_init(int argc, char** argv) {
  RT_UNUSED(argc);
  RT_UNUSED(argv);
  PrepareOptions options{true, true, true, true};
  bool ok = opt3101_prepare_device(options);
  host.printf("[OPT3101] init %s\r\n", ok ? "OK" : "FAIL");
  return result_from_bool(ok);
}
MSH_CMD_EXPORT_ALIAS(cmd_opt3101_init, opt3101_init, "Reset, init and load calibration");

static int cmd_opt3101_first(int argc, char** argv) {
  RT_UNUSED(argc);
  RT_UNUSED(argv);
  PrepareOptions options{true, true, false, true};
  if (!opt3101_prepare_device(options)) {
    return -RT_ERROR;
  }

  OPT3101::device& dev = opt3101_device();
  dev.calibrationSession_firstTimeBringUp();
  opt3101_save_registers_to_param(dev, RT_TRUE);
  return 0;
}
MSH_CMD_EXPORT_ALIAS(cmd_opt3101_first, opt3101_calib_first, "First-time bring-up calibration");

static int cmd_opt3101_xtalk(int argc, char** argv) {
  RT_UNUSED(argc);
  RT_UNUSED(argv);
  PrepareOptions options{true, true, false, true};
  if (!opt3101_prepare_device(options)) {
    return -RT_ERROR;
  }

  OPT3101::device& dev = opt3101_device();
  dev.calibrationSession_perDesignCalibrationCrosstalkTemp();
  opt3101_save_registers_to_param(dev, RT_TRUE);
  return 0;
}
MSH_CMD_EXPORT_ALIAS(cmd_opt3101_xtalk, opt3101_calib_xtalk, "Crosstalk temperature calibration");

static int cmd_opt3101_phase_temp(int argc, char** argv) {
  RT_UNUSED(argc);
  RT_UNUSED(argv);
  PrepareOptions options{true, true, false, true};
  if (!opt3101_prepare_device(options)) {
    return -RT_ERROR;
  }

  OPT3101::device& dev = opt3101_device();
  dev.calibrationSession_perDesignCalibrationPhaseTemp();
  opt3101_save_registers_to_param(dev, RT_TRUE);
  return 0;
}
MSH_CMD_EXPORT_ALIAS(cmd_opt3101_phase_temp, opt3101_calib_phase_temp, "Phase temperature calibration");

static int cmd_opt3101_phase_ambient(int argc, char** argv) {
  RT_UNUSED(argc);
  RT_UNUSED(argv);
  PrepareOptions options{true, true, false, true};
  if (!opt3101_prepare_device(options)) {
    return -RT_ERROR;
  }

  OPT3101::device& dev = opt3101_device();
  dev.calibrationSession_perDesignCalibrationPhaseAmbient();
  opt3101_save_registers_to_param(dev, RT_TRUE);
  return 0;
}
MSH_CMD_EXPORT_ALIAS(cmd_opt3101_phase_ambient, opt3101_calib_phase_ambient, "Phase ambient calibration");

static int cmd_opt3101_factory(int argc, char** argv) {
  RT_UNUSED(argc);
  RT_UNUSED(argv);
  PrepareOptions options{true, true, false, true};
  if (!opt3101_prepare_device(options)) {
    return -RT_ERROR;
  }

  OPT3101::device& dev = opt3101_device();
  dev.calibrationSession_perUnitFactoryCalibration();
  opt3101_save_registers_to_param(dev, RT_TRUE);
  return 0;
}
MSH_CMD_EXPORT_ALIAS(cmd_opt3101_factory, opt3101_calib_factory, "Per-unit factory calibration");

static int cmd_opt3101_live(int argc, char** argv) {
  uint32_t frames = 100;
  bool load_calibration = true;
  if (argc > 1) {
    frames = static_cast<uint32_t>(strtoul(argv[1], RT_NULL, 0));
  }
  if (argc > 2) {
    load_calibration = (std::strcmp(argv[2], "0") != 0);
  }

  PrepareOptions options{true, true, load_calibration, true};
  if (!opt3101_prepare_device(options)) {
    return -RT_ERROR;
  }

  OPT3101::device& dev = opt3101_device();
  dev.resetInitAndViewData(frames, load_calibration);
  return 0;
}
MSH_CMD_EXPORT_ALIAS(cmd_opt3101_live, opt3101_live, "View live frames, usage: opt3101_live [frames] [loadCalibration]");

static int cmd_opt3101_save(int argc, char** argv) {
  RT_UNUSED(argc);
  RT_UNUSED(argv);
  PrepareOptions options{false, false, false, false};
  opt3101_prepare_device(options);
  return opt3101_save_registers_to_param(opt3101_device(), RT_TRUE);
}
MSH_CMD_EXPORT_ALIAS(cmd_opt3101_save, opt3101_save, "Save current calibration registers to param");

static int cmd_opt3101_load(int argc, char** argv) {
  RT_UNUSED(argc);
  RT_UNUSED(argv);
  PrepareOptions options{false, false, false, false};
  opt3101_prepare_device(options);
  return opt3101_load_registers_from_param(opt3101_device(), RT_TRUE);
}
MSH_CMD_EXPORT_ALIAS(cmd_opt3101_load, opt3101_load, "Load calibration registers from param");

static int cmd_opt3101_review(int argc, char** argv) {
  RT_UNUSED(argc);
  RT_UNUSED(argv);
  OPT3101::device& dev = opt3101_device();
  dev.calibration[0].report();
  opt3101_print_param_summary();
  return 0;
}
MSH_CMD_EXPORT_ALIAS(cmd_opt3101_review, opt3101_review, "Print calibration summary");


