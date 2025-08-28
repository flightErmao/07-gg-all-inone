#include "anlCommand.h"

#ifdef PROJECT_MINIFLY_TASK04_DISTRIBUTE_COMMAND_EN

extern bool flyable;

void anlCommand(atkp_t *anlPacket) {
  switch (anlPacket->data[0]) {
    case D_COMMAND_ACC_CALIB:
      break;

    case D_COMMAND_GYRO_CALIB:
      break;

    case D_COMMAND_MAG_CALIB:
      break;

    case D_COMMAND_BARO_CALIB:
      break;

    case D_COMMAND_ACC_CALIB_STEP1:
      break;
    case D_COMMAND_ACC_CALIB_STEP2:
      break;
    case D_COMMAND_ACC_CALIB_STEP3:
      break;
    case D_COMMAND_ACC_CALIB_STEP4:
      break;
    case D_COMMAND_ACC_CALIB_STEP5:
      break;
    case D_COMMAND_ACC_CALIB_STEP6:
      break;

    case D_COMMAND_FLIGHT_LOCK:
      flyable = false;
      break;

    case D_COMMAND_FLIGHT_ULOCK:
      flyable = true;
      break;
  }
}

#endif /* PROJECT_MINIFLY_TASK04_DISTRIBUTE_COMMAND_EN */