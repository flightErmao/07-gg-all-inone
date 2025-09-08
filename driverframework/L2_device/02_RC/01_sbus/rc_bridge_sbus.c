#include "sbus.h"
#include "anlRemote.h"
#include <rtthread.h>

/* Provide RC raw data from SBUS when SBUS is enabled */
#ifdef RC_USING_SBUS
rcRawData_t getRcRawData(void)
{
    rcRawData_t out = {0};
    sbus_rc_data_t s = sbus_get_data();
    out.timestamp = s.timestamp;
    out.roll = s.roll;
    out.pitch = s.pitch;
    out.yaw = s.yaw;
    out.thrust = s.thrust;
    out.arm_status = s.arm_status;
    out.ctrl_mode = s.ctrl_mode;
    return out;
}
#endif


