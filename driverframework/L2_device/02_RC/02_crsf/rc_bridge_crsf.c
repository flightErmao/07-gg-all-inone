#include "crsf.h"
#include "anlRemote.h"

#ifdef RC_USING_CRSF
rcRawData_t getRcRawData(void)
{
    rcRawData_t out = {0};
    crsf_rc_data_t s = crsf_get_data();
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


