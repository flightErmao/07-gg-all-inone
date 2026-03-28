#ifndef APPLICATIONS_SESSION_MANAGER_H
#define APPLICATIONS_SESSION_MANAGER_H

#include <rtthread.h>

typedef enum
{
    SESSION_STATE_IDLE = 0,
    SESSION_STATE_RUNNING,
    SESSION_STATE_STOPPING,
} session_state_t;

typedef enum
{
    SESSION_TEST_NONE = 0,
    SESSION_TEST_BIAS,
    SESSION_TEST_TEMP_DRIFT,
    SESSION_TEST_ARW,
    SESSION_TEST_VIBRATION,
    SESSION_TEST_SHOCK,
} session_test_type_t;

int session_manager_init(void);
int session_start(session_test_type_t test_type);
int session_start_by_name(const char *test_name);
int session_stop(void);
void session_complete_current(void);
session_state_t session_get_state(void);
session_test_type_t session_get_test_type(void);
int session_mark_temperature(rt_int32_t temp_c);
const char *session_test_name(session_test_type_t test_type);

#endif
