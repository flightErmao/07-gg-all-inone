#include <rtthread.h>
#include <string.h>

#include "session_manager.h"
#include "test_arw.h"
#include "test_bias.h"
#include "test_shock.h"
#include "test_temp_drift.h"
#include "test_vibration.h"

static session_state_t g_session_state = SESSION_STATE_IDLE;
static session_test_type_t g_session_test_type = SESSION_TEST_NONE;
static rt_int32_t g_last_mark_temp_c = 0;

const char *session_test_name(session_test_type_t test_type)
{
    switch (test_type)
    {
    case SESSION_TEST_BIAS:
        return "bias";
    case SESSION_TEST_TEMP_DRIFT:
        return "temp";
    case SESSION_TEST_ARW:
        return "arw";
    case SESSION_TEST_VIBRATION:
        return "vibration";
    case SESSION_TEST_SHOCK:
        return "shock";
    default:
        return "none";
    }
}

static session_test_type_t session_find_test_type(const char *test_name)
{
    if (test_name == RT_NULL)
    {
        return SESSION_TEST_NONE;
    }

    if (rt_strcmp(test_name, "bias") == 0)
    {
        return SESSION_TEST_BIAS;
    }
    if ((rt_strcmp(test_name, "temp") == 0) || (rt_strcmp(test_name, "temp_drift") == 0))
    {
        return SESSION_TEST_TEMP_DRIFT;
    }
    if (rt_strcmp(test_name, "arw") == 0)
    {
        return SESSION_TEST_ARW;
    }
    if (rt_strcmp(test_name, "vibration") == 0)
    {
        return SESSION_TEST_VIBRATION;
    }
    if (rt_strcmp(test_name, "shock") == 0)
    {
        return SESSION_TEST_SHOCK;
    }

    return SESSION_TEST_NONE;
}

static int session_dispatch_start(session_test_type_t test_type)
{
    switch (test_type)
    {
    case SESSION_TEST_BIAS:
        return test_bias_start();
    case SESSION_TEST_TEMP_DRIFT:
        return test_temp_drift_start();
    case SESSION_TEST_ARW:
        return test_arw_start();
    case SESSION_TEST_VIBRATION:
        return test_vibration_start();
    case SESSION_TEST_SHOCK:
        return test_shock_start();
    default:
        return -RT_ERROR;
    }
}

static int session_dispatch_stop(session_test_type_t test_type)
{
    switch (test_type)
    {
    case SESSION_TEST_BIAS:
        return test_bias_stop();
    case SESSION_TEST_TEMP_DRIFT:
        return test_temp_drift_stop();
    case SESSION_TEST_ARW:
        return test_arw_stop();
    case SESSION_TEST_VIBRATION:
        return test_vibration_stop();
    case SESSION_TEST_SHOCK:
        return test_shock_stop();
    default:
        return RT_EOK;
    }
}

int session_manager_init(void)
{
    g_session_state = SESSION_STATE_IDLE;
    g_session_test_type = SESSION_TEST_NONE;
    g_last_mark_temp_c = 0;
    return RT_EOK;
}

int session_start(session_test_type_t test_type)
{
    int result;

    if (g_session_state != SESSION_STATE_IDLE)
    {
        return -RT_EBUSY;
    }

    result = session_dispatch_start(test_type);
    if (result != RT_EOK)
    {
        return result;
    }

    g_session_test_type = test_type;
    g_session_state = SESSION_STATE_RUNNING;
    return RT_EOK;
}

int session_start_by_name(const char *test_name)
{
    session_test_type_t test_type = session_find_test_type(test_name);

    if (test_type == SESSION_TEST_NONE)
    {
        return -RT_ERROR;
    }

    return session_start(test_type);
}

int session_stop(void)
{
    int result;

    if (g_session_state == SESSION_STATE_IDLE)
    {
        return RT_EOK;
    }

    g_session_state = SESSION_STATE_STOPPING;
    result = session_dispatch_stop(g_session_test_type);
    g_session_test_type = SESSION_TEST_NONE;
    g_session_state = SESSION_STATE_IDLE;
    return result;
}

void session_complete_current(void)
{
    g_session_test_type = SESSION_TEST_NONE;
    g_session_state = SESSION_STATE_IDLE;
}

session_state_t session_get_state(void)
{
    return g_session_state;
}

session_test_type_t session_get_test_type(void)
{
    return g_session_test_type;
}

int session_mark_temperature(rt_int32_t temp_c)
{
    g_last_mark_temp_c = temp_c;
    return RT_EOK;
}
