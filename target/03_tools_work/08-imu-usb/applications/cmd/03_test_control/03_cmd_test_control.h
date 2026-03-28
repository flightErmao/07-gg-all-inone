#ifndef APPLICATIONS_03_CMD_TEST_CONTROL_H
#define APPLICATIONS_03_CMD_TEST_CONTROL_H

#include <rtthread.h>

int cmd_test_control_start(const char *test_name);
int cmd_test_control_stop(void);
int cmd_test_control_mark_temperature(rt_int32_t temp_c);
int cmd_test_control_noise_prepare(void);
int cmd_test_control_noise_start(void);
int cmd_test_control_noise_status(void);
int cmd_test_control_noise_stop(void);

#endif
