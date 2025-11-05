#ifndef __PIDPX4_H
#define __PIDPX4_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
  float gain_p[3];
  float gain_i[3];
  float gain_d[3];
  float gain_ff[3];
  float lim_int[3];

  float rate_int[3];

  uint8_t sat_pos[3];
  uint8_t sat_neg[3];
} RateControlPX4;

void rcpx4_init(RateControlPX4 *rc);
void rcpx4_set_pid(RateControlPX4 *rc, const float P[3], const float I[3], const float D[3]);
void rcpx4_set_integrator_limit(RateControlPX4 *rc, const float lim_int[3]);
void rcpx4_set_ff(RateControlPX4 *rc, const float FF[3]);
void rcpx4_set_saturation(RateControlPX4 *rc, const uint8_t sat_pos[3], const uint8_t sat_neg[3]);
void rcpx4_set_pos_sat_axis(RateControlPX4 *rc, size_t axis, uint8_t is_saturated);
void rcpx4_set_neg_sat_axis(RateControlPX4 *rc, size_t axis, uint8_t is_saturated);
void rcpx4_reset_integral(RateControlPX4 *rc);
void rcpx4_reset_integral_axis(RateControlPX4 *rc, size_t axis);

/*
 * 计算一次控制输出。
 * 参数单位：
 *  - rate / rate_sp: deg/s
 *  - angular_accel: deg/s^2（若无加速度数据可传入 {0,0,0}）
 *  - dt: s
 *  - landed: true 则不积分
 * 输出 torque[3] 为未归一化力矩指令（根据增益维度）
 */
void rcpx4_update(RateControlPX4 *rc,
                  const float rate[3],
                  const float rate_sp[3],
                  const float angular_accel[3],
                  float dt,
                  uint8_t landed,
                  float torque[3]);

#endif /* __PIDPX4_H */


