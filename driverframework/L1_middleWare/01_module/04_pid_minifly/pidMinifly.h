#ifndef __PID_MINIFLY_H
#define __PID_MINIFLY_H
#include <stdbool.h>

#define DEFAULT_PID_INTEGRATION_LIMIT 500.0  // 默认pid的积分限幅
#define DEFAULT_PID_OUTPUT_LIMIT 0.0         // 默认pid输出限幅，0为不限幅
#define DEFAULT_PID_INTEGRAL_RESET_THRESHOLD 0.1  // 默认积分清零阈值
#define DEFAULT_PID_INTEGRAL_DECAY_FACTOR 0.95    // 默认积分泄放系数，0.95表示每次衰减5%

typedef struct {
  float kp;
  float ki;
  float kd;
} pidInit_t;

typedef struct {
  pidInit_t roll;
  pidInit_t pitch;
  pidInit_t yaw;
} pidParam_t;

typedef struct {
  pidInit_t vx; /*X轴速度PID*/
  pidInit_t vy; /*Y轴速度PID*/
  pidInit_t vz; /*Z轴速度PID*/
  pidInit_t x;  /*X轴位置PID*/
  pidInit_t y;  /*Y轴位置PID*/
  pidInit_t z;  /*Z轴位置PID*/
} pidParamPos_t;


typedef struct {
  float desired;      //< set point
  float error;        //< error
  float prevError;    //< previous error
  float integ;        //< integral
  float deriv;        //< derivative
  float kp;           //< proportional gain
  float ki;           //< integral gain
  float kd;           //< derivative gain
  float outP;         //< proportional output (debugging)
  float outI;         //< integral output (debugging)
  float outD;         //< derivative output (debugging)
  float iLimit;       //< integral limit
  float outputLimit;  //< total PID output limit, absolute value. '0' means no limit.
  float integralResetThreshold;  //< integral reset threshold, when error is smaller than this, integral will be reset
                                 // to 0
  float integralDecayFactor;  //< integral decay factor, when error is small, integral will decay by this factor
  float dt;           //< delta-time dt
  float out;          //< out
} PidObject;

/*pid结构体初始化*/
void pidInit(PidObject* pid, const float desired, const pidInit_t pidParam, const float dt);
void pidSetIntegralLimit(PidObject* pid, const float limit); /*pid积分限幅设置*/
void pidSetOutputLimit(PidObject* pid, const float limit);
void pidSetDesired(PidObject* pid, const float desired); /*pid设置期望值*/
float pidUpdate(PidObject* pid, const float error);      /*pid更新*/
float pidGetDesired(PidObject* pid);                     /*pid获取期望值*/
bool pidIsActive(PidObject* pid);                        /*pid状态*/
void pidReset(PidObject* pid);                           /*pid结构体复位*/
void pidSetError(PidObject* pid, const float error);     /*pid偏差设置*/
void pidSetKp(PidObject* pid, const float kp);           /*pid Kp设置*/
void pidSetKi(PidObject* pid, const float ki);           /*pid Ki设置*/
void pidSetKd(PidObject* pid, const float kd);           /*pid Kd设置*/
void pidSetDt(PidObject* pid, const float dt);           /*pid dt设置*/
void pidSetIntegralResetThreshold(PidObject* pid, const float threshold); /*pid积分清零阈值设置*/
void pidSetIntegralDecayFactor(PidObject* pid, const float decayFactor);  /*pid积分泄放系数设置*/

#endif /* __PID_H */
