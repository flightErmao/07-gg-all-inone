#include "pidMinifly.h"
#include <math.h>

void pidInit(PidObject* pid, const float desired, const pidInit_t pidParam, const float dt) {
  pid->error = 0;
  pid->prevError = 0;
  pid->integ = 0;
  pid->deriv = 0;
  pid->desired = desired;
  pid->kp = pidParam.kp;
  pid->ki = pidParam.ki;
  pid->kd = pidParam.kd;
  pid->iLimit = DEFAULT_PID_INTEGRATION_LIMIT;
  pid->outputLimit = DEFAULT_PID_OUTPUT_LIMIT;
  pid->integralResetThreshold = DEFAULT_PID_INTEGRAL_RESET_THRESHOLD;
  pid->integralDecayFactor = DEFAULT_PID_INTEGRAL_DECAY_FACTOR;
  pid->dt = dt;
}

float pidUpdate(PidObject* pid, const float error) {
  float output;

  pid->error = error;

  // Integral decay logic: when error is smaller than threshold, decay integral instead of reset
  if (fabsf(pid->error) < pid->integralResetThreshold) {
    // Apply decay factor to gradually reduce integral
    pid->integ *= pid->integralDecayFactor;
  } else {
    // Normal integral accumulation when error is significant
    pid->integ += pid->error * pid->dt;
  }

  // Integral limit
  if (pid->integ > pid->iLimit) {
    pid->integ = pid->iLimit;
  } else if (pid->integ < -pid->iLimit) {
    pid->integ = -pid->iLimit;
  }

  pid->deriv = (pid->error - pid->prevError) / pid->dt;

  pid->outP = pid->kp * pid->error;
  pid->outI = pid->ki * pid->integ;
  pid->outD = pid->kd * pid->deriv;

  output = pid->outP + pid->outI + pid->outD;

  // Output limit
  if (pid->outputLimit != 0) {
    if (output > pid->outputLimit)
      output = pid->outputLimit;
    else if (output < -pid->outputLimit)
      output = -pid->outputLimit;
  }

  pid->prevError = pid->error;

  pid->out = output;
  return output;
}

void pidSetIntegralLimit(PidObject* pid, const float limit) { pid->iLimit = limit; }

void pidSetOutputLimit(PidObject* pid, const float limit) { pid->outputLimit = limit; }

void pidSetError(PidObject* pid, const float error) { pid->error = error; }

void pidSetDesired(PidObject* pid, const float desired) { pid->desired = desired; }

float pidGetDesired(PidObject* pid) { return pid->desired; }

void pidSetKp(PidObject* pid, const float kp) { pid->kp = kp; }

void pidSetKi(PidObject* pid, const float ki) { pid->ki = ki; }

void pidSetKd(PidObject* pid, const float kd) { pid->kd = kd; }

void pidSetDt(PidObject* pid, const float dt) { pid->dt = dt; }

void pidSetIntegralResetThreshold(PidObject* pid, const float threshold) { pid->integralResetThreshold = threshold; }

void pidSetIntegralDecayFactor(PidObject* pid, const float decayFactor) { pid->integralDecayFactor = decayFactor; }

void pidReset(PidObject* pid) {
  pid->error = 0;
  pid->prevError = 0;
  pid->integ = 0;
  pid->deriv = 0;
}
