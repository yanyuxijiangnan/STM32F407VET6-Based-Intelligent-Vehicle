#include "PID.h"

void PID_init(PID *pid, float kp, float ki, float kd, float dt, float ki_limit)
{
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->dt = dt;
  pid->ki_limit = ki_limit;
  pid->target = 0.0f;
  pid->LastError = 0.0f;
  pid->PrevError = 0.0f;
  pid->integral = 0.0f;
  pid->current = 0.0f;
  pid->error = 0.0f;
  pid->out = 0.0f;
}

float PID_location(PID *pid, float current_value)
{
  pid->current = current_value;
  pid->error = pid->target - current_value;

  pid->integral += pid->error * pid->dt;
  if (pid->integral > pid->ki_limit)
  {
    pid->integral = pid->ki_limit;
  }
  else if (pid->integral < -pid->ki_limit)
  {
    pid->integral = -pid->ki_limit;
  }

  float derivative = 0.0f;
  if (pid->dt > 0.0f)
  {
    derivative = (pid->error - pid->LastError) / pid->dt;
  }

  pid->out = pid->kp * pid->error
           + pid->ki * pid->integral
           + pid->kd * derivative;
  pid->LastError = pid->error;
  return pid->out;
}