#ifndef __PID_H
#define __PID_H

typedef struct {
  float kp;
  float ki;
  float kd;
  float ki_limit;
  float dt;
  float target;
  float LastError;
  float PrevError;
  float integral;
  float current;
  float error;
  float out;
} PID;

void PID_init(PID *pid, float kp, float ki, float kd, float dt, float ki_limit);
float PID_location(PID *pid, float current_value);

#endif