#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "bsp_system.h"
#include "filter.h"
#include "PID.h"

extern PT1Filter_t motor_pt1;
extern PT1Filter_t motor_pt2;
extern PT1Filter_t motor_pt3;
extern PT1Filter_t motor_pt4;

extern int16_t motor_speed1;
extern int16_t motor_speed2;
extern int16_t motor_speed3;
extern int16_t motor_speed4;

extern PID motor_pid1;
extern PID motor_pid2;
extern PID motor_pid3;
extern PID motor_pid4;

void Encoder_Init(void);
void Encoder_Get_Counter(void);

#endif
