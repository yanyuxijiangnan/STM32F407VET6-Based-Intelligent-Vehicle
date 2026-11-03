#include "Encoder.h"
#include "motor.h"

int16_t motor_speed1 = 0;
int16_t motor_speed2 = 0;
int16_t motor_speed3 = 0;
int16_t motor_speed4 = 0;

PT1Filter_t motor_pt1;
PT1Filter_t motor_pt2;
PT1Filter_t motor_pt3;
PT1Filter_t motor_pt4;
 
PID motor_pid1;
PID motor_pid2;
PID motor_pid3;
PID motor_pid4;

/* 开环满占空比时约每窗脉冲，用于映射到 0~100 */
#define ENC_FULL_SPEED_A 3330.0f
#define ENC_FULL_SPEED_D 3430.0f

static float Encoder_ToUnit(int16_t pulse, float full_pulse)
{
  float spd = ((float)pulse) * 100.0f / full_pulse;
  if (spd < 0.0f) spd = -spd;
  if (spd > 100.0f) spd = 100.0f;
  return spd; 
}

static float Encoder_ClampDuty(float u)
{
  if (u > (float)MAX_SPEED) u = (float)MAX_SPEED;
  if (u < (float)MIN_SPEED) u = (float)MIN_SPEED;
  return u;
}

void Encoder_Init(void)
{
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
}

void Encoder_Get_Counter(void)
{
  motor_speed1 = (int16_t)__HAL_TIM_GetCounter(&htim2);
  motor_speed2 = (int16_t)__HAL_TIM_GetCounter(&htim3);
  motor_speed3 = (int16_t)__HAL_TIM_GetCounter(&htim4);
  motor_speed4 = (int16_t)__HAL_TIM_GetCounter(&htim5);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance != TIM7)
  {
    return;
  }

  /* 后退模式：开环维持后退占空比，禁止正转 PID 覆盖 */
  if (motor_run_mode == MOTOR_MODE_BACK)
  {
    motor_speed1 = (int16_t)__HAL_TIM_GetCounter(&htim2);
    PT1Filter_Apply(&motor_pt1, Encoder_ToUnit(motor_speed1, ENC_FULL_SPEED_A));
    __HAL_TIM_SET_COUNTER(&htim2, 0);

    motor_speed2 = (int16_t)__HAL_TIM_GetCounter(&htim3);
    PT1Filter_Apply(&motor_pt2, Encoder_ToUnit(motor_speed2, ENC_FULL_SPEED_A));
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    motor_speed3 = (int16_t)__HAL_TIM_GetCounter(&htim4);
    PT1Filter_Apply(&motor_pt3, Encoder_ToUnit(motor_speed3, ENC_FULL_SPEED_D));
    __HAL_TIM_SET_COUNTER(&htim4, 0);

    motor_speed4 = (int16_t)__HAL_TIM_GetCounter(&htim5);
    PT1Filter_Apply(&motor_pt4, Encoder_ToUnit(motor_speed4, ENC_FULL_SPEED_D));
    __HAL_TIM_SET_COUNTER(&htim5, 0);

    Motor_ApplyBackwardDuty(motor_cmd_back);
    return;
  }

  /* 同步左右目标：右 A/B，左 D/C */
  motor_pid1.target = (float)motor_cmd_R;
  motor_pid4.target = (float)motor_cmd_L;

  /* A 右后：闭环 */
  motor_speed1 = (int16_t)__HAL_TIM_GetCounter(&htim2);
  {
    float spd = Encoder_ToUnit(motor_speed1, ENC_FULL_SPEED_A);
    PT1Filter_Apply(&motor_pt1, spd);
    PID_location(&motor_pid1, spd);
    float u = Encoder_ClampDuty(motor_pid1.target + motor_pid1.out);
    Motor_SetWheelSpeed('A', (uint16_t)u);
  }
  __HAL_TIM_SET_COUNTER(&htim2, 0);

  /* B 右前：开环跟右目标 */
  motor_speed2 = (int16_t)__HAL_TIM_GetCounter(&htim3);
  PT1Filter_Apply(&motor_pt2, Encoder_ToUnit(motor_speed2, ENC_FULL_SPEED_A));
  Motor_SetWheelSpeed('B', motor_cmd_R);
  __HAL_TIM_SET_COUNTER(&htim3, 0);

  /* C 左前：开环跟左目标 */
  motor_speed3 = (int16_t)__HAL_TIM_GetCounter(&htim4);
  PT1Filter_Apply(&motor_pt3, Encoder_ToUnit(motor_speed3, ENC_FULL_SPEED_D));
  Motor_SetWheelSpeed('C', motor_cmd_L);
  __HAL_TIM_SET_COUNTER(&htim4, 0);

  /* D 左后：闭环 */
  motor_speed4 = (int16_t)__HAL_TIM_GetCounter(&htim5);
  {
    float spd = Encoder_ToUnit(motor_speed4, ENC_FULL_SPEED_D);
    PT1Filter_Apply(&motor_pt4, spd);
    PID_location(&motor_pid4, spd);
    float u = Encoder_ClampDuty(motor_pid4.target + motor_pid4.out);
    Motor_SetWheelSpeed('D', (uint16_t)u);
  }
  __HAL_TIM_SET_COUNTER(&htim5, 0);
}
