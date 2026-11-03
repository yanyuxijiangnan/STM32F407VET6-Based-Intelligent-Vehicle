#ifndef __LINE_FOLLOW_H__
#define __LINE_FOLLOW_H__

#include <stdint.h>
#include "PID.h"

/* line_state (VOFA readable) */
#define LF_FOLLOW       0U
#define LF_BACKUP       1U
#define LF_SEARCH_LOCK  2U  /* locked-dir fixed SEARCH_WZ sweep, no backup */
#define LF_SWEEP_L      3U
#define LF_SWEEP_R      4U
#define LF_COMMIT       5U
#define LF_STOP         6U

void LineFollow_Init(void);
void LineFollow_Update(void);

extern PID line_steer_pid;
extern volatile float line_error;
extern volatile float line_wz;
extern volatile uint16_t line_vL;
extern volatile uint16_t line_vR;
extern volatile uint8_t line_enabled;
extern volatile uint8_t line_state;

#endif