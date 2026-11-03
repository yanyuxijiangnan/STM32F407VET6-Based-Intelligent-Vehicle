#ifndef __CODE_H__
#define __CODE_H__

#include "bsp_system.h"

extern uint8_t key1_flag;
extern uint8_t key2_flag;
extern uint8_t key3_flag;
extern uint8_t key4_flag;


void LED_ON(void);
void LED_OFF(void);
void key_scan(void);
void Buzzer_ON();
void Buzzer_OFF();
void main_proc();
void loop();

#endif