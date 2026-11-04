#include "code.h"

void LED_ON(void)
{
	HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
}

void LED_OFF(void)
{
  HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
}

uint8_t key1_flag = 1;
uint8_t key2_flag = 0;
uint8_t key3_flag = 0;
uint8_t key4_flag = 0;

void key_scan(void)
{
	uint32_t Timetick = HAL_GetTick();
	static uint32_t last_tick;
	
	if(Timetick - last_tick <10) return;
	last_tick = Timetick;	
	static bool key1_state,key2_state,key3_state,key4_state;
	static bool key1_last,key2_last,key3_last,key4_last;
	
	key1_state = HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin);key2_state = HAL_GPIO_ReadPin(KEY2_GPIO_Port,KEY2_Pin);
	key3_state = HAL_GPIO_ReadPin(KEY3_GPIO_Port,KEY3_Pin);key4_state = HAL_GPIO_ReadPin(KEY4_GPIO_Port,KEY4_Pin);
	
	if(key1_last & (!key1_state))
	{
		key1_flag ++;
		if(key1_flag > 3)
		{
			key1_flag = 1;
		}
	}
	if(key2_last & (!key2_state))
	{
		key2_flag ++;
	}
	if(key3_last & (!key3_state))
	{
		key3_flag ++;
	}
	if(key4_last & (!key4_state))
	{
		key4_flag ++;
	}
	key1_last = key1_state;key2_last = key2_state;
	key3_last = key3_state;key4_last = key4_state;
}

void Buzzer_ON()
{
	HAL_GPIO_WritePin(Buzzer_GPIO_Port,Buzzer_Pin,GPIO_PIN_SET);
}

void Buzzer_OFF()
{
	HAL_GPIO_WritePin(Buzzer_GPIO_Port,Buzzer_Pin,GPIO_PIN_RESET);
}

void main_proc()
{
	key_scan(); 
	page_proc();
}

void loop()
{
	main_proc();

}