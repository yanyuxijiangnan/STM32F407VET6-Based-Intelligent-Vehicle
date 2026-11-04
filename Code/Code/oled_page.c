#include "oled_page.h"

uint8_t cnt = 0;

void page_proc(void)
{
	if(key1_flag == 1)
	{
		OLED_NewFrame(); 
		OLED_PrintASCIIString(0,0,"The First Page",&afont16x8,OLED_COLOR_NORMAL);
		
//		OLED_PrintASCIIString(0,10,"base_speed",&afont12x6,OLED_COLOR_NORMAL);
//		OLED_PrintASCIIString(0,20,"wz_limit",&afont12x6,OLED_COLOR_NORMAL);
//		OLED_PrintASCIIString(0,30,"kp:",&afont12x6,OLED_COLOR_NORMAL);
//		OLED_PrintFloat(30,30,line_steer_pid.kp,2,&afont12x6,OLED_COLOR_NORMAL);
//		OLED_PrintASCIIString(0,40,"ki:",&afont12x6,OLED_COLOR_NORMAL);
//		OLED_PrintFloat(30,40,line_steer_pid.ki,2,&afont12x6,OLED_COLOR_NORMAL);
//		OLED_PrintASCIIString(0,50,"kd:",&afont12x6,OLED_COLOR_NORMAL);
//		OLED_PrintFloat(30,50,line_steer_pid.kd,2,&afont12x6,OLED_COLOR_NORMAL);
		
		OLED_ShowFrame();
	}    
	else if(key1_flag == 2)
	{
		OLED_NewFrame();
		OLED_PrintASCIIString(0,0,"The Second Page",&afont16x8,OLED_COLOR_NORMAL);
		OLED_ShowFrame();	 
	}  
	else if(key1_flag == 3)
	{
		OLED_NewFrame();
		OLED_PrintASCIIString(0,0,"The Third Page",&afont16x8,OLED_COLOR_NORMAL);
		OLED_ShowFrame();
	}
}