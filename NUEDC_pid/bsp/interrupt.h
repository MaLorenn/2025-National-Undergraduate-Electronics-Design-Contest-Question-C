#ifndef _INTERRUPT_H_
#define _INTERRUPT_H_

#include "main.h"
#include "stdbool.h"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);


struct keys{
	bool key_sta;
	unsigned char judge_sta;
	bool singal_flag;
	bool long_flag;
	bool twice_flag;
	unsigned int time_count;
};
	
struct sur{
uint16_t d_flag;
uint16_t cnt;
};

#endif
