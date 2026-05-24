#include "interrupt.h"
#include "tim.h"
#include "pid.h"

extern uint32_t Pulse_Cnt_1;
extern uint32_t Pulse_exp_Cnt_1;
extern uint8_t motor_running_1;     // 电机运行状态（0-停止，1-运行）

extern uint32_t Pulse_Cnt_2;
extern uint32_t Pulse_exp_Cnt_2;
extern uint8_t motor_running_2;     // 电机运行状态（0-停止，1-运行）

///////////////////////////////////**按键**///////////////////////////////////////////////////////////////
struct keys key[4]={0,0,0,0,0,0};
bool motor_stop_flag=0;
extern float Motor_Cur_Pos_1;
extern float Motor_Cur_Pos_2;
extern float record_angle[2]; 

extern int rx_target_pixel_x;
extern int rx_target_pixel_y;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	key[0].key_sta=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12);
	key[1].key_sta=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13);
	key[2].key_sta=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14);
	key[3].key_sta=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15);
	if(htim->Instance==TIM1)
	{
		for(int i=0;i<4;i++)
		{
			switch(key[i].judge_sta)
			{
				case 0:
					if(key[i].key_sta==0)
					{
						key[i].judge_sta=1;
						key[i].time_count=0;
					}
					break;
				case 1:
					if(key[i].key_sta==0)
						key[i].judge_sta=2;
					else
						key[i].judge_sta=0;
					break;
				case 2:
					if(key[i].key_sta==1)
					{
						key[i].judge_sta=0;
						if(key[i].time_count<100)
						{
							key[i].singal_flag=1;
						}
					}
					else
					{
						key[i].time_count++;
						if(key[i].time_count>100){
							key[i].long_flag=1;
						}
					}
					break;
			}
		}
		
	
	}

	if(htim->Instance == TIM6)
	{
		static uint8_t time6_10ms = 0;
		if(rx_target_pixel_x!=0 && rx_target_pixel_y!=0)
		{
			if(++time6_10ms == 1)
			{
				PID_ControlLoop_X();
				time6_10ms = 0;
			}
		}
	}
	if(htim->Instance == TIM7)
	{
		static uint8_t time7_10ms = 0;
		if(rx_target_pixel_x!=0 && rx_target_pixel_y!=0)
		{
			if(++time7_10ms == 1)
			{
				PID_ControlLoop_Y();
				time7_10ms = 0;
			}
		}
	}
		/*每输出一个脉冲进入一次中断，Pulse_Cnt用于计数已经产生的脉冲个数*/
	/*Pulse_exp_Cnt为需要产生的脉冲个数*/
	/*当Pulse_Cnt与Pulse_exp_Cnt为相等时，表示已经产生了预期数目的脉冲数，从而停止PWM输出*/
	if(htim->Instance == TIM8 )
	{
		Pulse_Cnt_1++;
		if(Pulse_Cnt_1> Pulse_exp_Cnt_1-1)
		{
			 HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_1);
			 HAL_TIM_Base_Stop_IT(&htim8);
			 
			 __HAL_TIM_SET_COUNTER(&htim8, 0);
			 motor_running_1 = 0;
			 Pulse_Cnt_1=0;
		}
	}
	
		/*每输出一个脉冲进入一次中断，Pulse_Cnt用于计数已经产生的脉冲个数*/
	/*Pulse_exp_Cnt为需要产生的脉冲个数*/
	/*当Pulse_Cnt与Pulse_exp_Cnt为相等时，表示已经产生了预期数目的脉冲数，从而停止PWM输出*/
	if(htim->Instance == TIM3 )
	{
		Pulse_Cnt_2++;
		if(Pulse_Cnt_2> Pulse_exp_Cnt_2-1)
		{
			 HAL_TIM_Base_Stop_IT(&htim3);
			 HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_2);
			 __HAL_TIM_SET_COUNTER(&htim3, 0);
			 motor_running_2 = 0;
			 Pulse_Cnt_2=0;
		}
	}

}
