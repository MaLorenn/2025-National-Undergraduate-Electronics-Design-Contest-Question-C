#include "motor.h"
#include "math.h"
#include "stdbool.h"
#include "tim.h"
#include "usart.h"

#define		ABS(x)		((x) > 0 ? (x) : -(x)) 
#define motor_xifeng 32.0f
#define delay 100

float record_angle[3] = {0, 0, 0}; 
extern bool motor_stop_flag;

extern float Motor_Cur_Pos_1;
extern float Motor_Cur_Pos_2;
//////////////////////////////////////////////////////////////

void setStep_1(uint8_t w1,uint8_t w2) 
{
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_11,w1);//pul
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_10,w2);//dir
}

void setStep_2(uint8_t w1,uint8_t w2) 
{
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,w1);//pul
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,w2);//dir
}

//void DelayNop(uint8_t i) //延时
//{
//	uint32_t 	j;
//	do
//	{
//		for(j=0;j<100;j++){;}
//	}while(i--);
//}

/* 微秒延时函数 */
void DelayNop(uint16_t us)
{
    __HAL_TIM_SET_COUNTER(&htim4, 0);  // 把TIM4的计数器清零
    while (__HAL_TIM_GET_COUNTER(&htim4) < us);  // 等待计数器的值达到us
}

void rightward_1(int steps)
{
    for(int i=0;i<steps;i++)
    {
        setStep_1(0, 1);
        DelayNop(delay);
        setStep_1(1, 1);
        DelayNop(delay);
        setStep_1(1, 0);
        DelayNop(delay);
        setStep_1(0, 0);
        DelayNop(delay);
    }
}

void leftward_1(int steps)
{

    for(int i=0;i<steps;i++)
    {
        setStep_1(0, 0);
        DelayNop(delay);
        setStep_1(1, 0);
        DelayNop(delay);
        setStep_1(1, 1);
        DelayNop(delay);
        setStep_1(0, 1);
        DelayNop(delay);
    }

}


void rightward_2(int steps)
{

    for(int i=0;i<steps;i++)
    {
        setStep_2(0, 1);
        DelayNop(delay);
        setStep_2(1, 1);
        DelayNop(delay);
        setStep_2(1, 0);
        DelayNop(delay);
        setStep_2(0, 0);
        DelayNop(delay);
    }
}

void leftward_2(int steps)
{

    for(int i=0;i<steps;i++)
    {
        setStep_2(0, 0);
        DelayNop(delay);
        setStep_2(1, 0);
        DelayNop(delay);
        setStep_2(1, 1);
        DelayNop(delay);
        setStep_2(0, 1);
        DelayNop(delay);
    }

}


void xrunstep(int dxstep)//dxstep为脉冲数
{
   
    if(dxstep > 0)
        rightward_1(dxstep);
    else
    {
        dxstep = -dxstep;
        leftward_1(dxstep);
 	}
}

void yrunstep(int dxstep)//dxstep为脉冲数
{
   
    if(dxstep > 0)
        rightward_2(dxstep);
    else
    {
        dxstep = -dxstep;
        leftward_2(dxstep);
 	}
}


void multi_angle_control(float angle1, float angle2, float angle3) {
    int pul_1 = 0;
    int pul_2 = 0;
    int pul_3 = 0;
    float temp[3] = {0, 0, 0}; // ????????????

    // ??
	if(angle1>90) 
		angle1=90;
	else if(angle1<-90)
		angle1=-90;

	if(angle2>70)
		angle2=70;
	else if(angle2<-70)
		angle2=-70;

	if(angle3>70)
		angle3=70;
	else if(angle3<-70)
		angle3=-70;

    temp[0] = angle1 - record_angle[0];
    temp[1] = angle2 - record_angle[1];
    temp[2] = angle3 - record_angle[2];

    pul_1 = (int)(temp[0] * motor_xifeng / 1.8f);
    pul_2 = (int)(temp[1] * motor_xifeng / 1.8f);
    pul_3 = (int)(temp[2] * motor_xifeng / 1.8f);

    // Bresenham直线插补算法实现
    int dx = ABS(pul_1);
    int dy = ABS(pul_2);
    int sx = pul_1 > 0 ? 1 : -1;
    int sy = pul_2 > 0 ? 1 : -1;
    int err = dx - dy;

    int max_steps = dx > dy ? dx : dy;
    if (max_steps == 0) return;
    // 记录当前步数
    int current_step = 0;
    
    // 暂停后继续的步进循环
    while (current_step < max_steps) {
    	
		while (motor_stop_flag)
		{
			__WFI();
		}
    	
    	
        int e2 = 2 * err;
        
        // X轴步进
        if (e2 > -dy) {
            err -= dy;
            xrunstep(sx);
        }
        
        // Y轴步进
        if (e2 < dx) {
            err += dx;
            yrunstep(sy);
        }
        
        current_step++;
    }
    // 更新记录角度（基于实际脉冲数）
    record_angle[0] += (pul_1 * 1.8f) / motor_xifeng;
    record_angle[1] += (pul_2 * 1.8f) / motor_xifeng;
    record_angle[2] += (pul_3 * 1.8f) / motor_xifeng;
	
//    // 四舍五入保持精度
//    record_angle[0] = roundf(record_angle[0] * 100) / 100;
//    record_angle[1] = roundf(record_angle[1] * 100) / 100;
//    record_angle[2] = roundf(record_angle[2] * 100) / 100;
	
}



void read_angle_1(void)
{
  uint8_t i = 0;
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[i] = 1; ++i;                   // 地址
  cmd[i] = 0x36; ++i; 
  cmd[i] = 0x6B; ++i;                   // 校验字节
  
  // 发送命令
	HAL_UART_Transmit(&huart4, (uint8_t *)cmd, i, 0xffff);   //串口发送三个字节数据，最大传输时间0xffff
  
	while(rxFrameFlag_1 == false);
	rxFrameFlag_1 = false;
	HAL_Delay(1);
	rxFrameFlag_1 = 0;
  
	if(rxCmd_1[0] == 1 && rxCmd_1[1] == 0x36 && rxCount_1 == 8)
   {

    // 转换成角度
    Motor_Cur_Pos_1 = (float)((uint32_t)(
                      ((uint32_t)rxCmd_1[3] << 24)    |
                      ((uint32_t)rxCmd_1[4] << 16)    |
                      ((uint32_t)rxCmd_1[5] << 8)     |
                      ((uint32_t)rxCmd_1[6] << 0)
                    )) * 360.0f / 65536.0f;
	
    // 符号
    if(rxCmd_1[2]) { Motor_Cur_Pos_1 = -Motor_Cur_Pos_1; }
   }
}


void read_angle_2(void)
{
  uint8_t i = 0;
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[i] = 1; ++i;                   // 地址
  cmd[i] = 0x36; ++i; 
  cmd[i] = 0x6B; ++i;                   // 校验字节
  
  // 发送命令
	HAL_UART_Transmit(&huart5, (uint8_t *)cmd, i, 0xffff);   //串口发送三个字节数据，最大传输时间0xffff
  
	while(rxFrameFlag_2 == false);
	rxFrameFlag_2 = false;
	HAL_Delay(1);
	rxFrameFlag_2 = 0;
  
	if(rxCmd_2[0] == 1 && rxCmd_2[1] == 0x36 && rxCount_2 == 8)
   {

    // 转换成角度
    Motor_Cur_Pos_2 = (float)((uint32_t)(
                      ((uint32_t)rxCmd_2[3] << 24)    |
                      ((uint32_t)rxCmd_2[4] << 16)    |
                      ((uint32_t)rxCmd_2[5] << 8)     |
                      ((uint32_t)rxCmd_2[6] << 0)
                    )) * 360.0f / 65536.0f;
	
    // 符号
    if(rxCmd_2[2]) { Motor_Cur_Pos_2 = -Motor_Cur_Pos_2; }
   }

}
