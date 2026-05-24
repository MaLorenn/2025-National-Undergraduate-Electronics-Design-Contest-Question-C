/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "interrupt.h"
#include "stdlib.h"
#include "math.h"

#define motor_xifeng 64.0f
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float Motor_Cur_Pos_1 = 0.0f;
float Motor_Cur_Pos_2 = 0.0f;

//uart1 for nano/windows uart2 for uart_screen
char rxdata_1[70]={0};
uint8_t rxdat_1;
uint8_t rx_pointer_1=0;

extern struct keys key[4];

int rx_target_pixel_x ;      // 目标像素X
int rx_target_pixel_y;      // 目标像素Y


uint32_t Pulse_Cnt_1=0;
uint32_t Pulse_exp_Cnt_1=0;
uint8_t motor_running_1 = 0;     // 电机运行状态（0-停止，1-运行）

uint32_t Pulse_Cnt_2=0;
uint32_t Pulse_exp_Cnt_2=0;
uint8_t motor_running_2 = 0;     // 电机运行状态（0-停止，1-运行）

uint8_t jy62[33]={0};
extern float angl[3];
char myYAWCMD[3] = {0XFF,0XAA,0X52};

float angle_test=0;
uint8_t laser_flag=0;

int tiaozhen_angle=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

void Motor1_Set_angle(float angle) //一个电机转动指定角度，用之前先停止pwm输出
{
	if (motor_running_1 || angle==0) return;  // 若正在运行，忽略新指令
	
	int temp = (int)(angle * motor_xifeng / 1.8f);
	
	if(temp>=0)	
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_10,1);
	else 	
	{
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_10,0);
		temp=-temp;
	}
	
	Pulse_exp_Cnt_1 = temp;
	motor_running_1 = 1;          // 标记运行状态

	/*开启定时器更新中断*/
	HAL_TIM_Base_Start_IT(&htim8);
	/*开始输出PWM*/
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);	
}


void Motor2_Set_angle(float angle) //一个电机转动指定角度，用之前先停止pwm输出
{
	if (motor_running_2 || angle==0) return;  // 若正在运行，忽略新指令
	
	int temp = (int)(angle * motor_xifeng / 1.8f);
	
	if(temp>=0)	
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,1);
	else 	
	{
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,0);
		temp=-temp;
	}
	
	Pulse_exp_Cnt_2 = temp;
	motor_running_2 = 1;          // 标记运行状态

	/*开启定时器更新中断*/
	HAL_TIM_Base_Start_IT(&htim3);
	/*开始输出PWM*/
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);	
}


// 返回值单位：角度（float），正值 = 顺时针 CW，负值 = 逆时针 CCW
float nearest_to_zero_float(float angle) {
    // 归一化角度到 [0, 360)
    while (angle >= 360.0f) angle -= 360.0f;
    while (angle < 0.0f) angle += 360.0f;

    // 计算顺时针（CW）和逆时针（CCW）角度
    float cw = 360.0f - angle;  // 顺时针回零
    float ccw = -angle;         // 逆时针回零

    // 选择绝对值更小的方向
    if (cw <= -ccw) {
        return cw;   // 顺时针回零
    } else {
        return ccw;  // 逆时针回零
    }
}

void uart_rx_proc_1(void);

void key_prc(void);
void read_angle_1(void);
void read_angle_2(void);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	__HAL_UART_CLEAR_IDLEFLAG(&huart4); 											// ���IDLE��־
	__HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE); 							// ʹ�ܴ�UART4 IDLE�ж�
  HAL_UART_Receive_DMA(&huart4, (uint8_t *)rxCmd_1, CMD_LEN_1); // ����DMA����ģʽ
  
	__HAL_UART_CLEAR_IDLEFLAG(&huart5); 											// ���IDLE��־
	__HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE); 							// ʹ�ܴ�UART5 IDLE�ж�
  HAL_UART_Receive_DMA(&huart5, (uint8_t *)rxCmd_2, CMD_LEN_2); // ����DMA����ģʽ    
  
  
    __HAL_UART_ENABLE_IT(&huart2,UART_IT_IDLE); //start NVIC jy62
	HAL_UART_Receive_DMA(&huart2,jy62,33);
  HAL_UART_Transmit_DMA(&huart2,(uint8_t*)myYAWCMD,3);
  
  
	//start uart receive
	HAL_UART_Receive_IT(&huart1,&rxdat_1,1);

	//key scan timer
	HAL_TIM_Base_Start_IT(&htim1);
	
//	HAL_Delay(4000); //等待接收
	
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_9,0);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,0);	
	
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_10,1);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,1);	
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  
	 if(rx_pointer_1!=0)
	 {
		int temp=rx_pointer_1;
		HAL_Delay(1);	
		if(temp==rx_pointer_1)
			uart_rx_proc_1();
	 }
	 
	key_prc();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

//uart for windows
void uart_rx_proc_1()
{
	if(rx_pointer_1>0)
	{

		if(rxdata_1[0]=='a' && rxdata_1[1]!='-' && rxdata_1[rx_pointer_1-1]=='b')
		{
			sscanf(rxdata_1,"a%d,%db",&rx_target_pixel_x,&rx_target_pixel_y);
			float error_x = rx_target_pixel_x - 400;
			float error_y = rx_target_pixel_y - 300;
			if (fabsf(error_x) <= 2) 
			{
					HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
			}
			if (fabsf(error_y) <= 2)
			{
				HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
			}			
		}
		if(rxdata_1[0]=='a' && rxdata_1[1]=='-' && rxdata_1[rx_pointer_1-1]=='b' && rx_target_pixel_x!=0 && rx_target_pixel_y!=0)
		{
			rx_target_pixel_x = 400;
			rx_target_pixel_y = 300;
		}
		
	}
	rx_pointer_1=0;
	memset(rxdata_1,0,70);
}

void key_prc(void)
{
	if(key[0].singal_flag==1) //返回上电零点
	{
		__HAL_TIM_SET_PRESCALER(&htim8, 100 - 1); 
		__HAL_TIM_SET_PRESCALER(&htim3, 100 - 1);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_9,1);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,1);	
		read_angle_1();
		read_angle_2();
		Motor1_Set_angle(-Motor_Cur_Pos_1 + tiaozhen_angle);
		Motor2_Set_angle(-Motor_Cur_Pos_2);
		while (motor_running_1 || motor_running_2)  // 等待动作完成
		{
			printf("");
		}		
//		//read jy62 angle
//		// 然后转到指向靶纸正前方
//		
//		float temp = angl[2];
//		Motor1_Set_angle(nearest_to_zero_float(temp));	
//		while (motor_running_1)
//		{
//			printf("");
//		}
		
//		HAL_TIM_Base_Start_IT(&htim6);				//开启pid
//		HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
//		HAL_TIM_Base_Start_IT(&htim7);
//		HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
//		key[0].singal_flag=0;
	}
	
	if(key[1].singal_flag==1) 
	{
		tiaozhen_angle -= 30;    //向右调整
		key[1].singal_flag=0;
	}
	
	if(key[2].singal_flag==1)  
	{

		tiaozhen_angle += 30;   //向左调整
		key[2].singal_flag=0;
	}	
	
	if(key[3].singal_flag==1)  //急停按键
	{
		HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_2);
		
		key[3].singal_flag=0;
	}	
}


//UART_NVIC
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
		rxdata_1[rx_pointer_1++]=rxdat_1;
		HAL_UART_Receive_IT(&huart1,&rxdat_1,1);	
	}
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
//  HAL_UART_Transmit_DMA(&huart4, (uint8_t *)cmd, i);
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
//	HAL_UART_Transmit_DMA(&huart5, (uint8_t *)cmd, i);
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
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
