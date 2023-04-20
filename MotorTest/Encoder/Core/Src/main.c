/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 小车主控程序
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "stdio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//左轮
#define LIN_Port GPIOB
#define LIN1 GPIO_PIN_0
#define LIN2 GPIO_PIN_1
//右轮
#define RIN_Port GPIOA
#define RIN1 GPIO_PIN_6
#define RIN2 GPIO_PIN_7
//定时器以及通道
#define motor_TIM &htim1
#define rightMotorChannel TIM_CHANNEL_1
#define leftMotorChannel TIM_CHANNEL_2

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
short encoderPulse[2]={0};
typedef struct
{
	//相关速度PID参数
	float Kp;
	float Ki;
	float Kd;
	float Ur;				//限幅值
	
  int EN;                 //PID使能
	float Un;					//期望输出值
	float En_1;				//上一次的误差值
	float En_2;				//上上次的误差值
	int PWM;				//输出PWM值
}PID_InitDefStruct;

void PID_Init(PID_InitDefStruct* p) //PID值初始化
{
	p->Kp = 30.0;
	p->Ki = 7.25;
	p->Kd = 5.0;
	p->Ur = 2000;
	p->EN = 1;
	p->Un = 0;
	p->En_1 = 0;
	p->En_2 = 0;
	p->PWM = 0;
}

void Velocity_PID(float targetVelocity,float currentVelocity,PID_InitDefStruct* p)
{
	if(p->EN == 1)
	{
		float En = targetVelocity - currentVelocity;//误差值                                                     
	
		p->Un += p->Kp*(En - p->En_1) + p->Ki*En + p->Kd*(En - 2*p->En_1 + p->En_2); //增量式PID
		
		p->En_2 = p->En_1;
		p->En_1 = En;
		
		p->PWM = (int)p->Un;
		
		/*输出限幅*/
		if(p->PWM > p->Ur) p->PWM = p->Ur;
		if(p->PWM < -p->Ur) p->PWM = -p->Ur;
	}
	else
	{
		PID_Init(p);
	}
}
PID_InitDefStruct leftMotor_PID;
PID_InitDefStruct rightMotor_PID;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
*    @brief 控制电机进行正转、反转和停转
*    @param None
*    @retval None
*/
void LeftMotor_Go() 	//左电机正转 LIN1=1 LIN2=0 即PB0高电平PB1低电平
{
  HAL_GPIO_WritePin(LIN_Port, LIN1, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LIN_Port, LIN2, GPIO_PIN_RESET);
}
void LeftMotor_Back() //左电机反转 LIN1=0 LIN2=1 即PB0低电平 PB1高电平
{
  HAL_GPIO_WritePin(LIN_Port, LIN1, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LIN_Port, LIN2, GPIO_PIN_SET);
}
void LeftMotor_Stop() //左电机停转 LIN1和LIN2电平相同
{
  HAL_GPIO_WritePin(LIN_Port, LIN1, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LIN_Port, LIN2, GPIO_PIN_SET);
}
//-------------------------------------------------
void RightMotor_Go() 	//右电机正转 RIN1=0 RIN2=1 即PA6低电平 PA7高电平
{
  HAL_GPIO_WritePin(RIN_Port, RIN1, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RIN_Port, RIN2, GPIO_PIN_SET);
}
void RightMotor_Back() //右电机反转 RIN1=1 RIN2=0 即PA6高电平 PA7低电平
{
  HAL_GPIO_WritePin(RIN_Port, RIN1, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RIN_Port, RIN2, GPIO_PIN_RESET);
}
void RightMotor_Stop() //右电机停转 RIN1和RIN2电平相同
{
  HAL_GPIO_WritePin(RIN_Port, RIN1, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RIN_Port, RIN2, GPIO_PIN_SET);
}

/**
*    @brief 控制电机进行速度控制
*    @param 运动方向，左右电机的PWM值
*    @retval None
*/
void MotorControl(char motorDirection, int leftMotorPWM, int rightMotorPWM)
{
  switch (motorDirection)
  {
  case 0:   //前行
    LeftMotor_Go();
    RightMotor_Go();
    __HAL_TIM_SET_COMPARE(motor_TIM, rightMotorChannel, rightMotorPWM);
    __HAL_TIM_SET_COMPARE(motor_TIM, leftMotorChannel, leftMotorPWM);
    break;
  case 1:   //后退
    LeftMotor_Back();
    RightMotor_Back();
    __HAL_TIM_SET_COMPARE(motor_TIM, rightMotorChannel, rightMotorPWM);
    __HAL_TIM_SET_COMPARE(motor_TIM, leftMotorChannel, leftMotorPWM);
    break;
  case 2:   //停车
    LeftMotor_Stop();
    RightMotor_Stop();
    __HAL_TIM_SET_COMPARE(motor_TIM, rightMotorChannel, 0);
    __HAL_TIM_SET_COMPARE(motor_TIM, leftMotorChannel, 0);
    break;
  default: break;
  }
}
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
  PID_Init(&leftMotor_PID);
  PID_Init(&rightMotor_PID);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim2);      //设定50ms中断
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);   //开启TIM1_CH1 PWM -- 右轮
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);   //开启TIM1_CH2 PWM -- 左轮
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);   //开启编码器定时
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2);   //开启编码器定时
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1);   //开启编码器定时
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2);   //开启编码器定时
	
	__HAL_TIM_ENABLE_IT(&htim3,TIM_IT_UPDATE);      //开启编码器定时器更新中断，防溢出处理
	__HAL_TIM_ENABLE_IT(&htim4,TIM_IT_UPDATE);      //开启编码器定时器更新中断，防溢出处理
  
  __HAL_TIM_SET_COUNTER(&htim3, 30000);           //将编码器定时器初始，设定为30000
	__HAL_TIM_SET_COUNTER(&htim4, 30000);           //将编码器定时器初始，设定为30000

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  { 
    // MotorControl(0,900,900);  //直行
    // HAL_Delay(2000);
    // MotorControl(2,0,0);      //停止
    // HAL_Delay(2000);
    // MotorControl(1,5000,5000);  //后退
    // HAL_Delay(2000);
    // MotorControl(0,0,2000);    //前进左转
    // HAL_Delay(2000);
    // MotorControl(0,2000,0);    //前进右转
    // HAL_Delay(2000);
    // MotorControl(1,0,2000);    //左转退回
    // HAL_Delay(2000);
    // MotorControl(1,2000,0);    //右转退回
    // HAL_Delay(2000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//printf重定向
int fputc(int ch, FILE *f)
{
 uint8_t temp[1] = {ch};
 HAL_UART_Transmit(&huart2, temp, 1, 2);
 return ch;
}

//编码器测速-------------------------------------------------------------
/**
  * @brief  读取定时器2和定时器3的计数值(编码器脉冲值),TIM3对应右轮，TIM对应左轮
  * @retval None
  */
void GetEncoderPulse()
{    
  encoderPulse[0] = -((short)__HAL_TIM_GET_COUNTER(&htim3)); //配合小车轮子运动方向，进行取反操作
  encoderPulse[1] = -((short)__HAL_TIM_GET_COUNTER(&htim4));

  __HAL_TIM_GET_COUNTER(&htim3) = 0;   //计数值重新清零
  __HAL_TIM_GET_COUNTER(&htim4) = 0;  
}

/**
  * @brief  根据得到的编码器脉冲值计算速度 单位:m/s
  * @retval 速度值
  */
float CalActualSpeed(int pulse)
{
    return (float)(0.003925 * pulse);
}
//中断处理函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)  //定时器2中断回调函数，每50ms调用一次
{
  float c_leftSpeed, c_rightSpeed; 
  if(htim->Instance == TIM2)
  {
    GetEncoderPulse(); 
    c_leftSpeed = CalActualSpeed(encoderPulse[1]);   //获得当前的速度值
    c_rightSpeed = CalActualSpeed(encoderPulse[0]);
    printf("leftSpeed = %.2f m/s, rightSpeed = %.2f m/s\n\r", c_leftSpeed, c_rightSpeed);
    
    Velocity_PID(0.7,c_leftSpeed,&leftMotor_PID); //左电机PID计算
    Velocity_PID(0.7,c_rightSpeed,&rightMotor_PID); //右电机PID计算

    MotorControl(0,leftMotor_PID.PWM,rightMotor_PID.PWM);
    // printf("LeftMotor_PID.pwm_add = %.2f m/s, RightMotor_PID.pwm_add = %.2f m/s\n\r", LeftMotor_PID.pwm_add, RightMotor_PID.pwm_add);
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
