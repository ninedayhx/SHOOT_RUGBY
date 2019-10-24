/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
/* USER CODE BEGIN EV */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

uint8_t overspeed_flag = 0; //速度过载标志位

int moto1_speed;
int moto2_speed;
int moto1_PID_PWM, moto2_PID_PWM;
float Set_m1speed = 5.0, Set_m2speed = 5.0;
float moto1_true_speed, moto2_true_speed;
int PID_V(int encode, float ture_speed);

int moto1_PID_POSITION(int encode, float true_tag);
int moto2_PID_POSITION(int encode, float true_tag);

int m1_Incremental_PI (int Encoder,float Target);
int m2_Incremental_PI (int Encoder,float Target);

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
//10ms中断
//	moto1_PID_PWM = moto1_PID_POSITION(moto1_speed, Set_speed);   //电机控制
//	moto2_PID_PWM = moto2_PID_POSITION(moto2_speed, Set_speed);
	moto1_PID_PWM = m1_Incremental_PI(moto1_speed, Set_m1speed);
	moto2_PID_PWM = m2_Incremental_PI(moto2_speed, Set_m2speed);
	
	//方向控制
//	if(moto1_PID_PWM >= 0){ 
//		HAL_GPIO_WritePin(MOTOR1_DIR_GPIO_Port, MOTOR1_DIR_Pin, GPIO_PIN_SET);
//	}
//	if(moto1_PID_PWM <0 ) {
//		HAL_GPIO_WritePin(MOTOR1_DIR_GPIO_Port, MOTOR1_DIR_Pin, GPIO_PIN_RESET);
//		moto1_PID_PWM = 0 - moto1_PID_PWM;
//	}
//	if(moto2_PID_PWM >= 0) {
//		HAL_GPIO_WritePin(MOTOR2_DIR_GPIO_Port, MOTOR1_DIR_Pin, GPIO_PIN_SET);
//	}
//	if(moto2_PID_PWM <0 ) {
//		HAL_GPIO_WritePin(MOTOR2_DIR_GPIO_Port, MOTOR1_DIR_Pin, GPIO_PIN_RESET);
//		moto2_PID_PWM = 0- moto2_PID_PWM;
//	}
 if(moto1_PID_PWM<=0) 
	 moto1_PID_PWM = 0;
 if(moto2_PID_PWM<=0) 
	 moto2_PID_PWM = 0;
 if(overspeed_flag == 1){
		TIM3->CCR1 = 0;
		TIM3->CCR2 = 0;
 }else{
		TIM3->CCR1 = moto1_PID_PWM;
		TIM3->CCR2 = moto2_PID_PWM;
 }
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
	//TIM3->CCR1 = PID_V()
  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)          //测速中断
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
	static uint8_t cnt;
	cnt++;
	if(cnt ==3){
		cnt=0;
		moto1_speed = (short)TIM1->CNT;       //short 强制转换使得编码器数值变为有符号数
		if(moto1_speed < 0) moto1_speed = 0; 
		moto1_true_speed = (float)moto1_speed*0.157/4;
		moto2_speed = (short)TIM2->CNT;       
		if(moto2_speed < 0) moto2_speed = 0;
		moto2_true_speed = (float)moto2_speed*0.157/4;
		TIM1->CNT = 0;
		TIM2->CNT = 0;
		if(moto1_true_speed>50||moto2_true_speed>50){
			overspeed_flag = 1;
		}
		
	}

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */
	
  /* USER CODE END TIM4_IRQn 1 */
}

/* USER CODE BEGIN 1 */
float kp_v = 0, ki_v = 0 , kd_v = 0;
int tag ;
float bias_, kp_, ki_, kd_;

int PID_V(int encode, float true_tag){
	static float bias, bias_last, bias_last2, bias_d, pwm_v;
	tag = (int)(true_tag*4/0.157);
	bias = tag - encode;
	bias_ = bias - bias_last;
	kp_ = kp_v * ( bias - bias_last );
	ki_ = ki_v*bias;
	kd_ = kd_v*(bias - 2*bias_last +bias_last2);
	pwm_v += kp_+ki_+ kd_;
	bias_last2 = bias_last;
	bias_last = bias;
	if(pwm_v >10000){
		pwm_v = 9999;
	}
	if(pwm_v < -10000){
		pwm_v = -9999;
	}
	return pwm_v;
}



float m1_Velocity_KP = 20.0,  m1_Velocity_KI = 0.2 ;
int m1_Incremental_PI (int Encoder,float Target)
{ 	
	 static float Bias,Pwm,Last_bias;
	 int tag;
	 tag = (int)(Target*4/0.157);
	 Bias = tag - Encoder;                                  //
	 Pwm += m1_Velocity_KP * (Bias-Last_bias) + m1_Velocity_KI*Bias;   //
	 Last_bias = Bias;	       
	//限幅
	if(Pwm>9999) Pwm =9999;
	if(Pwm<-9999) Pwm = -9999;
	 return Pwm;                                           
}

float P_VAL, I_VAL;
float m2_Velocity_KP = 20.0,  m2_Velocity_KI = 0.2 ;
int m2_Incremental_PI (int Encoder,float Target)
{ 	
	 static float Bias,Pwm,Last_bias;
	int tag;
	tag = (int)(Target*4/0.157);
	 Bias = tag - Encoder;   
	 P_VAL = m2_Velocity_KP * (Bias-Last_bias);
	 I_VAL = m2_Velocity_KI*Bias;
	 Pwm += m2_Velocity_KP * (Bias-Last_bias) + m2_Velocity_KI*Bias;   //
	 Last_bias=Bias;	       
	//限幅
	if(Pwm>10000) Pwm =10000;
	if(Pwm<-10000) Pwm = -10000;
	 return Pwm;                                           
}


	
float moto1_bias_ki, moto1_bias_kp, moto1_bias_kd;
float moto1_pos_kp = 20.0, moto1_pos_ki = 0.1, moto1_pos_kd = 0.0;
int moto1_PID_POSITION(int encode, float true_tag){
	static float bias ,pwm, bias_i, bias_last;
	int tag;
	tag = (int)(true_tag*4/0.157);
	bias = tag - encode;
	bias_i += bias;
	moto1_bias_ki = moto1_pos_ki*bias_i;
	moto1_bias_kp = moto1_pos_kp * bias;
	moto1_bias_kd = moto1_pos_kd * (bias- bias_last);
	
	if(moto1_bias_ki<0) moto1_bias_ki = 0;   //积分分离
	
	pwm = moto1_bias_kp + moto1_bias_ki + moto1_bias_kd;
	
	bias_last = bias;
	
	if(pwm>10000){
		pwm = 9999;
	}
	if(pwm<0) {
		pwm = 0;
	}
	return pwm;
}

float moto2_bias_ki, moto2_bias_kp, moto2_bias_kd;
float moto2_pos_kp = 20.0, moto2_pos_ki = 0.1, moto2_pos_kd = 0.0;
int moto2_PID_POSITION(int encode, float true_tag){
	static float bias ,pwm, bias_i, bias_last;
	int tag;
	tag = (int)(true_tag*4/0.157);
	bias = tag - encode;
	bias_i += bias;
	
	moto2_bias_ki = moto2_pos_ki*bias_i;
	moto2_bias_kp = moto2_pos_kp * bias;
	moto2_bias_kd = moto2_pos_kd * (bias- bias_last);
	
	pwm = moto2_bias_kp + moto2_bias_ki + moto2_bias_kd;
	
	bias_last = bias;
	
	if(pwm>10000){
		pwm = 9999;
	}
	if(pwm<0) {
		pwm = 0;
	}
	return pwm;
}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
