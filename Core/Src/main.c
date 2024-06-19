/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "PS2.h"
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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

PS2Buttons ps2;


uint8_t isRun1=0;
uint8_t isRun2=0;
uint8_t isRun_xoaybanh=0;
uint8_t isRun_xoay_stepXL=0;
uint16_t tocdo_dco=100;
uint16_t tocdo_xoaybanh=10;
uint16_t tocdo_xoaystep=20;
uint8_t chieu_xoaybanh=0;
//uint8_t chieu_xoay_stepXL=0;
extern uint16_t xung_xoaybanhht;
extern uint16_t xung_xoaybanh90;
extern uint16_t xung_xoaybanh0;
extern uint16_t dem_xung_xoay_stepXL;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  if(htim->Instance == TIM2)
	{
		PS2_Update();
	}
}

void xoay_stepXLL(){
		HAL_GPIO_WritePin(dir_step3_GPIO_Port,dir_step3_Pin,GPIO_PIN_RESET);
		isRun_xoay_stepXL=1;
}

void xoay_stepXLR(){
		HAL_GPIO_WritePin(dir_step3_GPIO_Port,dir_step3_Pin,GPIO_PIN_SET);
		isRun_xoay_stepXL=1;
}

//cac' ham` chi? xoay banh' va` chinh? huong' cho banh'
void forward(){
	if(chieu_xoaybanh==0){}
	else{
		chieu_xoaybanh=0;
		HAL_GPIO_WritePin(dir_step1_GPIO_Port,dir_step1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(dir_step2_GPIO_Port,dir_step2_Pin,GPIO_PIN_SET);
		isRun_xoaybanh=1;
	}
}

void backward(){
	if(chieu_xoaybanh==0){}
	else{
		chieu_xoaybanh=0;
		HAL_GPIO_WritePin(dir_step1_GPIO_Port,dir_step1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(dir_step2_GPIO_Port,dir_step2_Pin,GPIO_PIN_SET);
		isRun_xoaybanh=1;
	}
}

void left(){
	if(chieu_xoaybanh==1){}
	else{
		chieu_xoaybanh=1;
		HAL_GPIO_WritePin(dir_step1_GPIO_Port,dir_step1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(dir_step2_GPIO_Port,dir_step2_Pin,GPIO_PIN_RESET);
		isRun_xoaybanh=1;
	}
}

void right(){
	if(chieu_xoaybanh==1){}
	else{
		chieu_xoaybanh=1;
		HAL_GPIO_WritePin(dir_step1_GPIO_Port,dir_step1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(dir_step2_GPIO_Port,dir_step2_Pin,GPIO_PIN_RESET);
		isRun_xoaybanh=1;
	}
}

void stop(){
	isRun1=0;
	isRun2=0;
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	
	PS2_Init(&htim1,&ps2);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim4);
	
	
	HAL_GPIO_WritePin(XL1_GPIO_Port,XL1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(XL2_GPIO_Port,XL2_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(XL3_GPIO_Port,XL3_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(XL4_GPIO_Port,XL4_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(XL5_GPIO_Port,XL5_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(XL6_GPIO_Port,XL6_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(XL7_GPIO_Port,XL7_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(XL8_GPIO_Port,XL8_Pin,GPIO_PIN_SET);
	
	
//	int x=20923;
//	int count=0;
//	while(1){
//		if(count<x){
//			count++;
//			HAL_GPIO_WritePin(dir_step3_GPIO_Port,dir_step3_Pin,GPIO_PIN_RESET);
//			HAL_GPIO_TogglePin(pulse_step3_GPIO_Port,pulse_step3_Pin);
//			HAL_Delay(1);
//		}
//		else{
//			
//			HAL_GPIO_WritePin(dir_step3_GPIO_Port,dir_step3_Pin,GPIO_PIN_SET);
//			HAL_GPIO_TogglePin(pulse_step3_GPIO_Port,pulse_step3_Pin);
//			HAL_Delay(1);
//		}
//	}
		
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//di chuyen
		if(ps2.L1){				//tien trai
//			do{forward();}while(0);
			HAL_GPIO_WritePin(dir_dc1_GPIO_Port,dir_dc1_Pin,GPIO_PIN_RESET);
			isRun1=1;
		}	
		else if(ps2.L2){	//lui trai
//			do{backward();}while(0);
			HAL_GPIO_WritePin(dir_dc1_GPIO_Port,dir_dc1_Pin,GPIO_PIN_SET);
			isRun1=1;
		}
		else{isRun1=0;}
		
		if(ps2.R1){	//tien phai
//			do{forward();}while(0);
			HAL_GPIO_WritePin(dir_dc2_GPIO_Port,dir_dc2_Pin,GPIO_PIN_SET);
			isRun2=1;
		}
		else if(ps2.R2){	//lui phai
//			do{backward();}while(0);
			HAL_GPIO_WritePin(dir_dc2_GPIO_Port,dir_dc2_Pin,GPIO_PIN_RESET);
			isRun2=1;
		}
		else{isRun2=0;}
		
		
		if(ps2.LY==0){
			forward();
		}
		else if(ps2.LY==255){
			backward();
		}
		else if(ps2.LX==0){
			left();
		}
		else if(ps2.LX==255){
			right();
		}
		
		
		if(ps2.SELECT){    //giam toc
			tocdo_dco+=10;
			HAL_Delay(200);
			if(tocdo_dco>180) tocdo_dco = 180;
		}
		if(ps2.START){     //tang toc
			tocdo_dco-=10;
			HAL_Delay(200);
			if(tocdo_dco<20) tocdo_dco = 20;
		}
		
		
		if(ps2.SQUARE){
//			xoay_stepXLR();
			HAL_GPIO_WritePin(dir_step3_GPIO_Port,dir_step3_Pin,GPIO_PIN_SET);
			isRun_xoay_stepXL=1;
		}
		else if(ps2.LEFT){
//			xoay_stepXLL();
			HAL_GPIO_WritePin(dir_step3_GPIO_Port,dir_step3_Pin,GPIO_PIN_RESET);
			isRun_xoay_stepXL=1;
		}
		else {
			if(tocdo_xoaystep==10){
				tocdo_xoaystep=40;
				dem_xung_xoay_stepXL=0;
			}
			if(dem_xung_xoay_stepXL==232){
				isRun_xoay_stepXL=0;
			}
			
			isRun_xoay_stepXL=0;
		}
		
		
		//dk xylanh
		if(ps2.TRIANGLE){ HAL_GPIO_TogglePin(XL2_GPIO_Port,XL2_Pin); HAL_Delay(200);}
		if(ps2.CIRCLE){ HAL_GPIO_TogglePin(XL1_GPIO_Port,XL1_Pin); HAL_Delay(200);}
		if(ps2.CROSS){ HAL_GPIO_TogglePin(XL3_GPIO_Port,XL3_Pin); HAL_Delay(200);}
		if(ps2.R3){ HAL_GPIO_TogglePin(XL6_GPIO_Port,XL6_Pin); HAL_Delay(200);}
		if(ps2.UP){ HAL_GPIO_TogglePin(XL4_GPIO_Port,XL4_Pin); HAL_Delay(200);}
		if(ps2.RIGHT){ HAL_GPIO_TogglePin(XL8_GPIO_Port,XL8_Pin); HAL_Delay(200);}
		if(ps2.DOWN){ HAL_GPIO_TogglePin(XL7_GPIO_Port,XL7_Pin); HAL_Delay(200);}
		if(ps2.L3){ HAL_GPIO_TogglePin(XL5_GPIO_Port,XL5_Pin); HAL_Delay(200);}
		
		
		
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 63;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 6399;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 69;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 63;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 9;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, dir_step3_Pin|pulse_step3_Pin|SS_Pin|SCK_Pin
                          |MOSI_Pin|XL1_Pin|XL2_Pin|XL3_Pin
                          |XL4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, XL5_Pin|XL6_Pin|XL7_Pin|XL8_Pin
                          |pulse_dc1_Pin|dir_step1_Pin|dir_step2_Pin|pulse_dc2_Pin
                          |dir_dc1_Pin|pulse_step1_Pin|pulse_step2_Pin|dir_dc2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : dir_step3_Pin pulse_step3_Pin SS_Pin SCK_Pin
                           MOSI_Pin XL1_Pin XL2_Pin XL3_Pin
                           XL4_Pin */
  GPIO_InitStruct.Pin = dir_step3_Pin|pulse_step3_Pin|SS_Pin|SCK_Pin
                          |MOSI_Pin|XL1_Pin|XL2_Pin|XL3_Pin
                          |XL4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MISO_Pin */
  GPIO_InitStruct.Pin = MISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MISO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : XL5_Pin XL6_Pin XL7_Pin XL8_Pin
                           pulse_dc1_Pin dir_step1_Pin dir_step2_Pin pulse_dc2_Pin
                           dir_dc1_Pin pulse_step1_Pin pulse_step2_Pin dir_dc2_Pin */
  GPIO_InitStruct.Pin = XL5_Pin|XL6_Pin|XL7_Pin|XL8_Pin
                          |pulse_dc1_Pin|dir_step1_Pin|dir_step2_Pin|pulse_dc2_Pin
                          |dir_dc1_Pin|pulse_step1_Pin|pulse_step2_Pin|dir_dc2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
