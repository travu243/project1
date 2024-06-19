/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f1xx_it.h"
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
extern uint8_t isRun1;
extern uint8_t isRun2;
extern uint8_t isRun_xoaybanh;
extern uint8_t isRun_xoay_stepXL;
uint8_t dem1=0;
uint8_t dem2=0;
uint8_t dem3=0;
uint8_t dem4=0;
extern uint16_t tocdo_dco;
extern uint16_t tocdo_xoaybanh;
extern uint16_t tocdo_xoaystep;
const uint16_t xung_xoaybanh90=29866;
const uint16_t xung_xoaybanh0=1;
uint16_t xung_xoaybanhht=0;
const uint16_t xung_xoay_stepXL=20923;
uint16_t dem_xung_xoay_stepXL;
extern uint8_t chieu_xoaybanh;
//extern uint8_t chieu_xoay_stepXL;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
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
  * @brief This function handles Prefetch fault, memory access fault.
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
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
	//1mhz/10=100000/s
	//150microseconds=1000/s
	if(isRun1==1){
		if(dem1<tocdo_dco) dem1++;
		else{
			dem1=0;
			HAL_GPIO_TogglePin(pulse_dc1_GPIO_Port,pulse_dc1_Pin);
		}
	}
	if(isRun2==1){
		if(dem2<tocdo_dco) dem2++;
		else{
			dem2=0;
			HAL_GPIO_TogglePin(pulse_dc2_GPIO_Port,pulse_dc2_Pin);
		}
	}
	if(isRun_xoaybanh==1){
		if(dem3<tocdo_xoaybanh) dem3++;
		else{
			dem3=0;
			if(chieu_xoaybanh==0) --xung_xoaybanhht;
			if(chieu_xoaybanh==1) ++xung_xoaybanhht;
			if(dem_xung_xoay_stepXL<5000 || dem_xung_xoay_stepXL>24000){tocdo_xoaybanh=8;}
			else {tocdo_xoaybanh=0;}
			HAL_GPIO_TogglePin(pulse_step1_GPIO_Port,pulse_step1_Pin);
			HAL_GPIO_TogglePin(pulse_step2_GPIO_Port,pulse_step2_Pin);
			if(xung_xoaybanhht<xung_xoaybanh0 || xung_xoaybanhht>xung_xoaybanh90){
				isRun_xoaybanh=0;
			}
		}
	}
	if(isRun_xoay_stepXL==1){
//		if(dem_xung_xoay_stepXL<4000 || dem_xung_xoay_stepXL>16000) tocdo_xoaystep=30;
//		else tocdo_xoaystep=10;
		
		if(dem4<tocdo_xoaystep) dem4++;
		else{
			dem4=0;
			++dem_xung_xoay_stepXL;
			HAL_GPIO_TogglePin(pulse_step3_GPIO_Port,pulse_step3_Pin);
			
			
			
//			if(dem_xung_xoay_stepXL>=xung_xoay_stepXL){
//				dem_xung_xoay_stepXL=0;
//				isRun_xoay_stepXL=0;
//			}
		}
	}  
  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
