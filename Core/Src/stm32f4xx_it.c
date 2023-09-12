/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "gpio.h"
#include "i2c.h"
#include "stddef.h"
#include "periph/ads101x.h"
#include "periph/mcp47cXbXX.h"
#include "periph/tmp117.h"
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
volatile uint32_t tick_10us = 0, tick_ms = 0, tick_second = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static PFV_EXTI pf_ext_int[]=
		{
				NULL,// 0
				NULL,
				NULL,
				NULL,
				NULL,//4
				NULL,
				NULL,
				NULL,
				NULL,
				NULL,//9
				NULL,
				NULL,
				NULL,
				NULL,
				NULL,
				NULL//15
		};

static const TIMED_PERIOD timed_task_second[] =
{
    { 3, 0,  Running_TMP117_StateMachine_Iteration },
    { 0, 0, NULL }
};

static const TIMED_PERIOD timed_task_ms[] =
{
    //{ 200, 0, Running_ADS1114_StateMachine_Iteration },
    {500, 0, Running_MCP47CXBXX_StateMachine_Iteration }
};

static const TIMED_PERIOD timed_task_10us[] =
{
    { 10, 0, Running_I2C_StateMachine_Iteration },
    { 0, 0, NULL }
};
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_i2c1_rx;
extern DMA_HandleTypeDef hdma_i2c1_tx;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim2;
/* USER CODE BEGIN EV */

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
	tick_ms++;

	if (tick_ms == 1000)
	{
		tick_ms = 0;
		tick_second++;

		for (const TIMED_PERIOD *ptr = timed_task_second; ptr->interval != 0; ptr++)
		{
			if (!((tick_second - ptr->offset) % ptr->interval) && (ptr->proc != NULL))
			{
				/* Time to call the function */
				(ptr->proc)();
			}
		}
	}

	if (tick_second == 60)
	{
		tick_second = 0;
	}

	for (const TIMED_PERIOD *ptr = timed_task_ms; ptr->interval != 0; ptr++)
	{
		if (!((tick_ms - ptr->offset) % ptr->interval) && (ptr->proc != NULL))
		{
			/* Time to call the function */
			(ptr->proc)();
		}
	}
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line2 interrupt.
  */
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */

  /* USER CODE END EXTI2_IRQn 0 */
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_2) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_2);
    /* USER CODE BEGIN LL_EXTI_LINE_2 */

    /* USER CODE END LL_EXTI_LINE_2 */
  }
  /* USER CODE BEGIN EXTI2_IRQn 1 */

  /* USER CODE END EXTI2_IRQn 1 */
}

/**
  * @brief This function handles EXTI line3 interrupt.
  */
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */

  /* USER CODE END EXTI3_IRQn 0 */
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_3) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_3);
    /* USER CODE BEGIN LL_EXTI_LINE_3 */

    /* USER CODE END LL_EXTI_LINE_3 */
  }
  /* USER CODE BEGIN EXTI3_IRQn 1 */

  /* USER CODE END EXTI3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream0 global interrupt.
  */
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c1_rx);
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */
  I2C_RX_TX_DMA_ACK();
  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream6 global interrupt.
  */
void DMA1_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream6_IRQn 0 */

  /* USER CODE END DMA1_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c1_tx);
  /* USER CODE BEGIN DMA1_Stream6_IRQn 1 */
  I2C_RX_TX_DMA_ACK();
  /* USER CODE END DMA1_Stream6_IRQn 1 */
}

/**
  * @brief This function handles ADC1 global interrupt.
  */
void ADC_IRQHandler(void)
{
  /* USER CODE BEGIN ADC_IRQn 0 */

  /* USER CODE END ADC_IRQn 0 */

  /* USER CODE BEGIN ADC_IRQn 1 */

  /* USER CODE END ADC_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
  tick_10us++;
  TOGGLE_GPIO_TEST_PIN();

    if (tick_10us == 100)
    {
  	  tick_10us = 0;
    }

    for (const TIMED_PERIOD *ptr = timed_task_10us; ptr->interval != 0; ptr++)
    {
  	  if (!((tick_10us - ptr->offset)  % ptr->interval) && (ptr->proc != NULL))
  	  {
  		  /* Time to call the function */
  		  (ptr->proc)();
  	  }
    }
  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles I2C1 event interrupt.
  */
void I2C1_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_EV_IRQn 0 */

  /* USER CODE END I2C1_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&hi2c1);
  /* USER CODE BEGIN I2C1_EV_IRQn 1 */

  /* USER CODE END I2C1_EV_IRQn 1 */
}

/**
  * @brief This function handles I2C1 error interrupt.
  */
void I2C1_ER_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_ER_IRQn 0 */

  /* USER CODE END I2C1_ER_IRQn 0 */
  HAL_I2C_ER_IRQHandler(&hi2c1);
  /* USER CODE BEGIN I2C1_ER_IRQn 1 */

  /* USER CODE END I2C1_ER_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_10) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_10);
    /* USER CODE BEGIN LL_EXTI_LINE_10 */

    /* USER CODE END LL_EXTI_LINE_10 */
  }
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_12) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_12);
    /* USER CODE BEGIN LL_EXTI_LINE_12 */

    /* USER CODE END LL_EXTI_LINE_12 */
  }
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_13) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_13);
    /* USER CODE BEGIN LL_EXTI_LINE_13 */

    /* USER CODE END LL_EXTI_LINE_13 */
  }
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_14) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_14);
    /* USER CODE BEGIN LL_EXTI_LINE_14 */

    /* USER CODE END LL_EXTI_LINE_14 */
  }
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_15) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_15);
    /* USER CODE BEGIN LL_EXTI_LINE_15 */

    /* USER CODE END LL_EXTI_LINE_15 */
  }
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
