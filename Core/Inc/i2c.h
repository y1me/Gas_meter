/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.h
  * @brief   This file contains all the function prototypes for
  *          the i2c.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN Private defines */
/* Begin I2C state machine structures */
typedef enum {
    ST_I2C_IDLE,
    ST_I2C_INIT,
	ST_I2C_DMA_RX,
	ST_I2C_DMA_TX,
	ST_I2C_RX,
	ST_I2C_TX,
	ST_I2C_ERROR
} state_i2c_t;

//typedef struct {
//	state_i2c_t currState;
//} i2c_stateMachine_t;

typedef enum {
    EV_I2C_INIT_DONE,
	EV_I2C_DMA_RX_DONE,
	EV_I2C_DMA_TX_DONE,
	EV_I2C_DMA_TX_RX_TX_DONE,
	EV_I2C_RX_DONE,
	EV_I2C_TX_DONE,
	EV_I2C_DMA_RX,
	EV_I2C_DMA_TX,
	EV_I2C_DMA_TX_RX,
	EV_I2C_RX,
	EV_I2C_TX,
	EV_I2C_ERROR,
	EV_I2C_NONE
} event_i2c_t;

/* End I2C state machine structures */

typedef struct {
	I2C_HandleTypeDef* i2cHandle;
	uint16_t address;
	uint8_t* bufferTx;
	uint8_t* bufferRx;
	uint16_t sizeTx;
	uint16_t sizeRx;
	uint32_t timeout;
	state_i2c_t currState;
	event_i2c_t event;
} i2cFunctionParam_t;

enum {
    I2C_FREE        =  1,       	/**< I2C free */
    I2C_OK          =  0,       	/**< I2C no error */
    I2C_FAIL       	= -1,       	/**< I2C command failed */
	I2C_BUSY       	= -2,			/**< I2C module in use */
	I2C_ERROR       = -3,			/**< I2C error */
	I2C_ERR_OCCUR	= -4      		/**< I2C error on last use*/
};
/* USER CODE END Private defines */

//void MX_I2C1_Init(void);

/* USER CODE BEGIN Prototypes */
//void MX_I2C1_Init(i2cFunctionParam_t *);
void Running_I2C_StateMachine_Iteration(void);
void I2C_RX_TX_DMA_ACK(void);
int16_t I2C_status(void);
int16_t I2C_clear_last_event(void);
int16_t read_I2C_device_DMA(I2C_HandleTypeDef *, uint16_t , uint8_t * , uint16_t );
int16_t write_I2C_device_DMA(I2C_HandleTypeDef *, uint16_t , uint8_t *, uint16_t );
int16_t write_read_I2C_device_DMA(I2C_HandleTypeDef *, uint16_t , uint8_t *, uint8_t *, uint16_t , uint16_t );

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */

