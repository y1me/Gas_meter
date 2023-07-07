/*
 * Copyright (C) 2017 OTA keys S.A.
 *               2018 Acutam Automation, LLC
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup   drivers_MCP47CXBXX DAC device driver
 */

#ifndef MCP47CXBXX_H
#define MCP47CXBXX_H

#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f4xx_hal.h"
/**
 * @defgroup drivers_MCP47CXBXX_config    ADS101 driver compile configuration
 * @ingroup config_drivers_sensors
 * @{
 */
extern I2C_HandleTypeDef hi2c1;
/**
 * @brief   Set MCP47CXBXX default I2C address
 *
 * Default STM32 I2C module
 */

#ifndef MCP47CXBXX_PARAM_I2C
#define MCP47CXBXX_PARAM_I2C    (&hi2c1)
#endif

/**
 * Tx or Rx Buffer size
 */
#ifndef MCP47CXBXX_BUFFER_SIZE
#define MCP47CXBXX_BUFFER_SIZE    4
#endif
/** @} */
/**
 * Tx data Buffer size
 */
#ifndef MCP47CXBXX_TX_DATA_SIZE
#define MCP47CXBXX_TX_DATA_SIZE    1
#endif
/** @} */
/**
 * Rx data Buffer size
 */
#ifndef MCP47CXBXX_RX_DATA_SIZE
#define MCP47CXBXX_RX_DATA_SIZE    3
#endif
/** @} */
/**
 * I2C timeout
 */
#ifndef MCP47CXBXX_I2C_TIMEOUT
#define MCP47CXBXX_I2C_TIMEOUT    2
#endif
/** @} */
/**
 * Address pin A0, A1 tied to: GND (0x60)
 */
#ifndef CONFIG_MCP47CXBXX_I2C_ADDRESS
#define CONFIG_MCP47CXBXX_I2C_ADDRESS    (0x60)
#endif
/** @} */

/* Begin MCP47 state machine structures */
typedef enum {
	ST_MCP47_RESET,
	ST_MCP47_SET_VREF,
	ST_MCP47_SET_PDOWN,
	ST_MCP47_SET_GAIN,
	ST_MCP47_SET_DAC0,
	ST_MCP47_SET_DAC1,
	ST_MCP47_IDLE,
	ST_MCP47_ERROR
} state_mcp47_t;

//typedef struct {
//	state_i2c_t currState;
//} i2c_stateMachine_t;

typedef enum {
	EV_MCP47_RESET_DONE,
	EV_MCP47_VREF_DONE,
    EV_MCP47_PDOWN_DONE,
	EV_MCP47_GAIN_DONE,
	EV_MCP47_DAC0_DONE,
	EV_MCP47_DAC1_DONE,
	EV_MCP47_NONE,
	EV_MCP47_ERROR_OCCUR
} event_mcp47_t;

/* End MCP47 state machine structures */

/**
 * @brief   Named return values
 */
enum {
    MCP47CXBXX_OK          =  0,       	/**< everything was fine */
    MCP47CXBXX_NOI2C       = -1,       	/**< I2C communication failed */
	MCP47CXBXX_I2CBUSY     = -2,       	/**< I2C communication failed */
    MCP47CXBXX_NODEV       = -3,       	/**< no MCP47CXBXX device found on the bus */
    MCP47CXBXX_NODATA      = -4        	/**< no data available */
};

/**
 * @brief   MCP47CXBXX params
 */
typedef struct mcp47cXbXX_params {
	I2C_HandleTypeDef* i2cHandle;		/**< i2c device */
    uint8_t addr;						/**< i2c address */
	uint8_t pointer;					/**< pointer register */
	uint8_t config[2];
    uint16_t gain;					/**< Mux and gain boolean settings */
    uint16_t vref;					/**< Mux and gain boolean settings */
    uint16_t pdown;					/**< Mux and gain boolean settings */
    uint16_t dac0;					/**< Mux and gain boolean settings */
    uint16_t dac1;					/**< Mux and gain boolean settings */
    uint16_t loaded_gain;					/**< Mux and gain boolean settings */
    uint16_t loaded_vref;					/**< Mux and gain boolean settings */
    uint16_t loaded_pdown;					/**< Mux and gain boolean settings */
    uint16_t loaded_dac0;					/**< Mux and gain boolean settings */
    uint16_t loaded_dac1;					/**< Mux and gain boolean settings */
	state_mcp47_t currState;
	event_mcp47_t event;
} mcp47cXbXX_params_t;


/**
 * @brief   MCP47CXBXX device descriptor
 */
typedef struct mcp47cXbXX {
	mcp47cXbXX_params_t params;    		/**< device driver configuration */
} mcp47cXbXX_t;


void Running_MCP47CXBXX_StateMachine_Iteration(void);

#ifdef __cplusplus
}
#endif

#endif /* MCP47CXBXX_H */
/** @} */
