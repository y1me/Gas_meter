/*
 * Copyright (C) 2017 OTA keys S.A.
 *               2018 Acutam Automation, LLC
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup   drivers_MCP47CXBXX MCP47CXBXX/111x ADC device driver
 * @ingroup    drivers_sensors
 * @ingroup    drivers_saul
 * @brief      I2C Analog-to-Digital Converter device driver
 *
 * This driver works with ADS1013-5 and ADS1113-5.
 *
 * This driver provides @ref drivers_saul capabilities.
 * @{
 *
 * @file
 * @brief      MCP47CXBXX/111x ADC device driver
 *
 * ADC and alert functionality are separated into two devices to
 * prevent wasteful representations on muxed devices.
 *
 * @author     Vincent Dupont <vincent@otakeys.com>
 * @author     Matthew Blue <matthew.blue.neuro@gmail.com>
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
 * @brief   Set MCP47CXBXX/111x default I2C address
 *
 * Default STM32 I2C module
 */

#ifndef MCP47CXBXX_PARAM_I2C
#define MCP47CXBXX_PARAM_I2C    (&hi2c1)
#endif
/**
 * alert pin status
 */
#ifndef MCP47CXBXX_PARAM_ALERT_TRUE
#define MCP47CXBXX_PARAM_ALERT_TRUE     1
#endif
#ifndef MCP47CXBXX_PARAM_ALERT_FALSE
#define MCP47CXBXX_PARAM_ALERT_FALSE    0
#endif
/** @} */
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
 * Address pin tied to: GND (0x48), Vcc (0x49), SDA (0x50), SCL (0x51)
 */
#ifndef CONFIG_MCP47CXBXX_I2C_ADDRESS
#define CONFIG_MCP47CXBXX_I2C_ADDRESS    (0x49)
#endif
/** @} */

/* Begin MCP47 state machine structures */
typedef enum {
    ST_MCP47_INIT,
	ST_MCP47_SET_VREF,
	ST_MCP47_SET_PDOWN,
	ST_MCP47_SET_GAIN,
	ST_MCP47_SET_DAC0,
	ST_MCP47_SET_DAC1,
	ST_MCP47_ERROR
} state_ads1114_t;

//typedef struct {
//	state_i2c_t currState;
//} i2c_stateMachine_t;

typedef enum {
	EV_MCP47_DO_INIT,
    EV_MCP47_INIT_DONE,
	EV_MCP47_LOW_LIMIT_DONE,
	EV_MCP47_HIGH_LIMIT_DONE,
	EV_MCP47_MUX_DONE,
	EV_MCP47_CONV_RDY,
	EV_MCP47_CONV_DONE,
	EV_MCP47_NONE,
	EV_MCP47_ERROR_OCCUR
} event_ads1114_t;

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
 * @brief   MCP47CXBXX/111x input data
 */
typedef struct MCP47CXBXX_data {
	uint8_t pointer;					/**< data pointer register */
	uint8_t config[2];					/**< data from/to config register */
	uint8_t ain0[2];					/**< data from single-ended input AIN0 */
	//uint8_t ain0_prev[2];				/**< previous data from single-ended input AIN0 */
	uint8_t ain1[2];					/**< data from single-ended input AIN1 */
	//uint8_t ain1_prev[2];				/**< previous data from single-ended input AIN1 */
	uint8_t ain2[2];					/**< data from single-ended input AIN2 */
	//uint8_t ain2_prev[2];				/**< previous data from single-ended input AIN2 */
	uint8_t ain3[2];					/**< data from single-ended input AIN3 */
	//uint8_t ain3_prev[2];				/**< previous data from single-ended input AIN3 */
} MCP47CXBXX_data_t;


/*typedef union DATA_DISPLAY
{
    unsigned char _byte[4];  //For byte access

    struct
    {
        unsigned char DataToWrite;
        unsigned char DataRead;
        unsigned char DataWrite;
        unsigned char CSTiming;
    };

} DATA_DISPLAY;*/
/**
 * @brief   MCP47CXBXX/111x params
 */
typedef struct MCP47CXBXX_params {
	I2C_HandleTypeDef* i2cHandle;		/**< i2c device */
    uint8_t addr;						/**< i2c address */
    uint8_t mux_gain;					/**< Mux and gain boolean settings */
	state_ads1114_t currState;
	event_ads1114_t event;
} MCP47CXBXX_params_t;

/**
 * @brief   MCP47CXBXX/111x alert params
 */
typedef struct MCP47CXBXX_alert_params {
	I2C_HandleTypeDef* i2cHandle;		/**< i2c device */
    uint8_t addr;						/**< i2c address */
    uint8_t alert_pin_status;			/**< alert pin (GPIO_UNDEF if not connected) */
    int16_t low_limit;					/**< alert low value */
    int16_t high_limit;					/**< alert high value */
} MCP47CXBXX_alert_params_t;

/**
 * @brief   MCP47CXBXX/111x device descriptor
 */
typedef struct MCP47CXBXX {
    MCP47CXBXX_params_t params;    		/**< device driver configuration */
} MCP47CXBXX_t;

/**
 * @brief   MCP47CXBXX/111x alert callback
 */
typedef void (*MCP47CXBXX_alert_cb_t)(void *);

/**
 * @brief   MCP47CXBXX/111x alert device descriptor
 */
typedef struct MCP47CXBXX_alert {
    MCP47CXBXX_alert_params_t params;    /**< device driver configuration */
    MCP47CXBXX_alert_cb_t cb;            /**< alert callback */
    void *arg;                        /**< alert callback param */
} MCP47CXBXX_alert_t;

void conv_ready(void);
void Running_ADS115_StateMachine_Iteration(void);

/**
 * @brief   Enable alert interrupt
 *
 * Alert settings have no effect on ADS1013 and ADS1113.
 *
 * @param[in] dev   device descriptor
 * @param[in] cb    callback called when the alert fires
 * @param[in] arg   callback argument
 *
 * @return zero on success, non zero on error
 */
int16_t MCP47CXBXX_enable_alert(MCP47CXBXX_alert_t *dev,
                         MCP47CXBXX_alert_cb_t cb, void *arg);

/**
 * @brief   Set the alert parameters
 *
 * Alert settings have no effect on ADS1013 and ADS1113.
 *
 * @param[in,out] dev      device descriptor
 * @param[in] low_limit    alert low limit
 * @param[in] high_limit   alert high limit
 *
 * @return zero on success, non zero on error
 */
int16_t MCP47CXBXX_set_alert_parameters(const MCP47CXBXX_alert_t *dev,
                                 int16_t low_limit, int16_t high_limit);

#ifdef __cplusplus
}
#endif

#endif /* MCP47CXBXX_H */
/** @} */
