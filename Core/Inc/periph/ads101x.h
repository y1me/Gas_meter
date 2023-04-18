/*
 * Copyright (C) 2017 OTA keys S.A.
 *               2018 Acutam Automation, LLC
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup   drivers_ads101x ADS101x/111x ADC device driver
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
 * @brief      ADS101x/111x ADC device driver
 *
 * ADC and alert functionality are separated into two devices to
 * prevent wasteful representations on muxed devices.
 *
 * @author     Vincent Dupont <vincent@otakeys.com>
 * @author     Matthew Blue <matthew.blue.neuro@gmail.com>
 */

#ifndef ADS101X_H
#define ADS101X_H

#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f4xx_hal.h"
/**
 * @defgroup drivers_ads101x_config    ADS101 driver compile configuration
 * @ingroup config_drivers_sensors
 * @{
 */
extern I2C_HandleTypeDef hi2c1;
/**
 * @brief   Set ADS101x/111x default I2C address
 *
 * Default STM32 I2C module
 */

#ifndef ADS101X_PARAM_I2C
#define ADS101X_PARAM_I2C    (&hi2c1)
#endif
/**
 * alert pin status
 */
#ifndef ADS101X_PARAM_ALERT_TRUE
#define ADS101X_PARAM_ALERT_TRUE    1
#endif
#ifndef ADS101X_PARAM_ALERT_FALSE
#define ADS101X_PARAM_ALERT_FALSE    0
#endif
/** @} */
/**
 * Tx or Rx Buffer size
 */
#ifndef ADS101X_BUFFER_SIZE
#define ADS101X_BUFFER_SIZE    4
#endif
/** @} */
/**
 * Tx data Buffer size
 */
#ifndef ADS101X_TX_DATA_SIZE
#define ADS101X_TX_DATA_SIZE    1
#endif
/** @} */
/**
 * Rx data Buffer size
 */
#ifndef ADS101X_RX_DATA_SIZE
#define ADS101X_RX_DATA_SIZE    3
#endif
/** @} */
/**
 * I2C timeout
 */
#ifndef ADS101X_I2C_TIMEOUT
#define ADS101X_I2C_TIMEOUT    2
#endif
/** @} */
/**
 * Address pin tied to: GND (0x48), Vcc (0x49), SDA (0x50), SCL (0x51)
 */
#ifndef CONFIG_ADS101X_I2C_ADDRESS
#define CONFIG_ADS101X_I2C_ADDRESS    (0x48)
#endif
/** @} */

/* Begin ADS1114 state machine structures */
typedef enum {
    ST_ADS1114_INIT,
	ST_ADS1114_READ,
	ST_ADS1114_CONV,
	ST_ADS1114_ERROR
} state_ads1114_t;

//typedef struct {
//	state_i2c_t currState;
//} i2c_stateMachine_t;

typedef enum {
	EV_ADS1114_DO_INIT,
    EV_ADS1114_INIT_DONE,
	EV_ADS1114_LOW_LIMIT_DONE,
	EV_ADS1114_HIGH_LIMIT_DONE,
	EV_ADS1114_CONV_START,
	EV_ADS1114_CONV_DONE,
	EV_ADS1114_NONE,
	EV_ADS1114_ERROR_OCCUR
} event_ads1114_t;

/* End ADS1114 state machine structures */

/**
 * @brief   Named return values
 */
enum {
    ADS101X_OK          =  0,       	/**< everything was fine */
    ADS101X_NOI2C       = -1,       	/**< I2C communication failed */
	ADS101X_I2CBUSY     = -2,       	/**< I2C communication failed */
    ADS101X_NODEV       = -3,       	/**< no ADS101X device found on the bus */
    ADS101X_NODATA      = -4        	/**< no data available */
};

/**
 * @brief   ADS101x/111x input data
 */
typedef struct ads101x_data {
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
} ads101x_data_t;


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
 * @brief   ADS101x/111x params
 */
typedef struct ads101x_params {
	I2C_HandleTypeDef* i2cHandle;		/**< i2c device */
    uint8_t addr;						/**< i2c address */
    uint8_t mux_gain;					/**< Mux and gain boolean settings */
	state_ads1114_t currState;
	event_ads1114_t event;
} ads101x_params_t;

/**
 * @brief   ADS101x/111x alert params
 */
typedef struct ads101x_alert_params {
	I2C_HandleTypeDef* i2cHandle;		/**< i2c device */
    uint8_t addr;						/**< i2c address */
    uint8_t alert_pin_status;			/**< alert pin (GPIO_UNDEF if not connected) */
    int16_t low_limit;					/**< alert low value */
    int16_t high_limit;					/**< alert high value */
} ads101x_alert_params_t;

/**
 * @brief   ADS101x/111x device descriptor
 */
typedef struct ads101x {
    ads101x_params_t params;    		/**< device driver configuration */
} ads101x_t;

/**
 * @brief   ADS101x/111x alert callback
 */
typedef void (*ads101x_alert_cb_t)(void *);

/**
 * @brief   ADS101x/111x alert device descriptor
 */
typedef struct ads101x_alert {
    ads101x_alert_params_t params;    /**< device driver configuration */
    ads101x_alert_cb_t cb;            /**< alert callback */
    void *arg;                        /**< alert callback param */
} ads101x_alert_t;

void Running_ADS1114_StateMachine_Iteration(void);

#ifdef __cplusplus
}
#endif

#endif /* ADS101X_H */
/** @} */
