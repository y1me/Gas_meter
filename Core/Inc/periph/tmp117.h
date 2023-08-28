/*
 * tmp117.h
 *
 *  Created on: Jul 10, 2023
 *      Author: blobby
 */

#ifndef INC_PERIPH_TMP117_H_
#define INC_PERIPH_TMP117_H_

#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f4xx_hal.h"
/**
 * @defgroup drivers_tmp117_config    ADS101 driver compile configuration
 * @ingroup config_drivers_sensors
 * @{
 */
extern I2C_HandleTypeDef hi2c1;
/**
 * @brief   Set ADS101x/111x default I2C address
 *
 * Default STM32 I2C module
 */

#ifndef TMP117_PARAM_I2C
#define TMP117_PARAM_I2C    (&hi2c1)
#endif
/**
 * alert pin status
 */
#ifndef TMP117_PARAM_ALERT_TRUE
#define TMP117_PARAM_ALERT_TRUE    1
#endif
#ifndef TMP117_PARAM_ALERT_FALSE
#define TMP117_PARAM_ALERT_FALSE    0
#endif
/** @} */
/**
 * Tx or Rx Buffer size
 */
#ifndef TMP117_BUFFER_SIZE
#define TMP117_BUFFER_SIZE    4
#endif
/** @} */
/**
 * Tx data Buffer size
 */
#ifndef TMP117_TX_DATA_SIZE
#define TMP117_TX_DATA_SIZE    1
#endif
/** @} */
/**
 * Rx data Buffer size
 */
#ifndef TMP117_RX_DATA_SIZE
#define TMP117_RX_DATA_SIZE    3
#endif
/** @} */
/**
 * I2C timeout
 */
#ifndef TMP117_I2C_TIMEOUT
#define TMP117_I2C_TIMEOUT    2
#endif
/** @} */
/**
 * Address pin tied to: GND (0x48), Vcc (0x49), SDA (0x50), SCL (0x51)
 */
#ifndef CONFIG_TMP117_I2C_ADDRESS
#define CONFIG_TMP117_I2C_ADDRESS    (0x49)
#endif
/** @} */

/* Begin TMP117 state machine structures */
typedef enum {
    ST_TMP117_INIT,
	ST_TMP117_CONV,
	ST_TMP117_READ,
	ST_TMP117_ERROR
} state_tmp117_t;

//typedef struct {
//	state_i2c_t currState;
//} i2c_stateMachine_t;

typedef enum {
	EV_TMP117_DO_INIT,
    EV_TMP117_INIT_DONE,
	EV_TMP117_CONV_DONE,
	EV_TMP117_TEMP_READED,
	EV_TMP117_NONE,
	EV_TMP117_ERROR_OCCUR
} event_tmp117_t;

/* End TMP117 state machine structures */

/**
 * @brief   Named return values
 */
enum {
    TMP117_OK          =  0,       	/**< everything was fine */
    TMP117_NOI2C       = -1,       	/**< I2C communication failed */
	TMP117_I2CBUSY     = -2,       	/**< I2C communication failed */
    TMP117_NODEV       = -3,       	/**< no TMP117 device found on the bus */
    TMP117_NODATA      = -4        	/**< no data available */
};

/**
 * @brief   ADS101x/111x input data
 */
typedef struct tmp117_data {
	uint8_t pointer;					/**< data pointer register */
	uint8_t config[2];					/**< data from/to config register */
	uint8_t temperature[2];				/**< data from temperature register */
} tmp117_data_t;

/**
 * @brief   ADS101x/111x params
 */
typedef struct tmp117_params {
	I2C_HandleTypeDef* i2cHandle;		/**< i2c device */
    uint8_t addr;						/**< i2c address */
	uint8_t pointer;					/**< pointer register */
	uint8_t config[2];
	uint8_t temperature[2];
    uint16_t mode;					/**< Set conversion mode */
    uint16_t conv;					/**< Set Conversion cycle bit */
    uint16_t average;				/**< Set Conversion averaging modes */
    uint16_t T_nA;					/**< Set Therm/alert mode select */
    uint16_t pol;					/**< Set ALERT pin polarity bit */
    uint16_t DR_Alert;				/**< Set ALERT pin select bit */
    uint16_t Soft_Reset;			/**< Set Software reset bit */
	state_tmp117_t currState;
	event_tmp117_t event;
} tmp117_params_t;

/**
 * @brief   ADS101x/111x alert params
 */
typedef struct tmp117_alert_params {
	I2C_HandleTypeDef* i2cHandle;		/**< i2c device */
    uint8_t addr;						/**< i2c address */
    uint8_t set_eeprom_lock;			/**< alert pin (GPIO_UNDEF if not connected) */
} tmp117_alert_params_t;

/**
 * @brief   ADS101x/111x device descriptor
 */
typedef struct tmp117 {
    tmp117_params_t params;    		/**< device driver configuration */
} tmp117_t;

/**
 * @brief   ADS101x/111x alert callback
 */
typedef void (*tmp117_alert_cb_t)(void *);

/**
 * @brief   ADS101x/111x alert device descriptor
 */
typedef struct tmp117_alert {
    tmp117_alert_params_t params;    /**< device driver configuration */
    tmp117_alert_cb_t cb;            /**< alert callback */
    void *arg;                        /**< alert callback param */
} tmp117_alert_t;

void Running_TMP117_StateMachine_Iteration(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_PERIPH_TMP117_H_ */
