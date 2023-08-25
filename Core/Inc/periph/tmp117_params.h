/*
 * tmp117_params.h
 *
 *  Created on: Jul 10, 2023
 *      Author: blobby
 */

#ifndef INC_PERIPH_TMP117_PARAMS_H_
#define INC_PERIPH_TMP117_PARAMS_H_

#include "tmp117.h"
#include "tmp117_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    Set default configuration parameters for the ADS101x/111x driver
 * @{
 */

#ifndef TMP117_PARAM_I2C
#define TMP117_PARAM_I2C       (TMP117_PARAM_I2C)
#endif

#ifndef TMP117_PARAM_ADDR
#define TMP117_PARAM_ADDR       (CONFIG_TMP117_I2C_ADDRESS)
#endif
#ifndef TMP117_PARAM_MUX_GAIN
#define TMP117_PARAM_MUX_GAIN   (TMP117_AIN0_SINGM \
                                  | TMP117_PGA_FSR_4V096)
#endif

#ifndef TMP117_PARAM_ALERT_STATUS
#define TMP117_PARAM_ALERT_STATUS       (TMP117_PARAM_ALERT_FALSE)
#endif

#ifndef TMP117_PARAM_LOW_LIMIT
#define TMP117_PARAM_LOW_LIMIT  (10000U)
#endif
#ifndef TMP117_PARAM_HIGH_LIMIT
#define TMP117_PARAM_HIGH_LIMIT (20000U)
#endif

#ifndef TMP117_INIT_PARAMS
#define TMP117_INIT_PARAMS     { TMP117_CONFIGURATION_ADDR, \
									TMP117_CONF_MOD_OS \
                                  | TMP117_CONF_CONV_15_5_MS \
								  | TMP117_CONF_NO_AVG \
								  | TMP117_CONF_ALERT_MODE \
								  | TMP117_CONF_POL_LOW \
                                  | TMP117_CONF_PIN_ALERT }
#endif

#ifndef TMP117_PARAMS
#define TMP117_PARAMS          { .i2cHandle        = TMP117_PARAM_I2C,\
                                  .addr       		= TMP117_PARAM_ADDR,\
                                  .mode   		= TMP117_PARAM_MUX_GAIN,\
                                  .conv = TMP117_CONF_CONV_15_5_MS,\
                                  .average = TMP117_CONF_NO_AVG,\
                                  .T_nA = TMP117_CONF_ALERT_MODE,\
                                  .pol = TMP117_CONF_POL_LOW,\
                                  .DR_Alert = TMP117_CONF_PIN_ALERT,\
                                  .Soft_Reset = TMP117_CONF_NO_SOFT_RESET }
#endif

#ifndef TMP117_EEPROM_PARAMS
#define TMP117_EEPROM_PARAMS    { .i2cHandle        = TMP117_PARAM_I2C,\
                                  .addr       		= TMP117_PARAM_ADDR,\
								  .set_eeprom_lock	= TMP117_EEPROM_LOCK_EUN }
#endif
//#ifndef TMP117_SAUL_INFO
//#define TMP117_SAUL_INFO       { .name = "tmp117" }
//#endif
/** @} */

/**
 * @brief   TMP117 defaults if not defined for a board or application
 */
static tmp117_params_t tmp117_params[] =
{
    TMP117_PARAMS
};

/**
 * @brief   TMP117 alert defaults if not defined for a board or application
 */
static const tmp117_alert_params_t tmp117_alert_params[] =
{
    TMP117_ALERT_PARAMS
};

/**
 * @brief   Additional meta information to keep in the SAUL registry
 */
//static const saul_reg_info_t tmp117_saul_info[] =
//{
//    TMP117_SAUL_INFO
//};

#ifdef __cplusplus
}
#endif

#endif /* INC_PERIPH_TMP117_PARAMS_H_ */
