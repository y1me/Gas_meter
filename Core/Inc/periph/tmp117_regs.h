/*
 * tmp117_regs.h
 *
 *  Created on: Jul 10, 2023
 *      Author: blobby
 */

#ifndef INC_PERIPH_TMP117_REGS_H_
#define INC_PERIPH_TMP117_REGS_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    TMP117 register addresses
 * @{
 */
#define TMP117_TEMP_RESULT_ADDR		(0)
#define TMP117_CONFIGURATION_ADDR	(1)
#define TMP117_THIGH_LIMIT_ADDR		(2)
#define TMP117_TLOW_LIMIT_ADDR		(3)
#define TMP117_EEPROM_UL_ADDR		(4)
#define TMP117_EEPROM1_ADDR			(5)
#define TMP117_EEPROM2_ADDR			(6)
#define TMP117_TEMP_OFFSET_ADDR		(7)
#define TMP117_EEPROM3_ADDR			(8)
#define TMP117_DEVICE_ID_ADDR		(15)
/** @} */

/**
 * @name    TMP117 Config flags
 *
 * Alert flags have no effect on TMP117
 *
 * @{
 */
#define TMP117_CONF_MOD_CC				((0 << 11) | (0 << 10))
#define TMP117_CONF_MOD_SD				((0 << 11) | (1 << 10))
#define TMP117_CONF_MOD_OS				((1 << 11) | (1 << 10))
#define TMP117_CONF_CONV_15_5_MS		((0 << 9) | (0 << 8) | (0 << 7))
#define TMP117_CONF_CONV_125_MS			((0 << 9) | (0 << 8) | (1 << 7))
#define TMP117_CONF_CONV_250_MS			((0 << 9) | (1 << 8) | (0 << 7))
#define TMP117_CONF_CONV_500_MS			((0 << 9) | (1 << 8) | (1 << 7))
#define TMP117_CONF_CONV_1S				((1 << 9) | (0 << 8) | (0 << 7))
#define TMP117_CONF_CONV_4S				((1 << 9) | (0 << 8) | (1 << 7))
#define TMP117_CONF_CONV_8S				((1 << 9) | (1 << 8) | (0 << 7))
#define TMP117_CONF_CONV_16S			((1 << 9) | (1 << 8) | (1 << 7))
#define TMP117_CONF_NO_AVG				((0 << 6) | (0 << 5))
#define TMP117_CONF_8_AVG				((0 << 6) | (1 << 5))
#define TMP117_CONF_32_AVG				((1 << 6) | (0 << 5))
#define TMP117_CONF_64_AVG				((1 << 6) | (1 << 5))
#define TMP117_CONF_THERM_MODE			(1 << 4)
#define TMP117_CONF_ALERT_MODE			(0 << 4)
#define TMP117_CONF_POL_HIGH			(1 << 3)
#define TMP117_CONF_POL_LOW				(0 << 3)
#define TMP117_CONF_PIN_DR				(1 << 2)
#define TMP117_CONF_PIN_ALERT			(0 << 2)
#define TMP117_CONF_SOFT_RESET			(1 << 1)
#define TMP117_CONF_NO_SOFT_RESET		(0 << 1)
/** @} */

/**
 * @name    TMP117 eeprom unlock register
 *
 * Supports both single mode and differential.
 * This has no effect on ADS1013-4 and ADS1113-4.
 *
 * @{
 */
#define TMP117_EEPROM_UNLOCK_EUN		(1 << 15)
#define TMP117_EEPROM_LOCK_EUN			(0 << 15)
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* INC_PERIPH_TMP117_REGS_H_ */
