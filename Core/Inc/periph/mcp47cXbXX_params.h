/*
 * Copyright (C) 2017 OTA keys S.A.
 *               2018 Acutam Automation, LLC
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_MCP47CXBXX
 * @{
 *
 * @file
 * @brief       Default configuration for MCP47CXBXX/111x devices
 *
 * @author      Vincent Dupont <vincent@otakeys.com>
 * @author      Matthew Blue <matthew.blue.neuro@gmail.com>
 */

#ifndef MCP47CXBXX_PARAMS_H
#define MCP47CXBXX_PARAMS_H

#include "MCP47CXBXX.h"
#include "MCP47CXBXX_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    Set default configuration parameters for the MCP47CXBXX/111x driver
 * @{
 */

#ifndef MCP47CXBXX_PARAM_I2C
#define MCP47CXBXX_PARAM_I2C       (MCP47CXBXX_PARAM_I2C)
#endif

#ifndef MCP47CXBXX_PARAM_ADDR
#define MCP47CXBXX_PARAM_ADDR       (CONFIG_MCP47CXBXX_I2C_ADDRESS)
#endif

#ifndef MCP47CXBXX_PARAM_VREF
#define MCP47CXBXX_PARAM_VREF   (MCP47CXBXX_CONF_INT_BGAP_0 \
                                  | MCP47CXBXX_CONF_INT_BGAP_1)
#endif



#ifndef MCP47CXBXX_PARAMS
#define MCP47CXBXX_PARAMS          { .i2cHandle        = MCP47CXBXX_PARAM_I2C,\
                                  .addr       		= MCP47CXBXX_PARAM_ADDR,\
                                  .mux_gain   		= MCP47CXBXX_PARAM_MUX_GAIN }
#endif

#ifndef MCP47CXBXX_ALERT_PARAMS
#define MCP47CXBXX_ALERT_PARAMS    { .i2cHandle        = MCP47CXBXX_PARAM_I2C,\
                                  .addr       		= MCP47CXBXX_PARAM_ADDR,\
								  .alert_pin_status	= MCP47CXBXX_PARAM_ALERT_STATUS,\
                                  .low_limit  		= MCP47CXBXX_PARAM_LOW_LIMIT,\
                                  .high_limit 		= MCP47CXBXX_PARAM_HIGH_LIMIT }
#endif
//#ifndef MCP47CXBXX_SAUL_INFO
//#define MCP47CXBXX_SAUL_INFO       { .name = "MCP47CXBXX" }
//#endif
/** @} */

/**
 * @brief   MCP47CXBXX/111x defaults if not defined for a board or application
 */
static MCP47CXBXX_params_t MCP47CXBXX_params[] =
{
    MCP47CXBXX_PARAMS
};

/**
 * @brief   MCP47CXBXX/111x alert defaults if not defined for a board or application
 */
static const MCP47CXBXX_alert_params_t MCP47CXBXX_alert_params[] =
{
    MCP47CXBXX_ALERT_PARAMS
};

/**
 * @brief   Additional meta information to keep in the SAUL registry
 */
//static const saul_reg_info_t MCP47CXBXX_saul_info[] =
//{
//    MCP47CXBXX_SAUL_INFO
//};

#ifdef __cplusplus
}
#endif

#endif /* MCP47CXBXX_PARAMS_H */
/** @} */
