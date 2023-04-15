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

#include "mcp47cXbXX.h"
#include "mcp47cXbXX_regs.h"

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

#ifndef MCP47CXBXX_PARAM_PDOWN
#define MCP47CXBXX_PARAM_PDOWN   (MCP47CXBXX_CONF_PD_OFF_0 \
                                  | MCP47CXBXX_CONF_PD_OFF_1)
#endif

#ifndef MCP47CXBXX_PARAM_GAIN
#define MCP47CXBXX_PARAM_GAIN   (MCP47CXBXX_CONF_GAIN_0_2X \
                                  | MCP47CXBXX_CONF_GAIN_1_2X)
#endif

#ifndef MCP47CXBXX_PARAMS
#define MCP47CXBXX_PARAMS          { .i2cHandle        = MCP47CXBXX_PARAM_I2C,\
                                  	  .addr       		= MCP47CXBXX_PARAM_ADDR,\
									  .gain       		= MCP47CXBXX_PARAM_GAIN,\
									  .vref       		= MCP47CXBXX_PARAM_VREF,\
									  .pdown       		= MCP47CXBXX_PARAM_PDOWN,\
									  .dac0       		= MCP47CXBXX_CONF_DAC_0,\
									  .dac1       		= MCP47CXBXX_CONF_DAC_1 }
#endif

/**
 * @brief   MCP47CXBXX/111x defaults if not defined for a board or application
 */
static mcp47cXbXX_params_t mcp47cXbXX_params[] =
{
    MCP47CXBXX_PARAMS
};

#ifdef __cplusplus
}
#endif

#endif /* MCP47CXBXX_PARAMS_H */
/** @} */
