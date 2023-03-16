/*
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_MCP47CXBXX
 * @{
 *
 * @file
 * @brief       Register definition for MCP47CXBXX devices
 *
 */

#ifndef MCP47CXBXX_REGS_H
#define MCP47CXBXX_REGS_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    MCP47CXBXX register addresses
 * @{
 */
#define MCP47CXBXX_VOL_DAC0_ADDR      (0)
#define MCP47CXBXX_VOL_DAC1_ADDR      (1)
#define MCP47CXBXX_VOL_VREF_ADDR      (8)
#define MCP47CXBXX_VOL_PD_ADDR        (9)
#define MCP47CXBXX_VOL_G_S_ADDR       (10)
#define MCP47CXBXX_MTP_0C_ADDR        (12)
#define MCP47CXBXX_MTP_0D_ADDR        (13)
#define MCP47CXBXX_MTP_0E_ADDR        (14)
#define MCP47CXBXX_MTP_0F_ADDR        (15)

#define MCP47CXBXX_NVOL_DAC0_ADDR      (16)
#define MCP47CXBXX_NVOL_DAC1_ADDR      (17)
#define MCP47CXBXX_NVOL_VREF_ADDR      (24)
#define MCP47CXBXX_NVOL_PD_ADDR        (25)
#define MCP47CXBXX_NVOL_G_A_ADDR       (26)
#define MCP47CXBXX_NVOL_WT_ADDR        (27)
#define MCP47CXBXX_MTP_1C_ADDR         (28)
#define MCP47CXBXX_MTP_1D_ADDR         (29)
#define MCP47CXBXX_MTP_1E_ADDR         (30)
#define MCP47CXBXX_MTP_1F_ADDR         (31)
/** @} */

/**
 * @name    MCP47CXBXX Vref settings
 *
 * VOLTAGE REFERENCE (VREF) CONTROL REGISTERS
 *
 * @{
 */

#define MCP47CXBXX_CONF_VREF_BUFF_0     ((1 << 1) | (1 << 0))
#define MCP47CXBXX_CONF_VREF_UNBUFF_0   ((1 << 1) | (0 << 0))
#define MCP47CXBXX_CONF_INT_BGAP_0      ((0 << 1) | (1 << 0))
#define MCP47CXBXX_CONF_VDD_UNBUFF_0    ((1 << 1) | (1 << 0))
#define MCP47CXBXX_CONF_VREF_BUFF_1     ((1 << 3) | (1 << 2))
#define MCP47CXBXX_CONF_VREF_UNBUFF_1   ((1 << 3) | (0 << 2))
#define MCP47CXBXX_CONF_INT_BGAP_1      ((0 << 3) | (1 << 2))
#define MCP47CXBXX_CONF_VDD_UNBUFF_1    ((1 << 3) | (1 << 2))

/**
 * @name    MCP47CXBXX PowerDown settings
 *
 * POWER-DOWN CONTROL REGISTERS
 *
 * @{
 */

#define MCP47CXBXX_CONF_PD_OPEN_0     ((1 << 1) | (1 << 0))
#define MCP47CXBXX_CONF_PD_100K_0     ((1 << 1) | (0 << 0))
#define MCP47CXBXX_CONF_PD_1K_0       ((0 << 1) | (1 << 0))
#define MCP47CXBXX_CONF_PD_OFF_0      ((1 << 1) | (1 << 0))
#define MCP47CXBXX_CONF_PD_OPEN_1     ((1 << 3) | (1 << 2))
#define MCP47CXBXX_CONF_PD_100K_1     ((1 << 3) | (0 << 2))
#define MCP47CXBXX_CONF_PD_1K_1       ((0 << 3) | (1 << 2))
#define MCP47CXBXX_CONF_PD_OFF_1      ((1 << 3) | (1 << 2))

/**
 * @name    MCP47CXBXX Gain settings
 *
 * GAIN CONTROL AND SYSTEM STATUS REGISTER
 *
 * @{
 */
#define MCP47CXBXX_CONF_GAIN_MASK_0		((0xFC3F))
#define MCP47CXBXX_CONF_GAIN_MASK_1		((1 << 8))
#define MCP47CXBXX_CONF_GAIN_0_1X		((0 << 8))
#define MCP47CXBXX_CONF_GAIN_1_1X		((0 << 9))
#define MCP47CXBXX_CONF_GAIN_0_2X		((1 << 8))
#define MCP47CXBXX_CONF_GAIN_1_2X		((1 << 9))

/**
 * @name    MCP47CXBXX DAC settings
 *
 * DAC initial value
 *
 * @{
 */
#define MCP47CXBXX_CONF_DAC_0		((0x000))
#define MCP47CXBXX_CONF_DAC_1		((0x000))


#ifdef __cplusplus
}
#endif

#endif /* MCP47CXBXX_REGS_H */
/** @} */
