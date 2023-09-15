/*
 * glasslcd.h
 *
 *  Created on: Sep 12, 2023
 *      Author: blobby
 */

#ifndef INC_PERIPH_GLASSLCD_H_
#define INC_PERIPH_GLASSLCD_H_

#define COM_PORT_MASK 0x00001E00

#define COM1_ONE_SHIFT 		22
#define COM1_ONE_MASK 		0x80000000
#define COM1_ZERO_SHIFT 	18
#define COM1_ZERO_MASK 		0x08000000

#define COM2_ONE_SHIFT 		13
#define COM2_ONE_MASK 		0x00800000
#define COM2_ZERO_SHIFT 	9
#define COM2_ZERO_MASK 		0x00080000

#define COM3_ONE_SHIFT 		4
#define COM3_ONE_MASK 		0x00008000
#define COM3_ZERO_SHIFT 	0
#define COM3_ZERO_MASK 		0x00000800

#define COM4_ONE_SHIFT 		5
#define COM4_ONE_MASK 		0x00000080
#define COM4_ZERO_SHIFT 	9
#define COM4_ZERO_MASK 		0x00000008
//typedef struct ads101x_alert_params {
//    uint32_t alert_pin_status;			/**< alert pin (GPIO_UNDEF if not connected) */
//    int16_t low_limit;					/**< alert low value */
//    int16_t high_limit;					/**< alert high value */
//} ads101x_alert_params_t;

#endif /* INC_PERIPH_GLASSLCD_H_ */
