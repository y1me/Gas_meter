/*
 * glasslcd.h
 *
 *  Created on: Sep 12, 2023
 *      Author: blobby
 */

#ifndef INC_PERIPH_GLASSLCD_H_
#define INC_PERIPH_GLASSLCD_H_

#include "main.h"

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

#define LINE_DEFAULT_MASK 	0xF0F0F0F0

#define LINE_STATE_RESET 	0xF0F0F0F0

#define ALL_LCD_LINE		0x63F

#define LINE0_SHIFT 		0
#define LINE1_SHIFT 		1
#define LINE2_SHIFT 		2
#define LINE3_SHIFT 		3
#define LINE4_SHIFT 		4
#define LINE5_SHIFT 		5

typedef struct lcd_segments {
    uint32_t line0;			/* lcd line segment 0 */
    uint32_t line1;			/* lcd line segment 1 */
    uint32_t line2;			/* lcd line segment 2 */
    uint32_t line3;			/* lcd line segment 3 */
    uint32_t line4;			/* lcd line segment 4 */
    uint32_t line5;			/* lcd line segment 5 */
} lcd_segments_t;


/**
 * @brief   lcd segments defaults
 */
static lcd_segments_t lcd_segments_value[] =
{
    {
    .line0 = LINE_STATE_RESET,\
	.line1 = LINE_STATE_RESET,\
	.line2 = LINE_STATE_RESET,\
	.line3 = LINE_STATE_RESET,\
	.line4 = LINE_STATE_RESET,\
	.line5 = LINE_STATE_RESET
    }
};

static uint32_t count_32;

void Running_glasslcd_StateMachine_Iteration(void);


#endif /* INC_PERIPH_GLASSLCD_H_ */
