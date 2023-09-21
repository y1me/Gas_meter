/*
 * glasslcd.c
 *
 *  Created on: Sep 12, 2023
 *      Author: blobby
 */
#include "periph/glasslcd.h"

void update_lcd(uint8_t );
void update_com_line(const uint32_t );
void update_data_line(const uint32_t , const lcd_segments_t *);
void set_deadtime_state(void);



void Running_glasslcd_StateMachine_Iteration(void)
{
	update_lcd(count_32);
	count_32++;
	if (count_32 >= 32)
	{
		count_32 = 0;
	}
}
// count++; reset when count == 4
void update_lcd(uint8_t count)
{
	uint8_t mod4 = count % 4;

	if (mod4 == 3)
	{
		set_deadtime_state();
	}
	else
	{
		update_com_line(1 << count);
		update_data_line(1 << count, lcd_segments_value);
	}

}

void set_deadtime_state(void)
{
	LL_GPIO_SetPinMode(GPIOA, ALL_LCD_LINE, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_ResetOutputPin(GPIOA, ALL_LCD_LINE);
}

void update_com_line(const uint32_t mask)
{
	uint32_t com1, com2, com3, com4;

	// set com port to input
	LL_GPIO_SetPinMode(GPIOA, COM_PORT_MASK, LL_GPIO_MODE_INPUT);

	com1 = ((mask & COM1_ONE_MASK) >> COM1_ONE_SHIFT) | ((mask & (COM1_ONE_MASK >> 1)) >> (COM1_ONE_SHIFT -1)) | ((mask & (COM1_ONE_MASK >> 2)) >> (COM1_ONE_SHIFT -2)) | ((mask & (COM1_ONE_MASK >> 3)) >> (COM1_ONE_SHIFT -3));

	com2 = ((mask & COM2_ONE_MASK) >> COM2_ONE_SHIFT) | ((mask & (COM2_ONE_MASK >> 1)) >> (COM2_ONE_SHIFT -1)) | ((mask & (COM2_ONE_MASK >> 2)) >> (COM2_ONE_SHIFT -2)) | ((mask & (COM2_ONE_MASK >> 3)) >> (COM2_ONE_SHIFT -3));

	com3 = ((mask & COM3_ONE_MASK) >> COM3_ONE_SHIFT) | ((mask & (COM3_ONE_MASK >> 1)) >> (COM3_ONE_SHIFT -1)) | ((mask & (COM3_ONE_MASK >> 2)) >> (COM3_ONE_SHIFT -2)) | ((mask & (COM3_ONE_MASK >> 3)) >> (COM3_ONE_SHIFT -3));

	com4 = ((mask & COM4_ONE_MASK) << COM4_ONE_SHIFT) | ((mask & (COM4_ONE_MASK << 1)) << (COM4_ONE_SHIFT +1)) | ((mask & (COM4_ONE_MASK << 2)) << (COM4_ONE_SHIFT +2)) | ((mask & (COM4_ONE_MASK << 3)) << (COM4_ONE_SHIFT +3));

	LL_GPIO_SetPinMode(GPIOA, com1|com2|com3|com4, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetOutputPin(GPIOA, com1|com2|com3|com4);

	com1 = ((mask & COM1_ZERO_MASK) >> COM1_ZERO_SHIFT) | ((mask & (COM1_ZERO_MASK >> 1)) >> (COM1_ZERO_SHIFT -1)) | ((mask & (COM1_ZERO_MASK >> 2)) >> (COM1_ZERO_SHIFT -2)) | ((mask & (COM1_ZERO_MASK >> 3)) >> (COM1_ZERO_SHIFT -3));

	com2 = ((mask & COM2_ZERO_MASK) >> COM2_ZERO_SHIFT) | ((mask & (COM2_ZERO_MASK >> 1)) >> (COM2_ZERO_SHIFT -1)) | ((mask & (COM2_ZERO_MASK >> 2)) >> (COM2_ZERO_SHIFT -2)) | ((mask & (COM2_ZERO_MASK >> 3)) >> (COM2_ZERO_SHIFT -3));

	com3 = ((mask & COM3_ZERO_MASK) << COM3_ZERO_SHIFT) | ((mask & (COM3_ZERO_MASK >> 1)) << (COM3_ZERO_SHIFT +1)) | ((mask & (COM3_ZERO_MASK >> 2)) << (COM3_ZERO_SHIFT +2)) | ((mask & (COM3_ZERO_MASK >> 3)) << (COM3_ZERO_SHIFT +3));

	com4 = ((mask & COM4_ZERO_MASK) << COM4_ZERO_SHIFT) | ((mask & (COM4_ZERO_MASK >> 1)) << (COM4_ZERO_SHIFT +1)) | ((mask & (COM4_ZERO_MASK >> 2)) << (COM4_ZERO_SHIFT +2)) | ((mask & (COM4_ZERO_MASK >> 3)) << (COM4_ZERO_SHIFT +3));

	LL_GPIO_SetPinMode(GPIOA, com1|com2|com3|com4, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_ResetOutputPin(GPIOA, com1|com2|com3|com4);

}

void update_data_line(const uint32_t mask, const lcd_segments_t *segments)
{
	uint32_t line_ones, line_zeros, active_bit = 1 << mask;

	line_ones = (((segments->line0 & active_bit) >> mask) << LINE0_SHIFT) |
				(((segments->line1 & active_bit) >> mask) << LINE1_SHIFT) |
				(((segments->line2 & active_bit) >> mask) << LINE2_SHIFT) |
				(((segments->line3 & active_bit) >> mask) << LINE3_SHIFT) |
				(((segments->line4 & active_bit) >> mask) << LINE4_SHIFT) |
				(((segments->line5 & active_bit) >> mask) << LINE5_SHIFT);
	LL_GPIO_SetOutputPin(GPIOA, line_ones);

	line_zeros = (((~segments->line0 & active_bit) >> mask) << LINE0_SHIFT) |
				 (((~segments->line1 & active_bit) >> mask) << LINE1_SHIFT) |
				 (((~segments->line2 & active_bit) >> mask) << LINE2_SHIFT) |
				 (((~segments->line3 & active_bit) >> mask) << LINE3_SHIFT) |
				 (((~segments->line4 & active_bit) >> mask) << LINE4_SHIFT) |
				 (((~segments->line5 & active_bit) >> mask) << LINE5_SHIFT);
	LL_GPIO_ResetOutputPin(GPIOA, line_zeros);
}
