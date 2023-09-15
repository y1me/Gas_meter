/*
 * glasslcd.c
 *
 *  Created on: Sep 12, 2023
 *      Author: blobby
 */

// count++; reset when count == 4
void update_lcd(uint8_t count)
{
	mod4 = count % 4;

	if (mod4 == 3)
	{
		set_deadtime_state();
	}
	else
	{
		update_com_line(1 << count);
		update_data_line(1 << count);
	}

}

void update_com_line(uint32_t mask)
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
	LL_GPIO_SetOutputPin(GPIOA, com1|com2|com3|com4);
	LL_GPIO_ResetOutputPin
	// mask output
	// mask input 0x1E00




	LL_GPIO_SetOutputPin

}

if ((output.value > output.size))
	{
		output.value = 1;
	}

	output.value++;

	if ((output.value )%4 ==0)
	{
		output.deadtime = 1;
	}
	else
	{
		output.deadtime = 0;
	}

	if ( output.mask > 0x80000000 || output.mask == 0 )
	{
		output.mask = 1;
	}

	if (output.deadtime)
	{
		LL_GPIO_SetPinMode (GPIOA, LL_GPIO_PIN_6 | LL_GPIO_PIN_7 | LL_GPIO_PIN_8 | LL_GPIO_PIN_9, LL_GPIO_MODE_OUTPUT);
		LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_9 | LL_GPIO_PIN_8 | LL_GPIO_PIN_7 | LL_GPIO_PIN_6 | LL_GPIO_PIN_5 | LL_GPIO_PIN_4 | LL_GPIO_PIN_3 | LL_GPIO_PIN_2 | LL_GPIO_PIN_1 | LL_GPIO_PIN_0);
	}
	else
	{

		if (output.mask & mask_com1)
		{
			LL_GPIO_SetPinMode (GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_OUTPUT);
			if (output.mask & pattern_com1)
			{
				LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_6);
				LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_5 | LL_GPIO_PIN_4 | LL_GPIO_PIN_3 | LL_GPIO_PIN_2 | LL_GPIO_PIN_1 | LL_GPIO_PIN_0);

				LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_3);
			}
			else
			{
				LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_6);
				LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5 | LL_GPIO_PIN_5 | LL_GPIO_PIN_3 | LL_GPIO_PIN_2 | LL_GPIO_PIN_1 | LL_GPIO_PIN_0);

				LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_3);

			}
		}
		else
		{
			LL_GPIO_SetPinMode (GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_INPUT);

		}

		if (output.mask & mask_com2)
		{
			LL_GPIO_SetPinMode (GPIOA, LL_GPIO_PIN_7, LL_GPIO_MODE_OUTPUT);
			if (output.mask & pattern_com2)
			{
				LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_7);
				LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_5 | LL_GPIO_PIN_4 | LL_GPIO_PIN_3 | LL_GPIO_PIN_2 | LL_GPIO_PIN_1 | LL_GPIO_PIN_0);
				LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_3 | LL_GPIO_PIN_4 | LL_GPIO_PIN_0);
			}
			else
			{
				LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_7);
				LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5 | LL_GPIO_PIN_4 | LL_GPIO_PIN_3 | LL_GPIO_PIN_2 | LL_GPIO_PIN_1 | LL_GPIO_PIN_0);
				LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_3 | LL_GPIO_PIN_4 | LL_GPIO_PIN_0);
			}
		}
		else
		{
			LL_GPIO_SetPinMode (GPIOA, LL_GPIO_PIN_7, LL_GPIO_MODE_INPUT);

		}

		if (output.mask & mask_com3)
		{
			LL_GPIO_SetPinMode (GPIOA, LL_GPIO_PIN_8, LL_GPIO_MODE_OUTPUT);
			if (output.mask & pattern_com3)
			{
				LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_8);
				LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_5 | LL_GPIO_PIN_4 | LL_GPIO_PIN_3 | LL_GPIO_PIN_2 | LL_GPIO_PIN_1 | LL_GPIO_PIN_0);
			}
			else
			{
				LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_8);
				LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5 | LL_GPIO_PIN_4 | LL_GPIO_PIN_3 | LL_GPIO_PIN_2 | LL_GPIO_PIN_1 | LL_GPIO_PIN_0);
			}
		}
		else
		{
			LL_GPIO_SetPinMode (GPIOA, LL_GPIO_PIN_8, LL_GPIO_MODE_INPUT);

		}

		if (output.mask & mask_com4)
		{
			LL_GPIO_SetPinMode (GPIOA, LL_GPIO_PIN_9, LL_GPIO_MODE_OUTPUT);
			if (output.mask & pattern_com4)
			{
				LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_9);
				LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_5 | LL_GPIO_PIN_4 | LL_GPIO_PIN_3 | LL_GPIO_PIN_2 | LL_GPIO_PIN_1 | LL_GPIO_PIN_0);
			}
			else
			{
				LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_9);
				LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5 | LL_GPIO_PIN_4 | LL_GPIO_PIN_3 | LL_GPIO_PIN_2 | LL_GPIO_PIN_1 | LL_GPIO_PIN_0);
			}
		}
		else
		{
			LL_GPIO_SetPinMode (GPIOA, LL_GPIO_PIN_9, LL_GPIO_MODE_INPUT);

		}
	}

	output.mask = output.mask << 1;
