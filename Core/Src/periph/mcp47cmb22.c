/*
 * ads1114.c
 *
 *  Created on: Aug 1, 2021
 *      Author: blobby
 */

/*
 * Copyright (C) 2017 OTA keys S.A.
 *               2018 Acutam Automation, LLC
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_mcp47cXbXX
 * @{
 *
 * @file
 * @brief       ADS101x/111x ADC device driver
 *
 * @author      Vincent Dupont <vincent@otakeys.com>
 * @author      Matthew Blue <matthew.blue.neuro@gmail.com>
 * @}
 */

#include "assert.h"


#include "periph/mcp47cXbXX.h"
#include "periph/mcp47cXbXX_params.h"
#include "periph/mcp47cXbXX_regs.h"

#include "i2c.h"
#include "Utils/Commons.h"


//#ifndef ADS101X_READ_DELAY
//#define ADS101X_READ_DELAY (8 * US_PER_MS)    /* Compatible with 128SPS */
//#endif

//#define DEV (dev->params.i2c)
//#define ADDR (dev->params.addr)

int16_t mcp47cXbXX_reset(mcp47cXbXX_params_t *);
int16_t mcp47cXbXX_set_vref(mcp47cXbXX_params_t *);
int16_t mcp47cXbXX_set_gain(mcp47cXbXX_params_t *);
int16_t mcp47cXbXX_set_powerdown(mcp47cXbXX_params_t *);
int16_t mcp47cXbXX_set_dac0(mcp47cXbXX_params_t *);
int16_t mcp47cXbXX_set_dac1(mcp47cXbXX_params_t *);


void MCP47CXBXX_StateMachine_Iteration(mcp47cXbXX_params_t *);
/* USER CODE END Private Prototypes */

typedef struct {
    const char * name;
    int16_t (* const func)(mcp47cXbXX_params_t *);
} stateFunctionRow_t;

static stateFunctionRow_t MCP47CXBXX_stateFunction[] = {
        // NAME         // FUNC
	{ "ST_MCP47_RESET",		mcp47cXbXX_reset },
	{ "ST_MCP47_SET_VREF",	mcp47cXbXX_set_vref },
	{ "ST_MCP47_SET_PDOWN",	mcp47cXbXX_set_powerdown },
	{ "ST_MCP47_SET_GAIN",	mcp47cXbXX_set_gain },
    { "ST_MCP47_SET_DAC0",	mcp47cXbXX_set_dac0 },
	{ "ST_MCP47_SET_DAC1",	mcp47cXbXX_set_dac1 },
	{ "ST_MCP47_IDLE",		NULL },
    { "ST_MCP47_ERROR",		mcp47cXbXX_reset }
};

typedef struct {
	state_mcp47_t currState;
    event_mcp47_t event;
    state_mcp47_t nextState;
} stateTransMatrixRow_t;

static stateTransMatrixRow_t MCP47CXBXX_stateTransMatrix[] = {
    // CURR STATE  v// EVENT           // NEXT STATE
    { ST_MCP47_RESET,			EV_MCP47_RESET_DONE,		ST_MCP47_SET_VREF  },
	{ ST_MCP47_SET_VREF,		EV_MCP47_VREF_DONE,			ST_MCP47_SET_PDOWN  },
	{ ST_MCP47_SET_PDOWN,		EV_MCP47_PDOWN_DONE,		ST_MCP47_SET_GAIN  },
    { ST_MCP47_SET_GAIN,		EV_MCP47_GAIN_DONE,			ST_MCP47_SET_DAC0 },
    { ST_MCP47_SET_DAC0,		EV_MCP47_DAC0_DONE,			ST_MCP47_SET_DAC1 },
    { ST_MCP47_SET_DAC1,  		EV_MCP47_DAC1_DONE,			ST_MCP47_SET_VREF  },
    { ST_MCP47_RESET,			EV_MCP47_ERROR_OCCUR,		ST_MCP47_IDLE  },
	{ ST_MCP47_SET_VREF,		EV_MCP47_ERROR_OCCUR,		ST_MCP47_IDLE  },
	{ ST_MCP47_SET_PDOWN,		EV_MCP47_ERROR_OCCUR,		ST_MCP47_IDLE  },
    { ST_MCP47_SET_GAIN,		EV_MCP47_ERROR_OCCUR,		ST_MCP47_IDLE },
    { ST_MCP47_SET_DAC0,		EV_MCP47_ERROR_OCCUR,		ST_MCP47_IDLE },
    { ST_MCP47_SET_DAC1,  		EV_MCP47_ERROR_OCCUR,		ST_MCP47_IDLE  }
};

/* Buffer used for transmission */
uint8_t mTxBuffer[MCP47CXBXX_TX_DATA_SIZE];

/* Buffer used for reception */
uint8_t mRxBuffer[MCP47CXBXX_RX_DATA_SIZE];


int16_t mcp47cXbXX_reset(mcp47cXbXX_params_t *params)
{
	int16_t ret;
	params->current_pdown = MCP47CXBXX_CONF_PD_OPEN_0 | MCP47CXBXX_CONF_PD_OPEN_1;
	params->pdown = params->current_pdown;
	uint8_t data_init[3] = {params->addr, (params->current_pdown >> 8), params->current_pdown};
		if (I2C_status() != I2C_FREE)
		{
			ret = MCP47CXBXX_I2CBUSY;
			goto out;
		}

		ret = write_I2C_device_DMA(params->i2cHandle, params->addr, &data_init[0], 3);
		if (ret == I2C_OK)
		{
			params->currState=ST_MCP47_RESET;
			params->event=EV_MCP47_NONE;
		}
		else
		{
			params->currState=ST_MCP47_RESET;
			params->event=EV_MCP47_NONE;
		}





		out:
	return ret;

}


int16_t mcp47cXbXX_set_vref(mcp47cXbXX_params_t *params)
{
	int16_t ret = 0;

	return ret;

}

int16_t mcp47cXbXX_set_gain(mcp47cXbXX_params_t *params)
{
	int16_t ret = 0;

	return ret;

}

int16_t mcp47cXbXX_set_powerdown(mcp47cXbXX_params_t *params)
{
	int16_t ret = 0;

	return ret;

}

int16_t mcp47cXbXX_set_dac0(mcp47cXbXX_params_t *params)
{
	int16_t ret = 0;

	return ret;

}

int16_t mcp47cXbXX_set_dac1(mcp47cXbXX_params_t *params)
{
	int16_t ret = 0;

	return ret;

}

void Running_MCP47CXBXX_StateMachine_Iteration(void)
{
	MCP47CXBXX_StateMachine_Iteration(mcp47cXbXX_params);
}

void MCP47CXBXX_StateMachine_Iteration(mcp47cXbXX_params_t *params)
{
	if (I2C_status() == I2C_OK && params->event == EV_MCP47_NONE)
	{
		I2C_clear_last_event();
		if(params->currState == ST_MCP47_RESET)
		{
			params->event = EV_MCP47_RESET_DONE;
		}
		if(params->currState == ST_MCP47_SET_VREF)
		{
			params->event = EV_MCP47_VREF_DONE;
		}
		if(params->currState == ST_MCP47_SET_PDOWN)
		{
			params->event = EV_MCP47_PDOWN_DONE;
		}
		if(params->currState == ST_MCP47_SET_GAIN)
		{
			params->event = EV_MCP47_GAIN_DONE;
		}
		if(params->currState == ST_MCP47_SET_DAC0)
		{
			params->event = EV_MCP47_DAC0_DONE;
		}
		if(params->currState == ST_MCP47_SET_DAC1)
		{
			params->event = EV_MCP47_DAC1_DONE;
		}

	}
	else if (I2C_status() == I2C_ERROR || I2C_status() == I2C_ERR_OCCUR)
	{
		params->event = EV_MCP47_ERROR_OCCUR;
	}

    // Iterate through the state transition matrix, checking if there is both a match with the current state
    // and the event
    for(int16_t i = 0; i < sizeof(MCP47CXBXX_stateTransMatrix)/sizeof(MCP47CXBXX_stateTransMatrix[0]); i++) {
        if(MCP47CXBXX_stateTransMatrix[i].currState == params->currState) {
            if(MCP47CXBXX_stateTransMatrix[i].event == params->event) {

                // Transition to the next state
            	params->currState =  MCP47CXBXX_stateTransMatrix[i].nextState;

                // Call the function associated with transition
            	if ( (MCP47CXBXX_stateFunction[params->currState].func) != NULL )
            	{
            		(MCP47CXBXX_stateFunction[params->currState].func)(params);
            	}
            	break;
            }
        }
    }
}
