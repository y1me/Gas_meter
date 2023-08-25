/*
 * tmp117.c
 *
 *  Created on: Aug 25, 2023
 *      Author: blobby
 */

#include "assert.h"


#include "periph/tmp117.h"
#include "periph/tmp117_params.h"
#include "periph/tmp117_regs.h"

#include "i2c.h"
#include "Utils/Commons.h"


//#ifndef ADS101X_READ_DELAY
//#define ADS101X_READ_DELAY (8 * US_PER_MS)    /* Compatible with 128SPS */
//#endif

//#define DEV (dev->params.i2c)
//#define ADDR (dev->params.addr)

int16_t tmp117_reset(tmp117_params_t *);
int16_t tmp117_set_vref(tmp117_params_t *);
int16_t tmp117_set_gain(tmp117_params_t *);
int16_t tmp117_set_powerdown(tmp117_params_t *);
int16_t tmp117_set_dac0(tmp117_params_t *);
int16_t tmp117_set_dac1(tmp117_params_t *);


void TMP117_StateMachine_Iteration(tmp117_params_t *);
/* USER CODE END Private Prototypes */

typedef struct {
    const char * name;
    int16_t (* const func)(tmp117_params_t *);
} stateFunctionRow_t;

static stateFunctionRow_t TMP117_stateFunction[] = {
        // NAME         // FUNC
	{ "ST_TMP117_RESET",		tmp117_reset },
	{ "ST_TMP117_SET_VREF",	tmp117_set_vref },
	{ "ST_TMP117_SET_PDOWN",	tmp117_set_powerdown },
	{ "ST_TMP117_SET_GAIN",	tmp117_set_gain },
    { "ST_TMP117_SET_DAC0",	tmp117_set_dac0 },
	{ "ST_TMP117_SET_DAC1",	tmp117_set_dac1 },
	{ "ST_TMP117_IDLE",		NULL },
    { "ST_TMP117_ERROR",		tmp117_reset }
};

typedef struct {
	state_mcp47_t currState;
    event_mcp47_t event;
    state_mcp47_t nextState;
} stateTransMatrixRow_t;

static stateTransMatrixRow_t TMP117_stateTransMatrix[] = {
    // CURR STATE  v// EVENT           // NEXT STATE
    { ST_TMP117_RESET,			EV_TMP117_RESET_DONE,		ST_TMP117_SET_VREF  },
	{ ST_TMP117_SET_VREF,		EV_TMP117_VREF_DONE,			ST_TMP117_SET_PDOWN  },
	{ ST_TMP117_SET_PDOWN,		EV_TMP117_PDOWN_DONE,		ST_TMP117_SET_GAIN  },
    { ST_TMP117_SET_GAIN,		EV_TMP117_GAIN_DONE,			ST_TMP117_SET_DAC0 },
    { ST_TMP117_SET_DAC0,		EV_TMP117_DAC0_DONE,			ST_TMP117_SET_DAC1 },
    { ST_TMP117_SET_DAC1,  		EV_TMP117_DAC1_DONE,			ST_TMP117_SET_VREF  },
    { ST_TMP117_RESET,			EV_TMP117_ERROR_OCCUR,		ST_TMP117_RESET  },
	{ ST_TMP117_SET_VREF,		EV_TMP117_ERROR_OCCUR,		ST_TMP117_IDLE  },
	{ ST_TMP117_SET_PDOWN,		EV_TMP117_ERROR_OCCUR,		ST_TMP117_IDLE  },
    { ST_TMP117_SET_GAIN,		EV_TMP117_ERROR_OCCUR,		ST_TMP117_IDLE },
    { ST_TMP117_SET_DAC0,		EV_TMP117_ERROR_OCCUR,		ST_TMP117_IDLE },
    { ST_TMP117_SET_DAC1,  		EV_TMP117_ERROR_OCCUR,		ST_TMP117_IDLE  }
};

/* Buffer used for transmission */
uint8_t mTxBuffer[TMP117_TX_DATA_SIZE];

/* Buffer used for reception */
uint8_t mRxBuffer[TMP117_RX_DATA_SIZE];


int16_t tmp117_reset(tmp117_params_t *params)
{
	int16_t ret;
	params->pdown = TMP117_CONF_PD_OFF_0 | TMP117_CONF_PD_OFF_1;
	params->pointer = TMP117_VOL_PD_ADDR << TMP117_ADD_POINTER_SHIFT;
	params->config[1] = params->pdown;
	params->config[0] = (params->pdown >> 8);
	if (I2C_status() != I2C_FREE)
	{
		ret = TMP117_I2CBUSY;
		goto out;
	}

	ret = write_I2C_device_DMA(params->i2cHandle, params->addr, &params->pointer, 3);
	if (ret == I2C_OK)
	{
		params->currState=ST_TMP117_RESET;
		params->event=EV_TMP117_NONE;
	}
	else
	{
		params->currState=ST_TMP117_RESET;
		params->event=EV_TMP117_NONE;
	}

	out:
	return ret;

}


int16_t tmp117_set_vref(tmp117_params_t *params)
{
	int16_t ret = 0;

	if (params->vref == params->loaded_vref)
	{
		params->currState=ST_TMP117_SET_VREF;
		params->event=EV_TMP117_NONE;
		goto out;
	}

	params->pointer = TMP117_VOL_VREF_ADDR << TMP117_ADD_POINTER_SHIFT;
	params->config[1] = params->vref;
	params->config[0] = (params->vref >> 8);

	if (I2C_status() != I2C_FREE)
	{
		ret = TMP117_I2CBUSY;
		goto out;
	}

	ret = write_I2C_device_DMA(params->i2cHandle, params->addr, &params->pointer, 3);
	if (ret == I2C_OK)
	{
		params->currState=ST_TMP117_SET_VREF;
		params->event=EV_TMP117_NONE;
	}
	else
	{
		params->currState=ST_TMP117_RESET;
		params->event=EV_TMP117_NONE;
	}

	out:
	return ret;

}

int16_t tmp117_set_gain(tmp117_params_t *params)
{
	int16_t ret = 0;

	if (params->gain == params->loaded_gain)
	{
		params->currState=ST_TMP117_SET_GAIN;
		params->event=EV_TMP117_NONE;
		goto out;
	}

	params->pointer = TMP117_VOL_G_S_ADDR << TMP117_ADD_POINTER_SHIFT;
	params->config[1] = params->gain;
	params->config[0] = (params->gain >> 8);

	if (I2C_status() != I2C_FREE)
	{
		ret = TMP117_I2CBUSY;
		goto out;
	}

	ret = write_I2C_device_DMA(params->i2cHandle, params->addr, &params->pointer, 3);
	if (ret == I2C_OK)
	{
		params->currState=ST_TMP117_SET_GAIN;
		params->event=EV_TMP117_NONE;
	}
	else
	{
		params->currState=ST_TMP117_RESET;
		params->event=EV_TMP117_NONE;
	}

	out:
	return ret;

}

int16_t tmp117_set_powerdown(tmp117_params_t *params)
{
	int16_t ret = 0;

	if (params->pdown == params->loaded_pdown)
	{
		params->currState=ST_TMP117_SET_PDOWN;
		params->event=EV_TMP117_NONE;
		goto out;
	}

	params->pointer = TMP117_VOL_PD_ADDR << TMP117_ADD_POINTER_SHIFT;
	params->config[1] = params->pdown;
	params->config[0] = (params->pdown >> 8);

	if (I2C_status() != I2C_FREE)
	{
		ret = TMP117_I2CBUSY;
		goto out;
	}

	ret = write_I2C_device_DMA(params->i2cHandle, params->addr, &params->pointer, 3);
	if (ret == I2C_OK)
	{
		params->currState=ST_TMP117_SET_PDOWN;
		params->event=EV_TMP117_NONE;
	}
	else
	{
		params->currState=ST_TMP117_RESET;
		params->event=EV_TMP117_NONE;
	}

	out:
	return ret;

}

int16_t tmp117_set_dac0(tmp117_params_t *params)
{
	int16_t ret = 0;

	if (params->dac0 == params->loaded_dac0)
	{
		params->currState=ST_TMP117_SET_DAC0;
		params->event=EV_TMP117_NONE;
		goto out;
	}

	params->pointer = TMP117_VOL_DAC0_ADDR << TMP117_ADD_POINTER_SHIFT;
	params->config[1] = params->dac0;
	params->config[0] = (params->dac0 >> 8);

	if (I2C_status() != I2C_FREE)
	{
		ret = TMP117_I2CBUSY;
		goto out;
	}

	ret = write_I2C_device_DMA(params->i2cHandle, params->addr, &params->pointer, 3);
	if (ret == I2C_OK)
	{
		params->currState=ST_TMP117_SET_DAC0;
		params->event=EV_TMP117_NONE;
	}
	else
	{
		params->currState=ST_TMP117_RESET;
		params->event=EV_TMP117_NONE;
	}

	out:
	return ret;

}

int16_t tmp117_set_dac1(tmp117_params_t *params)
{
	int16_t ret = 0;

	if (params->dac1 == params->loaded_dac1)
	{
		params->currState=ST_TMP117_SET_DAC1;
		params->event=EV_TMP117_NONE;
		goto out;
	}

	params->pointer = TMP117_VOL_DAC1_ADDR << TMP117_ADD_POINTER_SHIFT;
	params->config[1] = params->dac1;
	params->config[0] = (params->dac1 >> 8);

	if (I2C_status() != I2C_FREE)
	{
		ret = TMP117_I2CBUSY;
		goto out;
	}


	ret = write_I2C_device_DMA(params->i2cHandle, params->addr, &params->pointer, 3);
	if (ret == I2C_OK)
	{
		params->currState=ST_TMP117_SET_DAC1;
		params->event=EV_TMP117_NONE;
	}
	else
	{
		params->currState=ST_TMP117_RESET;
		params->event=EV_TMP117_NONE;
	}

	out:
	return ret;

}

void Running_TMP117_StateMachine_Iteration(void)
{
	TMP117_StateMachine_Iteration(tmp117_param);
}

void TMP117_StateMachine_Iteration(tmp117_params_t *params)
{
	if (I2C_status() == I2C_FREE && params->event == EV_TMP117_NONE)
	{
		I2C_clear_last_event();
		if(params->currState == ST_TMP117_RESET)
		{
			params->loaded_pdown = params->pdown;
			params->event = EV_TMP117_RESET_DONE;
		}
		if(params->currState == ST_TMP117_SET_VREF)
		{
			params->event = EV_TMP117_VREF_DONE;
		}
		if(params->currState == ST_TMP117_SET_PDOWN)
		{
			params->event = EV_TMP117_PDOWN_DONE;
		}
		if(params->currState == ST_TMP117_SET_GAIN)
		{
			params->event = EV_TMP117_GAIN_DONE;
		}
		if(params->currState == ST_TMP117_SET_DAC0)
		{
			params->event = EV_TMP117_DAC0_DONE;
		}
		if(params->currState == ST_TMP117_SET_DAC1)
		{
			params->event = EV_TMP117_DAC1_DONE;
		}

	}
	else if (I2C_status() == I2C_ERROR || I2C_status() == I2C_ERR_OCCUR)
	{
		params->event = EV_TMP117_ERROR_OCCUR;
	}

    // Iterate through the state transition matrix, checking if there is both a match with the current state
    // and the event
    for(int16_t i = 0; i < sizeof(TMP117_stateTransMatrix)/sizeof(TMP117_stateTransMatrix[0]); i++) {
        if(TMP117_stateTransMatrix[i].currState == params->currState) {
            if(TMP117_stateTransMatrix[i].event == params->event) {

                // Transition to the next state
            	params->currState =  TMP117_stateTransMatrix[i].nextState;

                // Call the function associated with transition
            	if ( (TMP117_stateFunction[params->currState].func) != NULL )
            	{
            		(TMP117_stateFunction[params->currState].func)(params);
            	}
            	break;
            }
        }
    }
}


