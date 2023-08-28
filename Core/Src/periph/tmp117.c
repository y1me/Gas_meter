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

int16_t tmp117_init(tmp117_params_t *);
int16_t tmp117_get_temperature(tmp117_params_t *);
int16_t tmp117_reset(tmp117_params_t *);
int16_t tmp117_start_conversion(tmp117_params_t *);



void TMP117_StateMachine_Iteration(tmp117_params_t *);
/* USER CODE END Private Prototypes */

typedef struct {
    const char * name;
    int16_t (* const func)(tmp117_params_t *);
} stateFunctionRow_t;

static stateFunctionRow_t TMP117_stateFunction[] = {
        // NAME         // FUNC
	{ "ST_TMP117_INIT",			tmp117_init },
	{ "ST_TMP117_CONV",			tmp117_start_conversion },
	{ "ST_TMP117_READ",			tmp117_get_temperature },
    { "ST_TMP117_ERROR",		tmp117_reset }
};

typedef struct {
	state_tmp117_t currState;
    event_tmp117_t event;
    state_tmp117_t nextState;
} stateTransMatrixRow_t;

static stateTransMatrixRow_t TMP117_stateTransMatrix[] = {
    // CURR STATE     // EVENT           // NEXT STATE
    { ST_TMP117_INIT,		EV_TMP117_INIT_DONE,		ST_TMP117_CONV  },
	{ ST_TMP117_CONV,		EV_TMP117_CONV_DONE,		ST_TMP117_READ  },
	{ ST_TMP117_READ,		EV_TMP117_TEMP_READED,		ST_TMP117_CONV  },
    { ST_TMP117_ERROR,  	EV_TMP117_NONE,				ST_TMP117_INIT  },
    { ST_TMP117_ERROR,  	EV_TMP117_ERROR_OCCUR,		ST_TMP117_ERROR  },
	{ ST_TMP117_INIT,		EV_TMP117_ERROR_OCCUR,		ST_TMP117_ERROR  },
	{ ST_TMP117_CONV,		EV_TMP117_ERROR_OCCUR,		ST_TMP117_ERROR  },
	{ ST_TMP117_READ,		EV_TMP117_ERROR_OCCUR,		ST_TMP117_ERROR  }
};

int16_t tmp117_reset(tmp117_params_t *params)
{
	int16_t ret = 0;
	uint16_t data_init[2] = TMP117_INIT_PARAMS;
	data_init[1] |= TMP117_CONF_SOFT_RESET;
	params->pointer = TMP117_CONFIGURATION_ADDR;
	params->config[1] = data_init[1];
	params->config[0] = (data_init[1] >> 8);
	if (I2C_status() != I2C_FREE)
	{
		ret = TMP117_I2CBUSY;
		goto out;
	}

	ret = write_I2C_device_DMA(params->i2cHandle, params->addr, &params->pointer, 3);
	if (ret == I2C_OK)
	{
		params->currState=ST_TMP117_ERROR;
		params->event=EV_TMP117_NONE;
	}
	else
	{
		params->currState=ST_TMP117_ERROR;
		params->event=EV_TMP117_ERROR_OCCUR;
	}

	out:
	return ret;

}


int16_t tmp117_init(tmp117_params_t *params)
{
	int16_t ret = 0;
	uint16_t data_init[2] = TMP117_INIT_PARAMS;

	params->pointer = TMP117_CONFIGURATION_ADDR;
	params->config[1] = data_init[1];
	params->config[0] = (data_init[1] >> 8);

	if (I2C_status() != I2C_FREE)
	{
		ret = TMP117_I2CBUSY;
		goto out;
	}

	ret = write_I2C_device_DMA(params->i2cHandle, params->addr, &params->pointer, 3);
	if (ret == I2C_OK)
	{
		params->currState=ST_TMP117_INIT;
		params->event=EV_TMP117_NONE;
	}
	else
	{
		params->currState=ST_TMP117_INIT;
		params->event=EV_TMP117_ERROR_OCCUR;
	}

	out:
	return ret;

}

int16_t tmp117_start_conversion(tmp117_params_t *params)
{
	int16_t ret = 0;

	params->pointer = TMP117_CONFIGURATION_ADDR;
	params->config[1] |= TMP117_CONF_MOD_OS;

	if (I2C_status() != I2C_FREE)
	{
		ret = TMP117_I2CBUSY;
		goto out;
	}

	ret = write_I2C_device_DMA(params->i2cHandle, params->addr, &params->pointer, 3);
	if (ret == I2C_OK)
	{
		params->currState=ST_TMP117_CONV;
		params->event=EV_TMP117_NONE;
	}
	else
	{
		params->currState=ST_TMP117_CONV;
		params->event=EV_TMP117_ERROR_OCCUR;
	}

	out:
	return ret;

}

int16_t tmp117_get_temperature(tmp117_params_t *params)
{
	int16_t ret = 0;

	params->pointer = TMP117_TEMP_RESULT_ADDR;

	if (I2C_status() != I2C_FREE)
	{
		ret = TMP117_I2CBUSY;
		goto out;
	}

	ret = write_read_I2C_device_DMA(params->i2cHandle, params->addr, &params->pointer, params->temperature, 1, 2);
	if (ret == I2C_OK)
	{
		params->currState = ST_TMP117_READ;
		params->event=EV_TMP117_NONE;
	}
	else
	{
		params->currState = ST_TMP117_READ;
		params->event=EV_TMP117_ERROR_OCCUR;
	}

	out:
	return ret;

}

void Running_TMP117_StateMachine_Iteration(void)
{
	TMP117_StateMachine_Iteration(tmp117_params);
}

void TMP117_StateMachine_Iteration(tmp117_params_t *params)
{
	if (I2C_status() == I2C_FREE && params->event == EV_TMP117_NONE)
	{
		I2C_clear_last_event();
		if(params->currState == ST_TMP117_INIT)
		{
			params->event = EV_TMP117_INIT_DONE;
		}
		if(params->currState == ST_TMP117_CONV)
		{
			params->event = EV_TMP117_CONV_DONE;
		}
		if(params->currState == ST_TMP117_READ)
		{
			params->event = EV_TMP117_TEMP_READED;
		}
//		if(params->currState == ST_TMP117_ERROR)
//		{
//			params->event = EV_TMP117_ERROR_OCCUR;
//		}

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


