/*
                              *******************
******************************* C SOURCE FILE *******************************
**                            *******************                          **
**                                                                         **
** project  : VersaSens                                                    **
** filename : app_data.c                                                   **
** version  : 1                                                            **
** date     : DD/MM/YY                                                     **
**                                                                         **
*****************************************************************************
**                                                                         **
** Copyright (c) EPFL                                                      **
** All rights reserved.                                                    **
**                                                                         **
*****************************************************************************
	 
VERSION HISTORY:
----------------
Version     : 1
Date        : 10/02/2021
Revised by  : Benjamin Duc
Description : Original version.


*/

/***************************************************************************/
/***************************************************************************/

/**
* @file   app_data.c
* @date   DD/MM/YY
* @brief  This is the main header of app_data.c
*
* Here typically goes a more extensive explanation of what the header
* defines.
*/

#define _APP_DATA_C_SRC

/****************************************************************************/
/**                                                                        **/
/*                             MODULES USED                                 */
/**                                                                        **/
/****************************************************************************/

#include <stdlib.h>
#include "app_data.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

/****************************************************************************/
/**                                                                        **/
/*                        DEFINITIONS AND MACROS                            */
/**                                                                        **/
/****************************************************************************/

LOG_MODULE_REGISTER(app_data, LOG_LEVEL_INF);

#define APP_DATA_FIFO_MAX_SIZE 10

#define APP_DATA_BNO_FIFO_MAX   50
#define APP_DATA_AUD_FIFO_MAX   9600

/****************************************************************************/
/**                                                                        **/
/*                        TYPEDEFS AND STRUCTURES                           */
/**                                                                        **/
/****************************************************************************/

/****************************************************************************/
/**                                                                        **/
/*                      PROTOTYPES OF LOCAL FUNCTIONS                       */
/**                                                                        **/
/****************************************************************************/

/****************************************************************************/
/**                                                                        **/
/*                           EXPORTED VARIABLES                             */
/**                                                                        **/
/****************************************************************************/

/****************************************************************************/
/**                                                                        **/
/*                            GLOBAL VARIABLES                              */
/**                                                                        **/
/****************************************************************************/

K_FIFO_DEFINE(app_data_fifo);

K_FIFO_DEFINE(bno086_data_fifo);
K_FIFO_DEFINE(t5838_data_fifo);

uint8_t app_data_fifo_counter = 0;

uint8_t app_data_BNO_fifo_counter = 0;
uint8_t app_data_AUD_fifo_counter = 0;

/****************************************************************************/
/**                                                                        **/
/*                           EXPORTED FUNCTIONS                             */
/**                                                                        **/
/****************************************************************************/

void app_data_add_to_fifo(uint8_t *data, size_t size)
{
    if (app_data_fifo_counter >= APP_DATA_FIFO_MAX_SIZE)
    {
        // LOG_INF("BNO FIFO FULL!\n");
        return;
    }

    // Allocate a new sensor data
	struct app_data_struct *new_data = k_malloc(sizeof(*new_data));

    if (new_data == NULL)
    {
        LOG_ERR("Failed to allocate memory for new_data\n");
        return;
    }

	// Set the sensor data
	new_data->size = size < MAX_DATA_SIZE_APP_DATA ? size : MAX_DATA_SIZE_APP_DATA;
	memcpy(new_data->data, data, new_data->size);

	// Put the sensor data in the FIFO
	k_fifo_put(&app_data_fifo, new_data);
    // LOG_INF("ADDEDD DATA (app_data)!\n");
    app_data_fifo_counter++;
}


void add_bno086_data_to_fifo(uint8_t *data, size_t size){

    if (app_data_BNO_fifo_counter >= APP_DATA_BNO_FIFO_MAX){
        LOG_INF("BNO FIFO FULL!\n");
        return;
    }

	struct app_data_struct *new_data = k_malloc(sizeof(*new_data));
    if (new_data == NULL)
    {
        LOG_ERR("Failed to allocate memory for new_data\n");
        return;
    }

	new_data->size = size < MAX_DATA_SIZE_APP_DATA ? size : MAX_DATA_SIZE_APP_DATA;
	memcpy(new_data->data, data, new_data->size);
    
	// Put the sensor data in the FIFO
	k_fifo_put(&bno086_data_fifo, new_data);
    LOG_INF("ADDEDD DATA (bno_foo)!\n");
    app_data_BNO_fifo_counter+=10;
}

// Reads 10 samples of 13 bytes (index included)
uint8_t get_bno086_data_from_fifo(uint8_t *data){

    // LOG_DBG("INSIDE!");
    // return -1;

    // struct app_data_struct *new_data = k_fifo_get(&bno086_data_fifo, K_MSEC(10));
    struct app_data_struct *new_data = k_fifo_get(&bno086_data_fifo, K_FOREVER);
    
    if(new_data != NULL){
        // LOG_INF("GET Succeded\n");
        memcpy(data, new_data->data, 13*10);
        k_free(new_data);
        app_data_BNO_fifo_counter-=10;
        return -1;
    } else {
        LOG_INF("NO_DATA\n");
        return NULL;
    }
}




void add_t5838_data_to_fifo(uint8_t *data, size_t size){

	struct app_data_struct *new_data = k_malloc(sizeof(*new_data));
    if (new_data == NULL)
    {
        LOG_ERR("Failed to allocate memory for new_data\n");
        return;
    }

	memcpy(new_data->data, data, new_data->size);

	// Put the sensor data in the FIFO
	k_fifo_put(&t5838_data_fifo, new_data);
}


/****************************************************************************/
/****************************************************************************/

void app_data_get_from_fifo(struct app_data_struct *data)
{
    struct app_data *p_data = k_fifo_get(&app_data_fifo, K_NO_WAIT);
    if (p_data != NULL)
    {
        memcpy(data, p_data, sizeof(*data));
        k_free(p_data);
        app_data_fifo_counter--;
    }
}

/****************************************************************************/
/**                                                                        **/
/*                            LOCAL FUNCTIONS                               */
/**                                                                        **/
/****************************************************************************/

/****************************************************************************/
/**                                                                        **/
/*                                 EOF                                      */
/**                                                                        **/
/****************************************************************************/
