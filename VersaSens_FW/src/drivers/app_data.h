/*
                              *******************
******************************* H SOURCE FILE *******************************
**                            *******************                          **
**                                                                         **
** project  : VersaSens                                                        **
** filename : app_data.h                                                   **
** version  : 1                                                            **
** date     : DD/MM/21                                                     **
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
* @file   app_data.h
* @date   DD/MM/YY
* @brief  This is the main header of app_data.c
*
* Here typically goes a more extensive explanation of what the header
* defines.
*/

#ifndef _APP_DATA_H
#define _APP_DATA_H

/****************************************************************************/
/**                                                                        **/
/**                            MODULES USED                                **/
/**                                                                        **/
/****************************************************************************/

// #include "sdk_common.h"
#include <zephyr/types.h>
#include <zephyr/kernel.h>

/****************************************************************************/
/**                                                                        **/
/**                       DEFINITIONS AND MACROS                           **/
/**                                                                        **/
/****************************************************************************/

#define MAX_DATA_SIZE_APP_DATA 250

/****************************************************************************/
/**                                                                        **/
/**                       TYPEDEFS AND STRUCTURES                          **/
/**                                                                        **/
/****************************************************************************/

struct app_data_struct {
	void *fifo_reserved;  // reserved for use by k_fifo
	uint8_t data[MAX_DATA_SIZE_APP_DATA];  // sensor data
	size_t size;  // size of the data
};

/****************************************************************************/
/**                                                                        **/
/**                          EXPORTED VARIABLES                            **/
/**                                                                        **/
/****************************************************************************/

#ifndef _APP_DATA_C_SRC



#endif  /* _APP_DATA_C_SRC */

/****************************************************************************/
/**                                                                        **/
/**                          EXPORTED FUNCTIONS                            **/
/**                                                                        **/
/****************************************************************************/

/*
    * @brief This function adds the data to the FIFO
    * 
    * @param data the sensor data
    * @param size the size of the data
*/
void app_data_add_to_fifo(uint8_t *data, size_t size);

/****************************************************************************/
/****************************************************************************/

/*
    * @brief This function gets the data from the FIFO
    * 
    * @param data the sensor data
*/
void app_data_get_from_fifo(struct app_data_struct *data);

/****************************************************************************/
/**                                                                        **/
/**                          INLINE FUNCTIONS                              **/
/**                                                                        **/
/****************************************************************************/



#endif /* _APP_DATA_H */
/****************************************************************************/
/**                                                                        **/
/**                                EOF                                     **/
/**                                                                        **/
/****************************************************************************/