/*
                              *******************
******************************* H SOURCE FILE *******************************
**                            *******************                          **
**                                                                         **
** project  : VersaSens                                                        **
** filename : T5838.h                                                   **
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
* @file   T5838.h
* @date   DD/MM/YY
* @brief  This is the main header of T5838.c
*
* Here typically goes a more extensive explanation of what the header
* defines.
*/

#ifndef _T5838_H
#define _T5838_H

/****************************************************************************/
/**                                                                        **/
/**                            MODULES USED                                **/
/**                                                                        **/
/****************************************************************************/

#include <zephyr/types.h>
#include "thread_config.h"

/****************************************************************************/
/**                                                                        **/
/**                       DEFINITIONS AND MACROS                           **/
/**                                                                        **/
/****************************************************************************/

/*! PDM pin configuration */
#define PDM_CLK_PIN         43
#define PDM_DATA_PIN        35

#define OPUS_COMPLEXITY 1
#define OPUS_VBR 1
#define STORAGE_SIZE_HEADER 10

/****************************************************************************/
/**                                                                        **/
/**                       TYPEDEFS AND STRUCTURES                          **/
/**                                                                        **/
/****************************************************************************/

/*! Foramt of the data to be stored in the flash */
typedef struct {
    int16_t header;
    int32_t rawtime_bin;
    int16_t time_ms_bin;
    uint8_t len;
    uint8_t index;
    uint8_t data[240];
} __attribute__((packed)) T5838_StorageFormat;

/****************************************************************************/
/**                                                                        **/
/**                          EXPORTED VARIABLES                            **/
/**                                                                        **/
/****************************************************************************/

#ifndef _T5838_C_SRC



#endif  /* _T5838_C_SRC */

/****************************************************************************/
/**                                                                        **/
/**                          EXPORTED FUNCTIONS                            **/
/**                                                                        **/
/****************************************************************************/

/**
 * @brief  Starts the T5838 driver.
 * @return 0 on success, -1 on failure.
 */
int t5838_start(void);

/**
 * @brief  Stops the T5838 driver and releases resources.
 * @return 0 on success, -1 on failure.
 */
int t5838_stop(void);

/**
 * @brief  Initializes the T5838 driver, setting up clocks and pins and initializes all required peripherals.
 * @return 0 on success, -1 on failure.
 */
int t5838_init(void);

/**
 * @brief  Starts continuous data saving from the T5838 driver in a separate thread.
 */
void t5838_start_saving(void);

/**
 * @brief  Stops continuous data saving from the T5838 driver.
 */
void t5838_stop_saving(void);

/****************************************************************************/
/**                                                                        **/
/**                          INLINE FUNCTIONS                              **/
/**                                                                        **/
/****************************************************************************/



#endif /* _T5838_H */
/****************************************************************************/
/**                                                                        **/
/**                                EOF                                     **/
/**                                                                        **/
/****************************************************************************/