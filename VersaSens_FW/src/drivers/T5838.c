/*
                              *******************
******************************* C SOURCE FILE *******************************
**                            *******************                          **
**                                                                         **
** project  : VersaSens                                                    **
** filename : T5838.c                                                      **
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
* @file   T5838.c
* @date   DD/MM/YY
* @brief  This is the main header of T5838.c
*
* Here typically goes a more extensive explanation of what the header
* defines.
*/

#define _T5838_C_SRC

/****************************************************************************/
/**                                                                        **/
/*                             MODULES USED                                 */
/**                                                                        **/
/****************************************************************************/

#include "storage.h"
#include <nrfx_uarte.h>
#include <stdlib.h>
#include "T5838.h"
#include <nrfx_pdm.h>
#include <nrfx_clock.h>
#include <zephyr/irq.h>
#include <zephyr/types.h>
#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "versa_time.h"
#include "versa_config.h"
#include "SPI_Heepocrates.h"

#include "opus.h"
#include "opus_types.h"
#include "opus_private.h"

/****************************************************************************/
/**                                                                        **/
/*                        DEFINITIONS AND MACROS                            */
/**                                                                        **/
/****************************************************************************/

LOG_MODULE_REGISTER(t5838, LOG_LEVEL_INF);

#define PDM_BUFFER_SIZE        480    /*!< Size of the buffer for PDM frames */

/****************************************************************************/
/**                                                                        **/
/*                        TYPEDEFS AND STRUCTURES                           */
/**                                                                        **/
/****************************************************************************/

/*! Buffer structure to hold PDM frames */
struct {
    int16_t frame1[PDM_BUFFER_SIZE];
    int16_t frame2[PDM_BUFFER_SIZE];
    int16_t frame3[PDM_BUFFER_SIZE];
} t5838_frames;

/****************************************************************************/
/**                                                                        **/
/*                      PROTOTYPES OF LOCAL FUNCTIONS                       */
/**                                                                        **/
/****************************************************************************/

/**
* @brief This function is the thread function that saves the frames to the fifo buffer
*
* @param arg1 : argument 1, not used
* @param arg2 : argument 2, not used
* @param arg3 : argument 3, not used
*
* @retval None
*/
void t5838_save_thread_func(void *arg1, void *arg2, void *arg3);

/****************************************************************************/
/**                                                                        **/
/*                           EXPORTED VARIABLES                             */
/**                                                                        **/
/****************************************************************************/

/*! PDM frame active buffer */
int16_t *t5838_frame = NULL;

/*! Flags to track new PDM frame availability */
bool frame1_new = false;
bool frame2_new = false;
bool frame3_new = false;

/****************************************************************************/
/**                                                                        **/
/*                            GLOBAL VARIABLES                              */
/**                                                                        **/
/****************************************************************************/

/*! Stack and thread data for saving PDM frames */
K_THREAD_STACK_DEFINE(save_thread_stack, 100000);
struct k_thread save_thread;
bool save_thread_stop = false;

/*! Structure to store encoded audio data */
T5838_StorageFormat storage_format;

/****************************************************************************/
/**                                                                        **/
/*                           EXPORTED FUNCTIONS                             */
/**                                                                        **/
/****************************************************************************/

static void pdm_handler(nrfx_pdm_evt_t const *event)
{
    if (event->buffer_requested) {
        /* Rotate buffers as needed */
        if (t5838_frame == t5838_frames.frame1) {
            t5838_frame = t5838_frames.frame2;
        } else if (t5838_frame == t5838_frames.frame2) {
            t5838_frame = t5838_frames.frame3;
        } else {
            t5838_frame = t5838_frames.frame1;
        }

        /* Set flags for newly released buffers */
        if (event->buffer_released == t5838_frames.frame1) {
            frame1_new = true;
        } else if (event->buffer_released == t5838_frames.frame2) {
            frame2_new = true;
        } else if (event->buffer_released == t5838_frames.frame3) {
            frame3_new = true;
        }

        nrfx_pdm_buffer_set(t5838_frame, PDM_BUFFER_SIZE);
    }
}

/*****************************************************************************
*****************************************************************************/

int t5838_start(void)
{
    nrfx_err_t status;
    (void)status; 

    /*! Start the PDM peripheral */
    status = nrfx_pdm_start();

    t5838_start_saving();

    return (status == NRFX_SUCCESS)? 0 : -1;
}

/*****************************************************************************
*****************************************************************************/

int t5838_stop(void)
{
    nrfx_err_t status;
    (void)status; 

    t5838_stop_saving();

    /*! Stop the PDM peripheral */
    status = nrfx_pdm_stop();

    return (status == NRFX_SUCCESS)? 0 : -1;
}

/*****************************************************************************
*****************************************************************************/

int t5838_init(void)
{
    nrfx_err_t status;
    (void)status; 

    /*! f_out = 12288000 */
    /*! FREQ_VALUE = 2^16 * ((12 * f_out / 32M) - 4) */
    uint16_t freq_value = 39846;

    /*! Set the audio clock frequency */
    nrfx_clock_enable();
    nrfx_clock_start(NRF_CLOCK_DOMAIN_HFCLKAUDIO);
    nrfx_clock_hfclkaudio_config_set(freq_value);

    /*! Set the PDM configuration */
    nrfx_pdm_config_t pdm_config = NRFX_PDM_DEFAULT_CONFIG(PDM_CLK_PIN, PDM_DATA_PIN);
    pdm_config.mode = NRF_PDM_MODE_MONO;
    pdm_config.edge = NRF_PDM_EDGE_LEFTFALLING;
    // pdm_config.clock_freq = NRF_PDM_FREQ_1000K * 0.768;
    pdm_config.clock_freq = 260300800;
    pdm_config.ratio = NRF_PDM_RATIO_64X;
    pdm_config.mclksrc = NRF_PDM_MCLKSRC_ACLK;

    /*! Initialize the PDM peripheral */
    status = nrfx_pdm_init(&pdm_config, pdm_handler);

    IRQ_DIRECT_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_PDM0), IRQ_PRIO_LOWEST, nrfx_pdm_irq_handler, 0)

    return (status == NRFX_SUCCESS)? 0 : -1;
}

/****************************************************************************/
/****************************************************************************/

void t5838_start_saving(void)
{
    save_thread_stop = false;
    k_thread_create(&save_thread, save_thread_stack, K_THREAD_STACK_SIZEOF(save_thread_stack),
                    t5838_save_thread_func, NULL, NULL, NULL, T5838_PRIO, 0, K_NO_WAIT);
    k_thread_name_set(&save_thread, "Save_Thread");
}

/****************************************************************************/
/****************************************************************************/

void t5838_stop_saving(void)
{
    save_thread_stop = true;
}

/****************************************************************************/
/**                                                                        **/
/*                            LOCAL FUNCTIONS                               */
/**                                                                        **/
/****************************************************************************/

void t5838_save_thread_func(void *arg1, void *arg2, void *arg3)
{
    /* Initialize Opus Encoder */
    int error;
    OpusEncoder *encoder = opus_encoder_create(12000, 1, OPUS_APPLICATION_AUDIO, &error);
    if (error != OPUS_OK) {
        LOG_ERR("Opus encoder initialization error: %s\n", opus_strerror(error));
        return;
    }
    LOG_INF("Opus encoder initialized\n");

    /* Configure Opus Encoder */
    error = opus_encoder_ctl(encoder, OPUS_SET_COMPLEXITY(OPUS_COMPLEXITY), OPUS_SET_VBR(OPUS_VBR));
    if (error != OPUS_OK) {
        LOG_ERR("Opus encoder configuration error: %s\n", opus_strerror(error));
        return;
    }
    LOG_INF("Opus encoder configured\n");

    int16_t compressed_frame[PDM_BUFFER_SIZE] = {0};  /*!< Compressed output buffer */
    uint8_t frame_index = 0;                           /*!< Frame sequence index */
    storage_format.header = 0xAAAA;                    /*!< Storage header marker */

    bool *frame_flags[] = {&frame1_new, &frame2_new, &frame3_new};  /*!< Pointers to frame flags */
    int16_t *frames[] = {t5838_frames.frame1, t5838_frames.frame2, t5838_frames.frame3};  /*!< Frame pointers */

    /* Main loop to handle new frames */
    while (!save_thread_stop) {
        struct time_values current_time = get_time_values();
        storage_format.rawtime_bin = current_time.rawtime_s_bin;
        storage_format.time_ms_bin = current_time.time_ms_bin;

        for (int i = 0; i < 3; i++) {
            if (*frame_flags[i]) {
                *frame_flags[i] = false;

                /* Encode frame using Opus */
                int start_time = k_uptime_get();
                uint8_t len = (uint8_t)opus_encode(encoder, frames[i], PDM_BUFFER_SIZE, compressed_frame, PDM_BUFFER_SIZE) * sizeof(int16_t);
                int end_time = k_uptime_get();
                printk("Encoding time: %d ms\n", end_time - start_time);

                /* Store encoded data in storage format structure */
                storage_format.len = len + 1;
                storage_format.index = frame_index++;
                memcpy(storage_format.data, compressed_frame, len);

                /* Store and add data to buffers */
                int ret = storage_add_to_fifo((uint8_t *)&storage_format, len + STORAGE_SIZE_HEADER);
                if (ret != 0) {
                    LOG_ERR("Error writing to flash\n");
                }

                /* Add to BLE and optional SPI Heep if configured */
                ble_add_to_fifo((uint8_t *)&storage_format, len + STORAGE_SIZE_HEADER);
                if (VCONF_T5838_HEEPO) {
                    SPI_Heep_add_fifo((uint8_t *)&storage_format, len + STORAGE_SIZE_HEADER);
                }
                if(VCONF_T5838_APPDATA) {
                    app_data_add_to_fifo((uint8_t *)&storage_format, len + STORAGE_SIZE_HEADER);
                }

                printk("Processed Frame %d, Length: %d bytes\n", i + 1, len);
            }
        }

        k_sleep(K_MSEC(10));  /*!< Sleep briefly before checking for new frames */
    }

    opus_encoder_destroy(encoder);
    k_thread_abort(k_current_get());
}

/****************************************************************************/
/**                                                                        **/
/*                                 EOF                                      */
/**                                                                        **/
/****************************************************************************/