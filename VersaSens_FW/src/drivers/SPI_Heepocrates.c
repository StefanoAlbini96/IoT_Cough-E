/*
                              *******************
******************************* C SOURCE FILE *******************************
**                            *******************                          **
**                                                                         **
** project  : VersaSens                                                    **
** filename : SPI_Heepocrates.c                                                   **
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
* @file   SPI_Heepocrates.c
* @date   DD/MM/YY
* @brief  This is the main header of SPI_Heepocrates.c
*
* Here typically goes a more extensive explanation of what the header
* defines.
*/

#define _SPI_HEEPOCRATES_C_SRC

/****************************************************************************/
/**                                                                        **/
/*                             MODULES USED                                 */
/**                                                                        **/
/****************************************************************************/

#include "SPI_Heepocrates.h"
#include <nrfx_spis.h>
#include <zephyr/logging/log.h>
#include <nrfx_gpiote.h>


/****************************************************************************/
/**                                                                        **/
/*                        DEFINITIONS AND MACROS                            */
/**                                                                        **/
/****************************************************************************/

/** @brief Symbol specifying message to be sent via SPIS data transfer. */
#define MSG_TO_SEND_SLAVE "test message"

/****************************************************************************/
/**                                                                        **/
/*                        TYPEDEFS AND STRUCTURES                           */
/**                                                                        **/
/****************************************************************************/

LOG_MODULE_REGISTER(SPI_Heepocrates, LOG_LEVEL_INF);

/****************************************************************************/
/**                                                                        **/
/*                      PROTOTYPES OF LOCAL FUNCTIONS                       */
/**                                                                        **/
/****************************************************************************/

/**
 * @brief Function for handling SPIS driver events.
 *
 * @param[in] p_event   Pointer to the SPIS driver event.
 * @param[in] p_context Pointer to the context passed from the driver.
 */
static void spis_handler(nrfx_spis_evt_t const * p_event, void * p_context);

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

/** @brief Transmit buffer initialized with the specified message ( @ref MSG_TO_SEND_SLAVE ). */
static uint8_t m_tx_buffer_slave[300] = MSG_TO_SEND_SLAVE;

/** @brief Receive buffer defined with the size to store specified message ( @ref MSG_TO_SEND_MASTER ). */
static uint8_t m_rx_buffer_slave[100] = MSG_TO_SEND_SLAVE;

nrfx_spis_t spis_inst = NRFX_SPIS_INSTANCE(SPIS_INST_IDX);

K_FIFO_DEFINE(heepo_fifo);

/** @brief Buffer for the next measurement to be sent */
static uint8_t heepo_next_meas[300];
uint8_t heepo_next_meas_size;

uint8_t heepo_fifo_counter = 0;

bool length_sent = false;

/****************************************************************************/
/**                                                                        **/
/*                           EXPORTED FUNCTIONS                             */
/**                                                                        **/
/****************************************************************************/

void SPI_Heepocrates_init(void)
{
    /* Assign pin 0 and 1 to the MCUAPP */
    nrf_gpio_pin_control_select(0, NRF_GPIO_PIN_SEL_APP);
    nrf_gpio_pin_control_select(1, NRF_GPIO_PIN_SEL_APP);

    nrfx_err_t status;
    (void)status;

    nrfx_spis_config_t spis_config = NRFX_SPIS_DEFAULT_CONFIG(SCK_PIN_SLAVE,
                                                              MOSI_PIN_SLAVE,
                                                              MISO_PIN_SLAVE,
                                                              CSN_PIN_SLAVE);
    spis_config.mode = NRF_SPIS_MODE_0;
    spis_config.miso_drive = NRF_GPIO_PIN_S0S1;

    nrfx_spis_uninit(&spis_inst);

    void * p_context = "Some context";
    status = nrfx_spis_init(&spis_inst, &spis_config, spis_handler, p_context);
    NRFX_ASSERT(status == NRFX_SUCCESS);

    IRQ_DIRECT_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_SPIS_INST_GET(SPIS_INST_IDX)), IRQ_PRIO_LOWEST,
                       NRFX_SPIS_INST_HANDLER_GET(SPIS_INST_IDX), 0);

    nrf_gpio_cfg_output(PIN_HEEPO_RDY);
    nrf_gpio_pin_set(PIN_HEEPO_RDY);

    SPI_Heepocrates_start(m_tx_buffer_slave, sizeof(m_tx_buffer_slave), m_rx_buffer_slave, sizeof(m_rx_buffer_slave));

    return;
}

/*****************************************************************************
*****************************************************************************/

void SPI_Heepocrates_start(uint8_t * p_tx_buffer, uint16_t length_tx, uint8_t * p_rx_buffer, uint16_t length_rx)
{
    nrfx_err_t status;
    (void)status;

    if (p_tx_buffer == NULL)
    {
        p_tx_buffer = m_tx_buffer_slave;
        length_tx = 0;
    }

    if (p_rx_buffer == NULL)
    {
        p_rx_buffer = m_rx_buffer_slave;
        length_rx = 0;
    }

    status = nrfx_spis_buffers_set(&spis_inst, p_tx_buffer, length_tx, p_rx_buffer, length_rx);
    NRFX_ASSERT(status == NRFX_SUCCESS);

    return;
}

// /*****************************************************************************
// *****************************************************************************/

void SPI_Heep_add_fifo(uint8_t *data, size_t size)
{
    if(heepo_fifo_counter >= 10)
    {
        return;
    }
    struct sensor_data_heepo *p_data = k_malloc(sizeof(*p_data));
    p_data->size = size;
    memcpy(p_data->data, data, size);
    k_fifo_put(&heepo_fifo, p_data);
    heepo_fifo_counter++;

    return;
}

// /*****************************************************************************
// *****************************************************************************/

void SPI_Heep_get_fifo()
{
    struct sensor_data_heepo *p_data = k_fifo_get(&heepo_fifo, K_NO_WAIT);
    if (p_data != NULL)
    {
        memcpy(heepo_next_meas, p_data->data, p_data->size);
        heepo_next_meas_size = p_data->size;
        k_free(p_data);
        heepo_fifo_counter--;
    }
    else
    {
        heepo_next_meas_size = 0;
    }

    return;
}

/****************************************************************************/
/**                                                                        **/
/*                            LOCAL FUNCTIONS                               */
/**                                                                        **/
/****************************************************************************/

static void spis_handler(nrfx_spis_evt_t const * p_event, void * p_context)
{
    if (p_event->evt_type == NRFX_SPIS_XFER_DONE)
    {
        nrf_gpio_pin_clear(PIN_HEEPO_RDY);

        // if (m_rx_buffer_slave[0] == 0x04)
        // {
        //     // LOG_INF("SPIS received 0x04");
        //     SPI_Heep_get_fifo();
        //     m_tx_buffer_slave[0] = heepo_next_meas_size;
        // }
        // else if (m_rx_buffer_slave[0] == 0x05)
        // {
        //     // LOG_INF("SPIS received 0x05");
        //     memcpy(m_tx_buffer_slave, heepo_next_meas, heepo_next_meas_size);
        // }
        

        if (length_sent == false)
        {
            SPI_Heep_get_fifo();
            m_tx_buffer_slave[0] = heepo_next_meas_size;
            if (heepo_next_meas_size != 0)
            {
                length_sent = true;
            }
        }
        else
        {
            memcpy(m_tx_buffer_slave, heepo_next_meas, heepo_next_meas_size);
            length_sent = false;
        }



        // char * p_msg = p_context;
        // LOG_INF("SPIS finished. Context passed to the handler: >%s<", p_msg);
        // LOG_INF("SPIS rx length: %d", p_event->rx_amount);
        // LOG_INF("SPIS rx buffer: %s", m_rx_buffer_slave);
        nrfx_spis_buffers_set(&spis_inst,
                              m_tx_buffer_slave, sizeof(m_tx_buffer_slave),
                              m_rx_buffer_slave, sizeof(m_rx_buffer_slave));
    }
    else if (p_event->evt_type == NRFX_SPIS_BUFFERS_SET_DONE)
    {
        nrf_gpio_pin_set(PIN_HEEPO_RDY);
    }
}

/****************************************************************************/
/**                                                                        **/
/*                                 EOF                                      */
/**                                                                        **/
/****************************************************************************/