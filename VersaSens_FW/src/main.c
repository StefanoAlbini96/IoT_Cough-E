#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/usb/usb_device.h>
#include <time.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <nrfx_gpiote.h>
#include "MAX77658.h"
#include "versa_ble.h"
#include "twim_inst.h"
#include "storage.h"
#include "ADS1298.h"
#include "MAX30001.h"
#include "versa_api.h"
#include <zephyr/devicetree.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/logging/log.h>
#include <zephyr/fs/fs.h>
#include <ff.h>
#include <SPI_Heepocrates.h>
#include <launcher.h>
#include "app_data.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);


int main(void)
{
    // nrf_gpio_cfg_output(START_PIN);
    // nrf_gpio_pin_set(START_PIN);
    
    versa_init();
    // enable_auto_connect();
    versa_config();

    versa_start_led_thread();
    versa_start_mode_thread();

    // Start the Cough-E thread for the application
    start_coughE_thread(); 

    while (1)
    {
        k_sleep(K_MSEC(500));
        // int8_t *test = k_malloc(1000 * sizeof(int8_t));
        // if(test != NULL){
        //     LOG_INF("POINTER %p", test);
        // } else {
        //     LOG_ERR("ERRO ALLOC");
        // }
        // k_free(test);
        // LOG_INF("HELLOW!\n");

        // // data aquisition example
        // k_sleep(K_MSEC(10));
        // struct app_data_struct *data = k_malloc(sizeof(*data));
        // if (data == NULL)
        // {
        //     LOG_ERR("Failed to allocate memory for new_data\n");
        // }
        // else
        // {
        //     app_data_get_from_fifo(data);
        // }
        
        // if (data != NULL)
        // {
        //     LOG_INF("Data received from FIFO: %02hx", data->data[0]);
        //     k_free(data);
        // }
    }
}
