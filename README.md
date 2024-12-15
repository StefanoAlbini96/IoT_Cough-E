# IoT_Cough-E


This repository contains the VersaSens Firmware for running the Cough-E application in real-time.
The configuration is set to use only the audio sensor (T5838) and send windows of data via BLE.

The core modules of the repository are:

- `src/drivers/T5838.c`: the driver for the audio sensor. In this module, the `t5838_save_thread_func()` thread  collects the samples and sends them into a FIFO, for the other threads to used them.
- `src/drivers/app_data.c`: the code for managing the data FIFO. This module creates the FIFO and provides two functions to add and get data from it: `add_t5838_data_to_fifo()`, and `get_t5838_data_from_fifo()` respectively.
- `src/cough_E_app`: this folder contains the soruce code and headers files of the Cough-E application. In particular, the `code/launcher.c` module contains the `data_thread()` and `iot_cough_E()` threads. The first one periodically collects samples from the FIFO. Once a full window is available it utilises a semaphore to notify the second thread, which processes the window, providing an esimated number of cough events. After a predefined amount of time, it sends the estimation, together with a window of data, to the BLE module.
- `src/drivers/versa_ble.c`: the driver for the BLE communication. It provides the `send_aud_wind_ble()` function, called by the `iot_cough_E` thread, to send result and window via BLE.
