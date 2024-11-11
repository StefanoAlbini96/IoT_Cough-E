# VersaSens_FW

## Firmeware Description

## Prerequisites

1. **NRF Connect Toolchain (v2.5.0) and SDKs (v2.5.0)**

   - **Windows Installation:**
     - Download and install the NRF-Connect for Desktop application from the [official Nordic Semiconductor website](https://www.nordicsemi.com/Products/Development-tools/nRF-Connect-for-Desktop).
     - Follow the instructions provided in the application to install the required toolchain and SDKs.

2. **NRF Command Line Tools**
   - Download and install the NRF Command Line Tools from the [official Nordic Semiconductor website](https://www.nordicsemi.com/Products/Development-tools/nRF-Command-Line-Tools).

3. **NRF Connect for VS Code**
   - Install the NRF Connect extension for Visual Studio Code using the Toolchain Manager found in nRF Connect for Desktop.

## Building and Flashing the Firmware

1. **Build Configuration**
   - Open the project in Visual Studio Code.
   - Use the NRF Connect extension to configure the build with the target set as `nrf5340dk_nrf5340_cpuapp`.
2. **Build**
   - Using the interface provided by NRF Connect for VS Code, select the **Build** action to compile the project.

3. **Flash**
   - Connect the programmer to your computer.
   - Using the interface provided by NRF Connect for VS Code, select the **Flash** action to upload the firmware to the device.