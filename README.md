# dwm1001-keil-examples

*Note that the examples below consist in very basic application using the UWB features of the DWM1001C. These examples are not intended to be used in a commercial application and may not comply with regulation requirements.*

*Advanced firmware for DWM1001C that would comply with regulations can be found on https://www.decawave.com/product/dwm1001-module/*

## Overview

This project contains C simple examples for DWM1001 hardware and its derivatives, such as the DWM1001-DEV board.

The DWM1001 module is a Ultra Wideband (UWB) and Bluetooth hardware based on DecaWave's DW1000 IC and Nordic Semiconductor nrF52832 SoC. It allows to build a scalable Two-Way-Ranging (TWR) RTLS systems with up to thousands of tags. 

The DWM1001-DEV is a development board for the DWM1001 module. It contains an integrated Jlink to facilitate development and debbuging.
For more information about DWM1001, please visit www.decawave.com.

The C simple examples allow user to discover the key functionalities offered by the DWM1001 and UWB. These examples are customized for DWM1001-DEV, and some modifications will be necessary to port them to other DWM1001 based hardware (in particular LED and button interface)

The project is built as follow : 
```
dwm1001-keil-examples/
├── boards            // DWM1001-DEV board specific definitions
├── deca_driver       // DW1000 API software package 2.04 
├── examples          // C simple examples 
│   ├── ss_twr_init   // Single Sided Two Way Ranging Initiator example
│   ├── ss_twr_resp   // Single Sided Two Way Ranging Responder example
│   └── twi_accel     // LIS2DH12 accelerometer example with Two Wire interface 
├── nRF5_SDK_14.2.0   // Nordic Semiconductor SDK 14.2 for nrF52832
└── README.md
```
For more information about nrF52832 and nrF SDK, please visit http://infocenter.nordicsemi.com/

## Supported IDE

The examples are ready to use with the following IDE :
* Segger Embedded Studio (SES)
* Keil KEIL µVision

## Segger Embedded Studio

Each example contains a emproject project file for SES. The examples compile and load cleanly to the DWM1001.   
The project was created with the SES version V3.34a. 

SES has a free license for nrF52832 development. Consequently, this IDE can be used without any limitation for DWM1001 development.

For more information regarding Segger Embedded Studio, please visit https://www.segger.com/products/development-tools/embedded-studio/

For more information about free license for nrF52832, please read https://www.nordicsemi.com/News/News-releases/Product-Related-News/Nordic-Semiconductor-adds-Embedded-Studio-IDE-support-for-nRF51-and-nRF52-SoC-development

### SES : Additional Package

When using SES IDE, you will need to install the following package :

Package                                                                                                                           
CMSIS 5 CMSIS-CORE Support Package (version 5.02)                                                                           
CMSIS-CORE Support Package (version 4.05)                                                                           
Nordic Semiconductor nRF CPU Support Package (version 1.06)                                                                           

They can be install from SES itself, through the package manager in the tools menu. 

## KEIL µVision IDE

Each example contains a µVision5 project file for Keil µVision IDE. The examples compile and load cleanly to the DWM1001.
The project was created with the KEIL uVision version V5.24.2.0. 

Keil µVision has a free license for project up to 32KB. For more information regarding Keil µVision, please visit http://www2.keil.com/mdk5/uvision/

### µVision Error: Flash Download failed - "Cortex-M4"

This error can be observed if there is a memory conflict between the binary to load and the current firmware on the target hardware. This issue can be easily fixed by fully erasing the target device 's flash memory. Keil µVision cannot perform a full erase and the following free tool can be used :

* J-flash lite 
* nrfjprog command line script

For more information about the issue, please see :

https://devzone.nordicsemi.com/f/nordic-q-a/18278/error-flash-download-failed---cortex---m4-while-flashing-softdevice-from-keil-uvision-5

## Example Details 

The SS-TWR scheme can be implemented using two DWM1001 modules, the first one programmed as an initiator and the second one as a responder.

### Single Sided Two Way Ranging -- Initiator

This example contains the source code for the initiator. The initiator will send a frame, wait for the response from the receiver, calculate the distance and output it on the UART (can be observed on a serial terminal)

```
dwm1001-keil-examples/examples/ss_twr_init/
├── config                    // Contains sdk_config.h file for nrF SDK 14.2 customization
├── main.c                    // Initialization and main program
├── ss_init_main.c            // Single sided initiator core program
├── UART                      // Uart 
├── SES
│   └── ss_twr_init.emProject // Segger Embedded Studio project
└── Keil uvision
     └── ss_twr_init.uvprojx  // Keil uvision project

```
The application function is detailed in the main.c and the ss_init_main.c files. 

Calibration may be necessary in order to have an accurate measurement. It can be done by adjusting the antenna delay which is hardware dependent. 

### Single Sided Two Way Ranging -- Responder

This example contains the source code for the responder. The receiver will receive a frame from the initiator and send the corresponding answer.

```
dwm1001-keil-examples/examples/ss_twr_resp/
├── config                    // Contains sdk_config.h file for nrF SDK 14.2 customization
├── main.c                    // Initialization and main program
├── ss_resp_main.c            // Single sided responder core program
├── SES
│   └── ss_twr_resp.emProject // Segger Embedded Studio project
└── Keil uvision
     └── ss_twr_resp.uvprojx  // Keil uvision project
```
The application function is detailed in the main.c and the ss_resp_main.c files. 

Calibration may be necessary in order to have an accurate measurement. It can be done by adjusting the antenna delay which is hardware dependent. 

### Two Wire Interface Accelerometer

This example implements a TWI between the LIS2DH12 and the nrF52832. 
The blue led will be on when a motion is detected by the LIS2DH12. It also reports the accelerometer state on the UART.

```
dwm1001-keil-examples/examples/twi_accel/
├── config                    // Contains sdk_config.h file for nrF SDK 14.2 customization
├── LIS2DH12                  // LIS2DH12 (accelerometer) low level driver and api
├── main.c                    // Initialization and main program
├── TWI                       // TWI
├── UART                      // Uart
├── SES
│   └── twi_accel.emProject // Segger Embedded Studio project
└── Keil uvision
     └── twi_accel.uvprojx  // Keil uvision project
```
The application function is detailed in the main.c file.





