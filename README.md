# dwm1001-keil-examples
## Overview

This project contains C simple examples for DWM1001 hardware and its derivatives.

The DWM1001 module is a Ultra Wide Band (UWB) and Bluetooth hardware based on DecaWave's DW1000 IC and Nordic Semiconductor nrF52832 SoC. It allows to build a scalable Two-Way-Ranging (TWR) RTLS systems with up to thousands of tags. For more information about DWM1001, please visit www.decawave.com.

The C simple examples allow user to discover the key functionalities offered by the DWM1001 and UWB. 

The project is built as follow : 
```
dwm1001-keil-examples/
├── boards            // DWM1001 board specific definitions
├── deca_driver       // DW1000 API software package 2.04 
├── examples          // C simple examples 
│   ├── ss_twr_init   // Single Sided Two Way Ranging Initiator example
│   ├── ss_twr_resp   // Single Sided Two Way Ranging Responder example
│   └── twi_accel     // LIS2DH12 accelerometer example with Two Wire interface 
├── nRF5_SDK_14.2.0   // Nordic Semiconductor SDK 14.2 for nrF52832
└── README.md
```
For more information about nrF52832 and nrF SDK, please visit http://infocenter.nordicsemi.com/


## KEIL Integrated Development Environment (IDE) 

Each example contains a uVision5 project file for Keil IDE setting up the environment. The example compile and load cleanly to the DWM1001. 

Keil IDE has a free license for project up to 32KB. For more information regarding Keil, please visit http://www2.keil.com/mdk5/

## Error: Flash Download failed - "Cortex-M4"

This error can be observed if there is a memory conflict between the code to load and the current code on the target hardware. This issue can be easily fixed performing a full erase of the target flash, using J-flash lite or the nrfjprog command line script.

## Example Details 

### Single Sided Two Way Ranging Initiator

This example contains the source code for the initiator in a SS-TWR scheme. The initiator will send a frame, wait for the response from the receiver, calculate the distance and output it on the UART (can be osbserved on a serial terminal)

```
dwm1001-keil-examples/examples/ss_twr_init/
├── config                    // Contains sdk_config.h file for nrF SDK 14.2 customization
├── EventRecorderStub.scvd
├── JLinkLog.txt
├── JLinkSettings.ini
├── main.c                    // Initialization and main program
├── RTE
├── ss_init_main.c            // Single sided initiator core program
├── ss_twr_init.uvoptx
├── ss_twr_init.uvprojx       // Keil project
└── UART                      // Uart 
```
The application functionnement is detailed in the main.c and the ss_init_main.c files. 

Calibration may be necessary in order to have an accurate measurement. It can be done by adjusting the antenna delay which is hardware dependent. 



#### Single Sided Two Way Ranging Responder

This example contains the source code for the responder in a SS-TWR scheme. The receiver will receive a frame from the initiator and send the corresponding answer.

```
dwm1001-keil-examples/examples/ss_twr_resp/
├── config                    // Contains sdk_config.h file for nrF SDK 14.2 customization
├── EventRecorderStub.scvd
├── JLinkLog.txt
├── JLinkSettings.ini
├── main.c                    // Initialization and main program
├── RTE
├── ss_resp_main.c            // Single sided responder core program
├── ss_twr_resp.uvoptx
└── ss_twr_resp.uvprojx       // Keil project
```
The application functionnement is detailed in the main.c and the ss_resp_main.c files. 

Calibration may be necessary in order to have an accurate measurement. It can be done by adjusting the antenna delay which is hardware dependent. 

#### Two Wire Interface Accelerometer

This example contains the source code for the TWI-Accelerometer example. With this application, the blue led will be on when a motion is detected by the LIS2DH12. It also reports the accelerometer state on the UART.

```
dwm1001-keil-examples/examples/twi_accel/
├── config                    // Contains sdk_config.h file for nrF SDK 14.2 customization
├── EventRecorderStub.scvd
├── JLinkLog.txt
├── JLinkSettings.ini
├── LIS2DH12                  // LIS2DH12 (accelerometer) low level driver and api
├── main.c                    // Initialization and main program
├── RTE
├── TWI                       // TWI
├── twi_accel.uvoptx
├── twi_accel.uvprojx         // Keil Project
└── UART                      // Uart
```
The application functionnement is detailed in the main.c file.





