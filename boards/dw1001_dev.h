/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */
#ifndef DW1001_DEV_H
#define DW1001_DEV_H

#define DW1000_RST    24
#define DW1000_IRQ    19

// LEDs definitions for DW1001_DEV_H
#define LEDS_NUMBER    4

// 4 Leds for user application with DWM1001-dev
#define LED_0          30 // Green
#define LED_1          31 // Blue
#define LED_2          22 // Red
#define LED_3          14 // Red

#define LEDS_LIST { LED_0, LED_1, LED_2, LED_3}

#define BSP_LED_0      LED_0
#define BSP_LED_1      LED_1
#define BSP_LED_2      LED_2
#define BSP_LED_3      LED_3

#define LEDS_ACTIVE_STATE 0

#define LEDS_INV_MASK  LEDS_MASK

#define BUTTONS_NUMBER 4

#define BUTTON_START   13
#define BUTTON_1       13
#define BUTTON_2       14
#define BUTTON_3       15
#define BUTTON_4       16
#define BUTTON_STOP    16
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

#define BUTTONS_LIST { BUTTON_1, BUTTON_2, BUTTON_3, BUTTON_4 }

#define BSP_BUTTON_0   BUTTON_1
#define BSP_BUTTON_1   BUTTON_2
#define BSP_BUTTON_2   BUTTON_3
#define BSP_BUTTON_3   BUTTON_4

#define BUTTONS_ACTIVE_STATE 0

#define BSP_BUTTON_0_MASK (1<<BSP_BUTTON_0)
#define BSP_BUTTON_1_MASK (1<<BSP_BUTTON_1)
#define BSP_BUTTON_2_MASK (1<<BSP_BUTTON_2)
#define BSP_BUTTON_3_MASK (1<<BSP_BUTTON_3)


#define RX_PIN_NUMBER  11
#define TX_PIN_NUMBER  5
#define CTS_PIN_NUMBER 7
#define RTS_PIN_NUMBER 6

#define HWFC           true	

#define SPIS_MISO_PIN   28  // SPI MISO signal.
#define SPIS_CSN_PIN    12  // SPI CSN signal.
#define SPIS_MOSI_PIN   25  // SPI MOSI signal.
#define SPIS_SCK_PIN    29  // SPI SCK signal.

#define SPIM0_SCK_PIN   2   // SPI clock GPIO pin number.
#define SPIM0_MOSI_PIN  3   // SPI Master Out Slave In GPIO pin number.
#define SPIM0_MISO_PIN  4   // SPI Master In Slave Out GPIO pin number.
#define SPIM0_SS_PIN    5   // SPI Slave Select GPIO pin number.

#define SPIM1_SCK_PIN   16  // DWM1001 SPIM1 sck connected to DW1000
#define SPIM1_MOSI_PIN  20  // DWM1001 SPIM1 mosi connected to DW1000
#define SPIM1_MISO_PIN  18  // DWM1001 SPIM1 miso connected to DW1000
#define SPIM1_IRQ_PRIORITY APP_IRQ_PRIORITY_LOW // 
#define SPIM1_SS_PIN    XX  //  Not used with DMW1001

#define SPIM2_SCK_PIN   12  // SPI clock GPIO pin number.
#define SPIM2_MOSI_PIN  13  // SPI Master Out Slave In GPIO pin number.
#define SPIM2_MISO_PIN  14  // SPI Master In Slave Out GPIO pin number.
#define SPIM2_SS_PIN    15  // SPI Slave Select GPIO pin number.

// UART symbolic constants
// NOTE - using pins from the RPI interface connector
// 				NOT compatible with RPi Gateway builds
#define TX_PIN_NUM			5		// DWM1001 module pin 20, DEV board name RXD
#define RX_PIN_NUM			11		// DWM1001 module pin 18, DEV board name TXD
#define RTS_PIN_NUM			UART_PIN_DISCONNECTED
#define CTS_PIN_NUM			UART_PIN_DISCONNECTED

// serialization APPLICATION board - temp. setup for running serialized MEMU tests
#define SER_APP_RX_PIN              23    // UART RX pin number.
#define SER_APP_TX_PIN              24    // UART TX pin number.
#define SER_APP_CTS_PIN             2     // UART Clear To Send pin number.
#define SER_APP_RTS_PIN             25    // UART Request To Send pin number.

#define SER_APP_SPIM0_SCK_PIN       27     // SPI clock GPIO pin number.
#define SER_APP_SPIM0_MOSI_PIN      2      // SPI Master Out Slave In GPIO pin number
#define SER_APP_SPIM0_MISO_PIN      26     // SPI Master In Slave Out GPIO pin number
#define SER_APP_SPIM0_SS_PIN        23     // SPI Slave Select GPIO pin number
#define SER_APP_SPIM0_RDY_PIN       25     // SPI READY GPIO pin number
#define SER_APP_SPIM0_REQ_PIN       24     // SPI REQUEST GPIO pin number

// serialization CONNECTIVITY board
#define SER_CON_RX_PIN              24    // UART RX pin number.
#define SER_CON_TX_PIN              23    // UART TX pin number.
#define SER_CON_CTS_PIN             25    // UART Clear To Send pin number. Not used if HWFC is set to false.
#define SER_CON_RTS_PIN             2     // UART Request To Send pin number. Not used if HWFC is set to false.


#define SER_CON_SPIS_SCK_PIN        27    // SPI SCK signal.
#define SER_CON_SPIS_MOSI_PIN       2     // SPI MOSI signal.
#define SER_CON_SPIS_MISO_PIN       26    // SPI MISO signal.
#define SER_CON_SPIS_CSN_PIN        23    // SPI CSN signal.
#define SER_CON_SPIS_RDY_PIN        25    // SPI READY GPIO pin number.
#define SER_CON_SPIS_REQ_PIN        24    // SPI REQUEST GPIO pin number.

#define SER_CONN_CHIP_RESET_PIN     11    // Pin used to reset connectivity chip

// Low frequency clock source to be used by the SoftDevice
#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_XTAL,            \
                                 .rc_ctiv       = 0,                                \
                                 .rc_temp_ctiv  = 0,                                \
                                 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}

#endif // PCA10040_H
