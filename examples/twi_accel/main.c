/*!
* @brief Component name:	MAIN
*
* Entry code for TWI/Accelerometer Driver.
* Initialise and configure the TWI, LIS2DH12 and UART.
* DWM1001 drivers for use with DEV board.
* Uses a re-directed STDIO for printf, through UART.
*
* @file main.c
*/

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "nordic_common.h"
#include "nrf_pwr_mgmt.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "app_timer.h"
#include "TWI.h"
#include "LIS2DH12.h"
#include "UART.h"
#include "bsp.h"
#include "boards.h"

// Select one of the following test modes:
#define TEST_MODE_MOTION_DETECT
//#define	TEST_MODE_FIFO

// Local function definitions
#ifdef TEST_MODE_FIFO
static void	vTestModeFifo(void);
#endif
#ifdef TEST_MODE_MOTION_DETECT
static void vTestModeMotionDetect(void);
#endif

// Symbolic constants
#define	GREEN_LED	BSP_BOARD_LED_0
#define	BLUE_LED	BSP_BOARD_LED_1

/*!
* @brief Enter point for accelerometer application.
*
* Initialise and configured the TWI, UART, LIS2DH12, LEDs.
* Configure the LIS2DH12 into a test mode.
* Loop running LIS2DH12 test mode.
*/
int main(void)
{
	// Optional: enable power management monitoring.
	// Useful if you want to designate an I/O pin
	// to indicate when the MCU is in low-power mode.
	// nrf_pwr_mgmt_init();
	
	// LED management - green LED on indicates MCU active
	bsp_board_leds_init();
	bsp_board_led_on(GREEN_LED);
	
	// UART for simple STDIO, ignore initialisation errors
	(void) boUART_Init();
	
	// TWI interface to accelerometer
	vTWI_Init();
	
	// Initialise the accelerometer
	// Note: this function blocks for 20ms.
	vLIS2_Init();
	
	// printf appears from nRF52 UART on RPi interface pins
	printf("Accel test running\n");
	
	// Check the TWI and acceleromter are talking
	uint8_t u8ID = u8LIS2_TestRead();
	printf("LIS2DH12 - Who am I code:%x\n", u8ID);
	
#ifdef TEST_MODE_FIFO
	// Set LIS2DH12 into FIFO mode
	vTestModeFifo();
#endif
	
#ifdef TEST_MODE_MOTION_DETECT
	// test mode is motion dection
	vTestModeMotionDetect();
#endif

#if (!defined TEST_MODE_FIFO && !defined TEST_MODE_MOTION_DETECT)
#warning "No test mode set"
	while (true)
		;
#endif
}

// Local functions

#ifdef TEST_MODE_FIFO
/*!
* @brief Run the accelerometer FIFO test mode.
*
* Set the accelerometer into continuous 10Hz 12-bit sampling,
* with samples stored in internal FIFO.  Interrupt when FIFO
* is almost full.
* This function loops continuously calling the accelerometer
* foreground task.
*/
static void	vTestModeFifo(void)
{
	vLIS2_EnableFifoSampling();
	
	// Finished processing, turn green LED off
	bsp_board_led_off(GREEN_LED);

	// Loop in test mode, waiting for FIFO full events
	while (true)
	{
		// Blue LED toggles between sampling periods.
		// Accelerometer task toggles the LED.
		vLIS2_Task();
		
		// Option: could put MCU into low-power mode until
		// next event (fifo full) occurs, but simple UART
		// driver output stream might be truncated.
		//__WFE();
	}
}
#endif

#ifdef TEST_MODE_MOTION_DETECT
/*!
* @brief Run the accelerometer motion detection test mode.
*
* Initialise the accelerometer to detect motion and generate
* an interrupt.  Call the accelerometer foreground task, waiting
* for the interrupt to occur.
* On detecting motion turn on the blue LED, after a period of
* inactivity, turn off the LED and wait for another motion event.
*/
static void vTestModeMotionDetect(void)
{
	// Set accelerometer activity detection level
	vLIS2_EnableWakeUpDetect();
	
	// Loop, driving the GREEN LED (D9),
	// LED is ON when motion detected then
	// turns OFF after a period of no motion.	
	while (true)
	{
		// Wait for an accelerometer trigger event
		if (boLIS2_InterruptOccurred())
		{
			uint8_t u8Status = u8LIS2_EventStatus();
			printf("Accel event code: 0x%x\n", u8Status);
			
			if (u8Status & (XHIE | YHIE | ZHIE))
			{
				// Any combination of X,Y,Z motion passing
				// high threshold generates an interrupt
				bsp_board_led_on(BLUE_LED);
				vLIS2_EnableInactivityDetect();
			}
			else // Assume low threshold (plus delay) detected
			{
				bsp_board_led_off(BLUE_LED);
				vLIS2_EnableWakeUpDetect();
			}
		}
		else
		{
			// Put MCU into low-power mode until event occurs.
			// Note, no check of UART fifo is possible, so this might
			// truncate serial output waiting in the fifo.
			bsp_board_led_off(GREEN_LED);
			__WFE();
			bsp_board_led_on(GREEN_LED);
		}
	}
}
#endif // TEST_MODE_MOTION_DETECT
