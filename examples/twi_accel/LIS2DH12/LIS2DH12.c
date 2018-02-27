/*
*	Component name:	LIS2DH12
*
*	File name: LIS2DH12.c
*
* Description:
* Device driver for LIS2DH12 accelerometer.
* Provides a public interface for control of the LIS2DH12.
*/

#include <stdio.h>
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_gpiote.h"
#include "nrf_delay.h"
#include "TWI.h"
#include "LIS2DH12.h"
#include "LIS2DH12registers.h"
#include "bsp.h"
#include "boards.h"

// Local variables and constants

// Blue LED toggles between FIFO sample intervals
#define	BLUE_LED	BSP_BOARD_LED_1
#define	GREEN_LED	BSP_BOARD_LED_0

// FIFO element
struct sFifoXYZelement
{
	uint16_t u16_X;
	uint16_t u16_Y;
	uint16_t u16_Z;
};

typedef struct sFifoXYZelement tsFifoElement;

// Storage for LIS2 FIFO samples
static tsFifoElement atsFifo[MAX_FIFO_SIZE];

// Seamphores for events
static volatile bool boInterruptEvent = false;

// Accelerometer mode - convert from left justified, signed
// by shifting readings
static uint8_t u8SignExtend;

// 8, 10, 12 bit resolution
enum AccelRES
{
	mode_8bit,
	mode_10bit,
	mode_12bit
};

// Currently selected resolution
enum AccelRES eRsolution;

// Local function declarations
static void vInterruptInit			(void);
static void vInterruptHandler		(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
static void vThresholdConfigure	(uint8_t u8Level, uint8_t u8Duration, uint8_t u8Mode);
static void vDumpFifo						(uint8_t u8Num);


// Public interface functions

/*!
* @brief Initialise the LIS2DH12 Accelerometer
*
* Initialises the LIS2DH12 accelerometer into known state.
* After an MCU restart or Power-On-Reset, LIS2 may still be running,
* so first force a reboot to initialise registers and place device
* into power-down mode.
* Allow ample time for the boot process to complete - datasheet says
* reboot takes 'approximately' 5ms.
* Next, place device in low-power mode, high resolution, 1Hz sampling
* on a single axis.
*/
void vLIS2_Init (void)
{
	// If this function is called after power on, then the LIS2DH12
	// will still be in boot mode, allow time for boot to complete. 
	nrf_delay_ms(20);
	
	// Disable all interrupt sources
	vTWI_Write(INT1_CFG,  0);

	// Clear any pending interrupts
	uint8_t u8Reg;
	vTWI_Read(INT1_SRC, &u8Reg);	

	// Put device into power-down
	vLIS2_PowerDown();
	nrf_delay_ms(10);
	
	// Enable MCU interrupts from INT1 pin
	vInterruptInit();
}

/*!
* @brief Test TWI connection to LIS2DH12 accelerometer
*
* Read and return the LIS2DH12's WHO_AM_I register.
*/
uint8_t u8LIS2_TestRead(void)
{
	uint8_t u8whoAmI;
	vTWI_Read (WHO_AM_I, &u8whoAmI);
	
	return u8whoAmI;
}

/*!
* @brief Places LIS2DH12 Accelerometer into low-power standby
*/
void vLIS2_PowerDown (void)
{
	// Set the ODR value to put device into power-down mode.
	vTWI_Write(CTRL_REG1, ODR_PDOWN);
}

/*!
* @brief Enable wake-up detection.
*
* Configured to generate an interupt if X,Y,Z sensors
* detect instantaneous dynamic acceleration greater than
* +/- 32mg over a 2g range sampled at 10Hz.
* High-Pass filter is enable to remove static (gravitational)
* acceleration.
*/
void vLIS2_EnableWakeUpDetect (void)
{
	// Configure threshold detector for activity wake-up event
	vThresholdConfigure (THS_FS2G_32mg, 0, (XHIE | YHIE | ZHIE));
}

/*!
* @brief Enable prolonged inactivity detection.
*
* Configured to generate an interupt if X,Y,Z sensors
* detect enduring dynamic acceleration less than
* +/- 32mg over a 2g range for a period of 3 seconds sampling
* at 10Hz.
* High-Pass filter is enable to remove static (gravitational)
* acceleration.
*/
void vLIS2_EnableInactivityDetect (void)
{
	// Configure threshold detector for inactivity event
	vThresholdConfigure (THS_FS2G_32mg, 30, (XLIE | YLIE | ZLIE));
}

void vLIS2_EnableFifoSampling (void)
{
	// Set semaphore, no FIFO event yet
	boInterruptEvent = false;

  // Enable X,Y,Z sensors and set a default sample rate
	vTWI_Write(CTRL_REG1, (ODR_10Hz | X_EN | Y_EN | Z_EN));
	
	// Disable high-pass filtering
	vTWI_Write(CTRL_REG2, 0);

	// Enabling FIFO high watermark interrupt
	vTWI_Write(CTRL_REG3, I1_WTM);
	
	// Selecting 12-bit resolution, range +/- 2g
	vTWI_Write(CTRL_REG4, (HIRES_MODE | FS_2G | BDU_ENABLE));
	eRsolution = mode_12bit;
	u8SignExtend = 4;

	// Enable FIFO interrupt, latched on INT1 pin
	vTWI_Write(CTRL_REG5, (FIFO_EN | LIR_INT1));
	
	// Set BYPASS mode to reset the FIFO
	vTWI_Write(FIFO_CTRL_REG, BYPASS);
	uint8_t u8Dummy;
	vTWI_Read(FIFO_SRC_REG, &u8Dummy);

	// Set FIFO to streaming mode, set level of high watermark and
	// enable interrupt on reaching high watermark
	vTWI_Write(FIFO_CTRL_REG, (STREAM_MODE | TR_INT1 | FIFO_WATERMARK_30));
}

/*!
* @brief Polled function call returning interrupt event semaphore.
*/
bool boLIS2_InterruptOccurred (void)
{
	if (boInterruptEvent)
	{
		boInterruptEvent = false;
		return true;
	}
	
	return false;
}

/*!
* @brief Returns interrupt status register,
* clearing any pending interrupts.
*/
uint8_t u8LIS2_EventStatus (void)
{
	uint8_t u8status;
	vTWI_Read(INT1_SRC, &u8status);

	return u8status;
}
	
/*!
* @brief Polled function call that checks for FIFO full events
* and then copies the LIS2 internal FIFO to a local structure.
* Calls a function to dump the FIFO samples to the STDIO.
*/
void vLIS2_Task (void)
{
	if (boInterruptEvent)
	{
		uint8_t u8FifoStat;
		uint8_t u8NumReadings;
		
		bsp_board_led_on(GREEN_LED);

		// clear the interrupt source, ignore status reg contents
		vTWI_Read(INT1_SRC, &u8FifoStat);
		printf("INT1_SRC: 0x%x\n", u8FifoStat);
		
		// Find number of FIFO samples
		vTWI_Read(FIFO_SRC_REG, &u8FifoStat);
		u8NumReadings = u8FifoStat & FSS_MASK;
		printf("Readings: %d\n", u8NumReadings);

		// Read all LIS2 FIFO samples to local structure
		uint8_t u8Cnt, u8Reading, u8LoData;
		for (u8Cnt=0; u8Cnt < u8NumReadings; u8Cnt++)
		{
			// X-axis
			vTWI_Read(OUT_X_LO, &u8Reading);
			u8LoData = u8Reading;
			vTWI_Read(OUT_X_HI, &u8Reading);
			atsFifo[u8Cnt].u16_X = ( (u8Reading << 8) | u8LoData);

			// Y-axis	
			vTWI_Read(OUT_Y_LO, &u8Reading);
			u8LoData = u8Reading;
			vTWI_Read(OUT_Y_HI, &u8Reading);
			atsFifo[u8Cnt].u16_Y = ( (u8Reading << 8) | u8LoData);

			// Z-axis
			vTWI_Read(OUT_Z_LO, &u8Reading);
			u8LoData = u8Reading;
			vTWI_Read(OUT_Z_HI, &u8Reading);
			atsFifo[u8Cnt].u16_Z = ( (u8Reading << 8) | u8LoData);
		}

		// Send samples to STDOUT 
		vDumpFifo(u8NumReadings);
		
		// wait for Fifo to refill
		boInterruptEvent = false;
		
		// Indicate end of fifo processing
		bsp_board_led_off(GREEN_LED);
		
		// Toggle the FIFO state LED
		bsp_board_led_invert(BLUE_LED);
	}
}

/* --- Local scope functions */

/*!
* @brief Handler for LIS2DH12 INT1 interrupt pin, sets a semaphore.
* Interrupt is active on positive edge and latched until cleared by
* reading the INT1 status register.
*/
static void vInterruptHandler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	// Register interrupt event 
	boInterruptEvent = true;
}

/*!
* @brief Configure an IO pin as a positive edge triggered interrupt source.
*/
static void vInterruptInit (void)
{
	ret_code_t err_code;
	boInterruptEvent = false;
	
	if (nrf_drv_gpiote_is_init())
		printf("nrf_drv_gpiote_init already installed\n");
	else
		nrf_drv_gpiote_init();

	// input pin, +ve edge interrupt, no pull-up
	nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
	in_config.pull = NRF_GPIO_PIN_NOPULL;

	// Link this pin interrupt source to its interrupt handler
	err_code = nrf_drv_gpiote_in_init(READY_PIN, &in_config, vInterruptHandler);
	APP_ERROR_CHECK(err_code);

	nrf_drv_gpiote_in_event_enable(READY_PIN, true);
}
/*!
* @brief Configures the LIS2DH12 threshold detectors.
*
* Inputs: u8Level - requested acceleration detection threshold
*					u8Duration - acceleration duration must be sustained
*											 for this duration.
*											 Duration in multiples of sampling rate.
*					u8Mode - detection mode, passing above or below threshold.
*/
static void vThresholdConfigure (uint8_t u8Level, uint8_t u8Duration, uint8_t u8Mode)
{
  // Enable X,Y,Z sensors and set a default sample rate
	vTWI_Write(CTRL_REG1, (ODR_10Hz | X_EN | Y_EN | Z_EN));
	
	// Enable high-pass filtering with highest cut-off frequency
	// and unfiltered samples to the data registers
	vTWI_Write(CTRL_REG2, (HPF_CUTOFF_F0 | HP_IA1));
	
	// Enable INT1 interrupts
	vTWI_Write(CTRL_REG3, I1_IA1);
	
	// Select measurement range to +/- 2g, 12-bit resolution
	vTWI_Write(CTRL_REG4, HIRES_MODE | FS_2G);
	//vTWI_Write(CTRL_REG4, FS_2G);
	eRsolution = mode_12bit;
	u8SignExtend = 4;

	// Set wake-up threshold level
	vTWI_Write(INT1_THS, u8Level);
	
	// Set duration that threshold needs to be held
	vTWI_Write(INT1_DURATION, u8Duration);
	
	// Enable interrupt on INT1 pin
	boInterruptEvent = false;
	vTWI_Write(CTRL_REG5, LIR_INT1);
	
	// Read reference register to force HP filters to current
	// acceleration/tilt value
	uint8_t u8dummy;
	vTWI_Read(REFERENCE_REG, &u8dummy);
	
	// Enable threshold event
	vTWI_Write(INT1_CFG, u8Mode);
}

/*!
* @brief Send the local copy of the LIS2DH12 FIFO samples to STDOUT.
*/
static void vDumpFifo(uint8_t u8Num)
{
	uint8_t u8Cnt;

	for (u8Cnt=0; u8Cnt < u8Num; u8Cnt++)
	{
		// Convert twos-compliment into signed 16-bit integer and remove left justification
		printf("%d %d %d\n",
							((int16_t)atsFifo[u8Cnt].u16_X) / (1 << u8SignExtend),
							((int16_t)atsFifo[u8Cnt].u16_Y) / (1 << u8SignExtend),
							((int16_t)atsFifo[u8Cnt].u16_Z) / (1 << u8SignExtend)
					);
	}
}

#ifdef LIS2DH12_DEBUG
// Useful debug tool, dump accelerometer registers
// via the UART.

// Forward declation - place at top of file.
// 			static void vDebugDumpRegisters(void);

static void vDebugDumpRegisters(void)
{
	uint8_t u8Reg;
	
	vTWI_Read(CTRL_REG0, &u8Reg);
	printf("C_REG_0: 0x%x\n", u8Reg);
	vTWI_Read(CTRL_REG1, &u8Reg);
	printf("C_REG_1: 0x%x\n", u8Reg);
	vTWI_Read(CTRL_REG2, &u8Reg);
	printf("C_REG_2: 0x%x\n", u8Reg);
	vTWI_Read(CTRL_REG3, &u8Reg);
	printf("C_REG_3: 0x%x\n", u8Reg);
	vTWI_Read(CTRL_REG4, &u8Reg);
	printf("C_REG_4: 0x%x\n", u8Reg);
	vTWI_Read(CTRL_REG5, &u8Reg);
	printf("C_REG_5: 0x%x\n", u8Reg);
	vTWI_Read(CTRL_REG6, &u8Reg);
	printf("C_REG_6: 0x%x\n", u8Reg);
	
	vTWI_Read(REFERENCE_REG, &u8Reg);
	printf("REFERENCE: 0x%x\n", u8Reg);
	vTWI_Read(STATUS_REG_ADD, &u8Reg);
	printf("STATUS: 0x%x\n", u8Reg);
	
	vTWI_Read(FIFO_CTRL_REG, &u8Reg);
	printf("FIFO_CTRL: 0x%x\n", u8Reg);
	vTWI_Read(FIFO_SRC_REG, &u8Reg);
	printf("FIFO_SRC: 0x%x\n", u8Reg);
	
	vTWI_Read(INT1_CFG, &u8Reg);
	printf("INT1_CFG: 0x%x\n", u8Reg);
	vTWI_Read(INT1_SRC, &u8Reg);
	printf("INT1_SRC: 0x%x\n", u8Reg);	
	vTWI_Read(INT1_THS, &u8Reg);
	printf("INT1_THS: 0x%x\n", u8Reg);	
	vTWI_Read(INT1_DURATION, &u8Reg);
	printf("INT1_DURATION: 0x%x\n", u8Reg);
	
	vTWI_Read(INT2_CFG, &u8Reg);
	printf("INT2_CFG: 0x%x\n", u8Reg);
	vTWI_Read(INT2_SRC, &u8Reg);
	printf("INT2_SRC: 0x%x\n", u8Reg);	
	vTWI_Read(INT2_THS, &u8Reg);
	printf("INT2_THS: 0x%x\n", u8Reg);	
	vTWI_Read(INT2_DURATION, &u8Reg);
	printf("INT2_DURATION: 0x%x\n", u8Reg);
	
	vTWI_Read(CLICK_SRC, &u8Reg);
	printf("CLICK_SRC: 0x%x\n", u8Reg);
}
#endif // LIS2DH12_DEBUG
