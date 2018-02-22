/*!
* @brief Component name:	TWI
*
* Two Wire Interface (I2C) driver.
* Single threaded version blocks with CPU in
* sleep mode until TWI transfer complete event
* occurs.
*
* @file TWI.c
*/

#include <stdio.h>
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_gpiote.h"
#include "TWI.h"

// ****** APVW: ToDo.
// Needs to go into a bsp.h file.

// DWM1001 module has LIS2DH12 pin SDO/SA0 pin pulled high,
// ensuring the on-chip bus address register
// (default setting 0001100x) is set to 00011001.
#define	LIS2DH_ADD			0x19

// DWM1001 module TWI pin allocation, P0.28, P0.29
#define	ARGO_TWI_SCL	28
#define	ARGO_TWI_SDA	29

// Local symbolic constants
#define TWI_INSTANCE_ID		0

// Local variables
// TWI instance structure for Nordic nrf device driver.
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

// Semaphore: true if TWI transfer operation has completed
static volatile bool boTransferDone = false;

static ret_code_t twi_err_code;

// *** Local function declarations
static void vSetSubAdd		(uint8_t u8SubAdd);
static void vEventHandler	(nrf_drv_twi_evt_t const * p_event, void * p_context);
static void vWaitForEvent	(void);

// Public Interface Functions

/*!
* @brief Initialise the nRF52 TWI block
*
* Initialises the TWI bus connected to the LIS2DH12 accelerometer.
* No higher layer initialisation options supported.
* For RTOS version the interrupt priority level would need changing.
*/
void vTWI_Init (void)
{
	// Set TWI bus to LIS2DH12 (accelerometer) to maximum supported
	// bus speed with highest priority (for single threaded operation).
	const nrf_drv_twi_config_t twi_config = {
		.scl                = 28,
		.sda                = 29,
		.frequency          = NRF_TWI_FREQ_400K,
		.interrupt_priority = APP_IRQ_PRIORITY_HIGH,
		.clear_bus_init     = false
	};

	twi_err_code = nrf_drv_twi_init(&m_twi, &twi_config, vEventHandler, NULL);
	APP_ERROR_CHECK(twi_err_code);

	nrf_drv_twi_enable(&m_twi);
}

/*
* void vTWI_Write (uint8_t u8address, uint8_t u8data)
* 
* Parameters.
* u8address: address of LIS2DH12 register
* u8data: data to write at address
*
* Returns: void
*/
void vTWI_Write (uint8_t u8address, uint8_t u8data)
{
	uint8_t au8addData[2];
	au8addData[0] = u8address;
	au8addData[1] = u8data;

	boTransferDone = false;
	twi_err_code = nrf_drv_twi_tx(&m_twi, LIS2DH_ADD, au8addData, sizeof(au8addData), false);
	APP_ERROR_CHECK(twi_err_code);

	vWaitForEvent();
}

/*
* void vTWI_Read (uint8_t u8subAdd, uint8_t *pu8readData)
* 
* Parameters.
* u8subAdd: LIS2DH12 register read address following TWI cmd write
* pu8readData: pointer to buffer for data read from LIS2DH12
*
* Returns: void
*/
void vTWI_Read (uint8_t u8subAdd, uint8_t *pu8readData)
{
	vSetSubAdd(u8subAdd);

	boTransferDone = false;
	twi_err_code = nrf_drv_twi_rx(&m_twi, LIS2DH_ADD, pu8readData, 1);
	APP_ERROR_CHECK(twi_err_code);

	vWaitForEvent();
}

/* --- Local scope functions */

// Sets sub address for read operation following initial TWI comand write.
static void vSetSubAdd(uint8_t u8subAdd)
{
	boTransferDone = false;
	twi_err_code = nrf_drv_twi_tx(&m_twi, LIS2DH_ADD, &u8subAdd, 1, false);
	APP_ERROR_CHECK(twi_err_code);

	vWaitForEvent();
}

/*
* Interrupt event handler for TWI.
* Expecting events for read and write transfers complete,
* anything else is an error condition.
* Transfer complete sets a semaphore (boTransferDone) to 
* release the MCU from __WFE sleep mode.
*/
static void vEventHandler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
	switch (p_event->type)
	{
		case NRF_DRV_TWI_EVT_DONE:
		{
			switch (p_event->xfer_desc.type)
			{
				case NRF_DRV_TWI_XFER_TX:
				case NRF_DRV_TWI_XFER_RX:
				{
					boTransferDone = true;
					break;
				}
				default:
					//printf("unknown xfer_desc.type: %x\n", p_event->xfer_desc.type);
					break;
			}
			break;
		}
		default:
			//printf("Unknown event type: %x\n", p_event->type);
			break;
	}
}

/*
* void vWaitForEvent(void)
*
* For single-threaded systems, this function blocks until the
* TWI transfer complete interrupt occurs. Whilst waiting, the
* CPU enters low-power sleep mode.
*
* For multi-threaded (RTOS) builds, this function would suspend
* the thread until the transfer complete event occurs.
*/

static void vWaitForEvent(void)
{
	do
	{
		__WFE();
	}
	while (! boTransferDone);
}
