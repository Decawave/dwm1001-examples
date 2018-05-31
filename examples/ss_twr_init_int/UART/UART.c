/*!
* @brief Component name:	UART
*
* Simple two-wire UART application level driver.
* Provides buffered UART interface, compatible with
* a redirected STDIO for printf and getc.
*
* @file UART.c
*/

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "nrf_uart.h"
#include "UART.h"
#include "bsp.h"
#include "boards.h"

#define NO_PARITY	false

// UART circular buffers - Tx and Rx size
#define UART_TX_BUF_SIZE 512
#define UART_RX_BUF_SIZE 32

// UART initialisation structure
const app_uart_comm_params_t comm_params =
{
	RX_PIN_NUM,
  TX_PIN_NUM,
	RTS_PIN_NUM,
  CTS_PIN_NUM,
  APP_UART_FLOW_CONTROL_DISABLED,
  NO_PARITY,
  NRF_UART_BAUDRATE_115200
};

// local functions
static void vHandleUartInternalErrors (uint32_t u32Error);
static void vUartErrorHandle					(app_uart_evt_t * p_event);

/**
 * @brief Public interface, initialise the FIFO UART.
 */
bool boUART_Init(void)
{
	// Initialis the nrf UART driver returning state
	uint32_t err_code;

  APP_UART_FIFO_INIT
		(	&comm_params,
			UART_RX_BUF_SIZE,
      UART_TX_BUF_SIZE,
      vUartErrorHandle,
      APP_IRQ_PRIORITY_LOWEST,
      err_code
		);

	return (err_code == NRF_SUCCESS) ? true : false;
}

bool boUART_getc(uint8_t *u8ch)
{
	bool boSuccess = false;
	
	if (app_uart_get(u8ch) == NRF_SUCCESS)
		boSuccess = true;
	
	return boSuccess;
}

static void vUartErrorHandle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        vHandleUartInternalErrors(p_event->evt_type);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        vHandleUartInternalErrors(p_event->evt_type);
    }
}

static void vHandleUartInternalErrors (uint32_t u32Error)
{
	// notify app of error - LED ?
}

