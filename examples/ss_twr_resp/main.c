/*! ----------------------------------------------------------------------------
*  @file    main_nRF_responder.c
*  @brief   Single-sided two-way ranging (SS TWR) responder example code
*
*           This is a simple code example which acts as the responder in a SS TWR distance measurement exchange. 
*           This application waits for a "poll" message (recording the RX time-stamp of the poll) expected from 
*           the "SS TWR initiator" example code (companion to this application), and
*           then sends a "response" message recording its TX time-stamp, 
*
* @attention
*
* Copyright 2018 (c) Decawave Ltd, Dublin, Ireland.
*
* All rights reserved.
*
* @author Decawave
*/

#include "sdk_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "bsp.h"
#include "boards.h"
#include "nordic_common.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf.h"
#include "app_error.h"
#include <string.h>

#include "port_platform.h"
#include "deca_types.h"
#include "deca_param_types.h"
#include "deca_regs.h"
#include "deca_device_api.h"

// Defines ---------------------------------------------


//-----------------dw1000----------------------------

static dwt_config_t config = {
  5,                /* Channel number. */
  DWT_PRF_64M,      /* Pulse repetition frequency. */
  DWT_PLEN_128,     /* Preamble length. Used in TX only. */
  DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
  10,               /* TX preamble code. Used in TX only. */
  10,               /* RX preamble code. Used in RX only. */
  0,                /* 0 to use standard SFD, 1 to use non-standard SFD. */
  DWT_BR_6M8,       /* Data rate. */
  DWT_PHRMODE_STD,  /* PHY header mode. */
  (129 + 8 - 8)     /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

//--------------dw1000---end---------------


#define TASK_DELAY        10            /**< Task delay. Delays a LED0 task for 200 ms */
#define TIMER_PERIOD      2000          /**< Timer period. LED1 timer will expire after 1000 ms */

#ifdef USE_FREERTOS
  TaskHandle_t  ss_responder_task_handle;   /**< Reference to SS TWR Initiator FreeRTOS task. */
  extern void ss_responder_task_function (void * pvParameter);
  TaskHandle_t  led_toggle_task_handle;   /**< Reference to LED0 toggling FreeRTOS task. */
  TimerHandle_t led_toggle_timer_handle;  /**< Reference to LED1 toggling FreeRTOS timer. */
#else
  extern int ss_resp_run(void);
#endif    // #ifdef USE_FREERTOS

#ifdef USE_FREERTOS

  /**@brief LED0 task entry function.
  *
  * @param[in] pvParameter   Pointer that will be used as the parameter for the task.
  */

  static void led_toggle_task_function (void * pvParameter)
  {
    UNUSED_PARAMETER(pvParameter);
    while (true)
    {
      LEDS_INVERT(BSP_LED_0_MASK);
      /* Delay a task for a given number of ticks */
      vTaskDelay(TASK_DELAY);
    /* Tasks must be implemented to never return... */
    }
  }

  /**@brief The function to call when the LED1 FreeRTOS timer expires.
  *
  * @param[in] pvParameter   Pointer that will be used as the parameter for the timer.
  */
  static void led_toggle_timer_callback (void * pvParameter)
  {
    UNUSED_PARAMETER(pvParameter);
    LEDS_INVERT(BSP_LED_1_MASK);
}

#endif  // #ifdef USE_FREERTOS

int main(void)
{
  /* Setup some LEDs for debug Green and Blue on DWM1001-DEV */
  LEDS_CONFIGURE(BSP_LED_0_MASK | BSP_LED_1_MASK);
  LEDS_ON(BSP_LED_0_MASK | BSP_LED_1_MASK);

  #ifdef USE_FREERTOS
    /* Create task for LED0 blinking with priority set to 2 */
    UNUSED_VARIABLE(xTaskCreate(led_toggle_task_function, "LED0", configMINIMAL_STACK_SIZE + 200, NULL, 2, &led_toggle_task_handle));

    /* Start timer for LED1 blinking */
    led_toggle_timer_handle = xTimerCreate( "LED1", TIMER_PERIOD, pdTRUE, NULL, led_toggle_timer_callback);
    UNUSED_VARIABLE(xTimerStart(led_toggle_timer_handle, 0));

    /* Create task for SS TWR Initiator set to 2 */
    UNUSED_VARIABLE(xTaskCreate(ss_responder_task_function, "SSTWR_RESP", configMINIMAL_STACK_SIZE + 200, NULL, 2, &ss_responder_task_handle)); 
  #endif  // #ifdef USE_FREERTOS

  //-------------dw1000  ini------------------------------------	

  /* Setup DW1000 IRQ pin */
  nrf_gpio_cfg_input(DW1000_IRQ, NRF_GPIO_PIN_NOPULL); 		//irq

  /* Reset DW1000 */
  reset_DW1000(); 

  /* Set SPI clock to 2MHz */
  port_set_dw1000_slowrate();			

  /* Init the DW1000 */
  if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
  {
    //Init of DW1000 Failed
    while (1)
    {};
  }

  // Set SPI to 8MHz clock
  port_set_dw1000_fastrate();  

  /* Configure DW1000. */
  dwt_configure(&config);

  /* Apply default antenna delay value. Defined in port platform.h */
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);

  /* Set preamble timeout for expected frames.  */
  //dwt_setpreambledetecttimeout(PRE_TIMEOUT);

  dwt_setrxtimeout(0);    // set to NO receive timeout for this simple example   

  //-------------dw1000  ini------end---------------------------	

  // IF WE GET HERE THEN THE LEDS WILL BLINK
  #ifdef USE_FREERTOS
    /* Start FreeRTOS scheduler. */
    vTaskStartScheduler();	

    while(1)
    {}
  #else
    // No RTOS task here so just call the main loop here.
    // Loop forever responding to ranging requests.
    while (1)
    {
      ss_resp_run();
    }
    #endif  // #ifdef USE_FREERTOS
}

/*****************************************************************************************************************************************************
* NOTES:
*
* 1. The single-sided two-way ranging scheme implemented here has to be considered carefully as the accuracy of the distance measured is highly
*    sensitive to the clock offset error between the devices and the length of the response delay between frames. To achieve the best possible
*    accuracy, this response delay must be kept as low as possible. In order to do so, 6.8 Mbps data rate is used in this example and the response
*    delay between frames is defined as low as possible. The user is referred to User Manual for more details about the single-sided two-way ranging
*    process.  NB:SEE ALSO NOTE 3.
* 2. This is the task delay when using FreeRTOS. Task is delayed a given number of ticks. Useful to be able to define this out to see the effect of the RTOS
*    on timing.
* 3. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
*     DW1000 API Guide for more details on the DW1000 driver functions.
*
****************************************************************************************************************************************************/

