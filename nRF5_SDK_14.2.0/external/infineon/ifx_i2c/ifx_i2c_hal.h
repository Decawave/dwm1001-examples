/*
 * Copyright (c) 2017, Infineon Technologies AG
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1.  Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 * 2.  Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *
 * 3.  Neither the name of the copyright holder nor the names of its contributors
 *     may be used to endorse or promote products derived from this software
 *     without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * @defgroup ifx_i2c_hal Infineon I2C Protocol Stack: Hardware Abstraction Layer
 * @{
 * @ingroup ifx_i2c
 *
 * @brief Module for the data link layer of the Infineon I2C Protocol Stack library.
 */

#ifndef IFX_I2C_HAL_H__
#define IFX_I2C_HAL_H__


#include "ifx_i2c_config.h"


/** @brief Success event propagated to upper layer */
#define IFX_I2C_HAL_TX_SUCCESS            0x01
/** @brief Success event propagated to upper layer */
#define IFX_I2C_HAL_RX_SUCCESS            0x02
/** @brief Error event propagated to upper layer */
#define IFX_I2C_HAL_ERROR                 0x03


/**
 * @brief Function pointer type for upper layer event handler.
 */
typedef void (*IFX_I2C_EventHandler)(uint8_t event);


/**
 * @brief Function for initializing a HAL module.
 *
 * The function initializes a HAL module.
 *
 * @param  reinit   If 1, the call shal re-initializes the HAL module if it was used before.
 *                  If 0, the module is initialized for the first time.
 * @param  handler  Event handler to propagate events to the upper layer
 */
uint16_t ifx_i2c_init(uint8_t reinit, IFX_I2C_EventHandler handler);

/**
 * @brief I2C transmit function to conduct an I2 write on I2C bus.
 *
 * The function conducts an I2C write on the I2C bus.
 *
 * @param  p_data  Pointer to buffer with data to be written to I2C slave.
 * @param  length  Length of data in data buffer.
 */
void ifx_i2c_transmit(uint8_t* p_data, uint16_t length);

/**
 * @brief I2C receive function to conduct an I2 read on I2C bus.
 *
 * The function conducts an I2C read on the I2C bus.
 *
 * @param  p_data   Pointer to buffer where received data shall be stored.
 * @param  length   Number of bytes to read from I2C slave.
 */
void ifx_i2c_receive(uint8_t* p_data, uint16_t length);

/**
 * @brief Callback function to handle elapsed timer.
 */
typedef void (*IFX_Timer_Callback)(void);

/**
 * @brief Timer setup function to initialize and start a timer.
 *
 * The function initializes and starts a timer that will call callback_function
 * after time_ms milliseconds have elapsed.
 *
 * @param  time_us            Time in microseconds after the timer expires.
 * @param  callback_function  Function to be called once timer expired.
 */
void ifx_timer_setup(uint16_t time_us, IFX_Timer_Callback callback_function);

/**
 * @}
 **/

#endif /* IFX_I2C_HAL_H__ */
