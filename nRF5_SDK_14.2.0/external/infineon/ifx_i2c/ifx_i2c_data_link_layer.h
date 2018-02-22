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
 * @defgroup ifx_i2c_data_link_layer Infineon I2C Protocol Stack: Data Link Layer
 * @{
 * @ingroup ifx_i2c
 *
 * @brief Module for the data link layer of the Infineon I2C Protocol Stack library.
 */

#ifndef IFX_I2C_DATA_LINK_LAYER_H__
#define IFX_I2C_DATA_LINK_LAYER_H__


#include "ifx_i2c_config.h"


/** @brief Error event propagated to upper layer */
#define IFX_I2C_DL_EVENT_ERROR              0x01
/** @brief Transmit success event propagated to upper layer */
#define IFX_I2C_DL_EVENT_TX_SUCCESS         0x02
/** @brief Receive success event propagated to upper layer */
#define IFX_I2C_DL_EVENT_RX_SUCCESS         0x04


/**
 * @brief Function for initializing the module.
 *
 * Function initializes and enables the module and registers
 * an event handler to receive events from this module.
 * @attention This function must be called before using the module.
 *
 * @param[in] handler     Function pointer to the event handler of the upper layer.
 *
 * @retval  IFX_I2C_STACK_SUCCESS If initialization was successful.
 * @retval  IFX_I2C_STACK_ERROR If the module is already initialized.
 */
uint16_t ifx_i2c_dl_init(ifx_i2c_event_handler_t handler);

/**
 * @brief Function for sending a frame.
 *
 * Asynchronous function to send a frame. The function returns immediately.
 * One of the following events is propagated to the event handler registered
 * with @ref ifx_i2c_dl_init.
 *
 * @param[in] p_frame       Pointer to buffer containing the frame.
 * @param[in] frame_len     Frame length.
 *
 * @retval  IFX_I2C_STACK_SUCCESS If function was successful.
 * @retval  IFX_I2C_STACK_ERROR If the module is busy.
 */
uint16_t ifx_i2c_dl_send_frame(uint8_t* p_frame, uint16_t frame_len);

/**
 * @brief Function for receiving a frame.
 *
 * Asynchronous function to receive a frame. The function returns immediately.
 * One of the following events is propagated to the event handler registered
 * with @ref ifx_i2c_dl_init.
 *
 * @retval  IFX_I2C_STACK_SUCCESS If function was successful.
 * @retval  IFX_I2C_STACK_ERROR If the module is busy.
 */
uint16_t ifx_i2c_dl_receive_frame(void);

/**
 * @}
 **/

#endif /* IFX_I2C_DATA_LINK_LAYER_H__ */
