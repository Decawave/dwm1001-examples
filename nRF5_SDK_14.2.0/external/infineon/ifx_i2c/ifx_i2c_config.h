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
 * @defgroup ifx_i2c_config Infineon I2C Protocol Stack: Configuration
 * @{
 * @ingroup ifx_i2c
 *
 * @brief Module for the configuration of the Infineon I2C Protocol Stack library.
 */

#ifndef IFX_I2C_CONFIG_H__
#define IFX_I2C_CONFIG_H__

// IFX I2C Protocol Stack configuration

/** @brief I2C slave address of the Infineon device */
#ifndef IFX_I2C_BASE_ADDR
#define IFX_I2C_BASE_ADDR           0x30
#endif

/** @brief Physical Layer: polling interval in microseconds */
#ifndef PL_POLLING_INTERVAL_US
#define PL_POLLING_INVERVAL_US      10000
#endif 

/** @brief Physical Layer: guard time interval in microseconds */
#ifndef PL_GUARD_TIME_INTERVAL_US
#define PL_GUARD_TIME_INTERVAL_US   50
#endif

/** @brief Physical layer: maximal attempts */
#ifndef PL_POLLING_MAX_CNT
#define PL_POLLING_MAX_CNT          200
#endif

/** @brief Data link layer: maximum frame size */
#ifndef DL_MAX_FRAME_SIZE
#define DL_MAX_FRAME_SIZE           0xFF
#endif

/** @brief Data link layer: maximum number of retries in case of transmission error */
#ifndef DL_MAX_RETRIES
#define DL_MAX_RETRIES              3
#endif

/** @brief Data link layer: header size (constant) */
#define DL_HEADER_SIZE              5

/** @brief Transport layer: maximum fragment size */
#define TL_MAX_FRAGMENT_SIZE        (DL_MAX_FRAME_SIZE - DL_HEADER_SIZE)
/** @brief Transport layer: size of internal buffer
 *  @note Should be large enough to store an X.509 certificate
 */
#ifndef TL_BUFFER_SIZE
#define TL_BUFFER_SIZE              0x40A
#endif

/** @brief Protocol Stack status codes for success */
#define IFX_I2C_STACK_SUCCESS       0x00
/** @brief Protocol Stack status codes for error */
#define IFX_I2C_STACK_ERROR         0x01

/** @brief Protocol Stack debug switch for physical layer (set to 0 or 1) */
#ifndef IFX_I2C_LOG_PL
#define IFX_I2C_LOG_PL              0
#endif 

/** @brief Protocol Stack debug switch for data link layer (set to 0 or 1) */
#ifndef IFX_I2C_LOG_DL
#define IFX_I2C_LOG_DL              0
#endif

/** @brief Protocol Stack debug switch for transport layer (set to 0 or 1) */
#ifndef IFX_I2C_LOG_TL
#define IFX_I2C_LOG_TL              0
#endif

// Macro to redefine logging macro to platform specific macro
#ifndef IFX_I2C_LOG
    #if IFX_I2C_LOG_PL == 1 || IFX_I2C_LOG_DL == 1 || IFX_I2C_LOG_TL == 1
        #include "nrf_log.h"
        #define IFX_I2C_LOG NRF_LOG_RAW_INFO
    #else
        #define IFX_I2C_LOG(...)
    #endif
#endif

// Protocol Stack Includes
#include <stdint.h>

/** @brief Event handler function prototype */
typedef void (*ifx_i2c_event_handler_t)(uint8_t event, uint8_t* data, uint16_t data_len);

/**
 * @}
 **/

#endif /* IFX_I2C_CONFIG_H__ */
