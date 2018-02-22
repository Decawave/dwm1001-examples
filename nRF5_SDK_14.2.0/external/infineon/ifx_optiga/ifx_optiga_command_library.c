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

/*lint -e???? -save */

#include "ifx_optiga_command_library.h"
#include "ifx_i2c_transport_layer.h"
#include <string.h> // memcpy

// Command headers
#define OPTIGA_CMD_HEADER_LEN                   4
#define HEADER_SPACE                            0x00, 0x00, 0x00, 0x00
#define OPTIGA_CMD_FLAG_FLUSH_LAST_ERROR        0x80

// Command status codes
#define OPTIGA_CMD_STATUS_SUCCESS               0x00

// Command Open Application
#define OPTIGA_CMD_OPEN_APPLICATION             0x70
#define APP_ID                                  0xD2, 0x76, 0x00, 0x00, 0x04, 0x47, 0x65, 0x6E, \
                                                0x41, 0x75, 0x74, 0x68, 0x41, 0x70, 0x70, 0x6C

// Command Set Auth Scheme
#define OPTIGA_CMD_SET_AUTH_SCHEME              0x10
#define OPTIGA_AUTH_ECDSA_SECP256R1_SHA256      0x91
#define OID_PRIVATE_KEY                         OPTIGA_OID_TAG, OPTIGA_OID_PRIVATE_KEY

// Command Set Auth Message
#define OPTIGA_CMD_SET_AUTH_MSG                 0x19
#define OPTIGA_PARAM_CHALLENGE                  0x01
#define OPTIGA_AUTH_MSG_LEN                     16

// Command Get Auth Message
#define OPTIGA_CMD_GET_AUTH_MSG                 0x18
#define OPTIGA_PARAM_SIGNATURE                  0x02

// Command Get Random
#define OPTIGA_CMD_GET_RANDOM                   0x0C

// Command Get Data Object
#define OPTIGA_CMD_GET_DATA_OBJECT              0x01
#define OPTIGA_PARAM_READ_DATA                  0x00
#define OID_CERTIFICATE                         OPTIGA_OID_TAG, OPTIGA_OID_INFINEON_CERT

// Data structure object identifiers
#define OPTIGA_OID_TAG                          0xE0
#define OPTIGA_OID_PRIVATE_KEY                  0xF0
#define OPTIGA_OID_INFINEON_CERT                0xE0
#define OPTIGA_OID_PROJECT_CERT                 0xE1

// Members to use library in blocking mode
static volatile uint8_t   m_ifx_i2c_busy = 0;
static volatile uint8_t   m_ifx_i2c_status;
static volatile uint8_t * m_optiga_rx_buffer;
static volatile uint16_t  m_optiga_rx_len;

static void optiga_create_header(uint8_t* header, uint8_t command, uint8_t param,
    uint16_t payload_len)
{
    header[0] = command | OPTIGA_CMD_FLAG_FLUSH_LAST_ERROR;
    header[1] = param;
    header[2] = payload_len >> 8;
    header[3] = payload_len;
}

static uint16_t optiga_send_apdu(uint8_t* data, uint16_t length)
{
    uint16_t response_len = 0;

    m_ifx_i2c_busy = 1;
    if (ifx_i2c_tl_transceive(data, length, 0, 0))
    {
        return IFX_I2C_STACK_ERROR;
    }
    while (m_ifx_i2c_busy)
    {
        // Wait
    }

    if (m_optiga_rx_len < OPTIGA_CMD_HEADER_LEN)
    {
        return IFX_I2C_STACK_ERROR;
    }
    if (m_optiga_rx_buffer[0] != OPTIGA_CMD_STATUS_SUCCESS)
    {
        return IFX_I2C_STACK_ERROR;
    }

    response_len = (m_optiga_rx_buffer[2] << 8) | m_optiga_rx_buffer[3];
    if (OPTIGA_CMD_HEADER_LEN + response_len != m_optiga_rx_len)
    {
        return IFX_I2C_STACK_ERROR;
    }

    if (m_ifx_i2c_status == IFX_I2C_STACK_SUCCESS)
    {
        return IFX_I2C_STACK_SUCCESS;
    }
    else
    {
        return IFX_I2C_STACK_ERROR;
    }
}

static void ifx_i2c_tl_event_handler(uint8_t event, uint8_t* data, uint16_t data_len)
{
    m_optiga_rx_buffer = data;
    m_optiga_rx_len = data_len;
    m_ifx_i2c_status = event;
    m_ifx_i2c_busy = 0;
}

uint16_t optiga_open_application(void)
{
    uint8_t apdu[] = { HEADER_SPACE, APP_ID };
    optiga_create_header(apdu, OPTIGA_CMD_OPEN_APPLICATION, 0x00,
        sizeof(apdu) - OPTIGA_CMD_HEADER_LEN);

    if (ifx_i2c_tl_init(ifx_i2c_tl_event_handler) != IFX_I2C_STACK_SUCCESS)
    {
        return IFX_I2C_STACK_ERROR;
    }
    return optiga_send_apdu(apdu, sizeof(apdu));
}

uint16_t optiga_set_auth_scheme(void)
{
    uint8_t apdu[] = { HEADER_SPACE, OID_PRIVATE_KEY };
    optiga_create_header(apdu, OPTIGA_CMD_SET_AUTH_SCHEME,
    OPTIGA_AUTH_ECDSA_SECP256R1_SHA256, sizeof(apdu) - OPTIGA_CMD_HEADER_LEN);

    return optiga_send_apdu(apdu, sizeof(apdu));
}

uint16_t optiga_sign(uint8_t* p_message, uint16_t message_length, uint8_t** pp_signature,
    uint32_t* p_signature_len)
{
    uint8_t apdu[OPTIGA_CMD_HEADER_LEN + OPTIGA_AUTH_MSG_LEN];
    optiga_create_header(apdu, OPTIGA_CMD_SET_AUTH_MSG, OPTIGA_PARAM_CHALLENGE,
    OPTIGA_AUTH_MSG_LEN);

    if (message_length != OPTIGA_AUTH_MSG_LEN)
    {
        return IFX_I2C_STACK_ERROR;
    }
    memcpy(apdu + OPTIGA_CMD_HEADER_LEN, p_message, message_length);
    if (optiga_send_apdu(apdu, sizeof(apdu)))
    {
        return IFX_I2C_STACK_ERROR;
    }

    optiga_create_header(apdu, OPTIGA_CMD_GET_AUTH_MSG, OPTIGA_PARAM_SIGNATURE, 0);
    if (optiga_send_apdu(apdu, OPTIGA_CMD_HEADER_LEN))
    {
        return IFX_I2C_STACK_ERROR;
    }

    *pp_signature = (uint8_t*)(m_optiga_rx_buffer + OPTIGA_CMD_HEADER_LEN);
    *p_signature_len = m_optiga_rx_len - OPTIGA_CMD_HEADER_LEN;
    return IFX_I2C_STACK_SUCCESS;
}

uint16_t optiga_read_certificate(uint8_t** pp_cert, uint32_t* p_length)
{
    uint8_t apdu[] = { HEADER_SPACE, OID_CERTIFICATE };
    optiga_create_header(apdu, OPTIGA_CMD_GET_DATA_OBJECT, OPTIGA_PARAM_READ_DATA,
        sizeof(apdu) - OPTIGA_CMD_HEADER_LEN);

    if (optiga_send_apdu(apdu, sizeof(apdu)))
    {
        return IFX_I2C_STACK_ERROR;
    }

    // Determine true length without trailing zero bytes
    if (m_optiga_rx_buffer[OPTIGA_CMD_HEADER_LEN] == 0x30)
    {
        // ASN1 Sequence
        if (m_optiga_rx_buffer[OPTIGA_CMD_HEADER_LEN + 1] == 0x82)
        {
            // ASN1 Extended Length UINT16
            *pp_cert = (uint8_t*)(m_optiga_rx_buffer + OPTIGA_CMD_HEADER_LEN);
            *p_length = ((m_optiga_rx_buffer[OPTIGA_CMD_HEADER_LEN + 2] << 8)
                | m_optiga_rx_buffer[OPTIGA_CMD_HEADER_LEN + 3]) + 4;
            return IFX_I2C_STACK_SUCCESS;
        }
    }
    return IFX_I2C_STACK_ERROR;
}

uint16_t optiga_get_random(uint16_t length, uint8_t* p_random)
{
    uint8_t apdu[] = { HEADER_SPACE, length >> 8, length };
    optiga_create_header(apdu, OPTIGA_CMD_GET_RANDOM, 0x00, 2);

    if (p_random == NULL || length < 0x0008 || length > 0x100)
    {
        return IFX_I2C_STACK_ERROR;
    }

    if (optiga_send_apdu(apdu, sizeof(apdu)))
    {
        return IFX_I2C_STACK_ERROR;
    }

    memcpy(p_random, (uint8_t*)(m_optiga_rx_buffer + OPTIGA_CMD_HEADER_LEN), length);
    return IFX_I2C_STACK_SUCCESS;
}
/* lint -restore */
