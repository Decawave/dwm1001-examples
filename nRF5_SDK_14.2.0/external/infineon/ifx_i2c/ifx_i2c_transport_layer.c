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
// IFX I2C Protocol Stack - Transport Layer (source file)

#include "ifx_i2c_transport_layer.h"
#include "ifx_i2c_data_link_layer.h" // include lower layer header
#include <string.h> // functions memcpy, memset

// Transport Layer states
#define TL_STATE_UNINIT                     0x00
#define TL_STATE_IDLE                       0x01
#define TL_STATE_TX                         0x02
#define TL_STATE_RX                         0x04

// Transport Layer header size
#define TL_HEADER_SIZE                      1

// Transport Layer chaining values
#define TL_CHAINING_NO                      0x00
#define TL_CHAINING_FIRST                   0x01
#define TL_CHAINING_INTERMEDIATE            0x02
#define TL_CHAINING_LAST                    0x04
#define TL_CHAINING_ERROR                   0x07

// Transport Layer state and buffer
static volatile uint8_t  m_state = TL_STATE_UNINIT;
static          uint8_t  m_buffer[TL_BUFFER_SIZE];
static volatile uint16_t m_buffer_size;
static volatile uint16_t m_buffer_pos;

// Upper layer event handler
static volatile ifx_i2c_event_handler_t m_upper_layer_event_handler;

// Setup debug log statements
#if IFX_I2C_LOG_TL == 1
#define LOG_TL IFX_I2C_LOG
#else
#define LOG_TL(...)
#endif

// Helper Macro to report an error to the upper layer and return
#define TL_ERROR(void) { m_upper_layer_event_handler(IFX_I2C_STACK_ERROR, 0u, 0u); return; }

// Internal helper function
static uint16_t ifx_i2c_tl_send_next_fragment(void)
{
    uint16_t buffer_offset = m_buffer_pos;

    // Calculate size of fragment (last one might be shorter)
    uint16_t fragment_size = TL_MAX_FRAGMENT_SIZE;
    if (m_buffer_pos + fragment_size > m_buffer_size)
    {
        fragment_size = m_buffer_size - m_buffer_pos;
    }

    // Shift buffer position for later use and start transmission
    m_buffer_pos += fragment_size;
    return ifx_i2c_dl_send_frame(m_buffer + buffer_offset, fragment_size);
}

// Data Link layer event handler
static void ifx_i2c_dl_event_handler(uint8_t event, uint8_t* p_data, uint16_t data_len)
{
    uint8_t pctr;
    uint8_t chaining;

    // Propagate errors to upper layer
    if (event & IFX_I2C_DL_EVENT_ERROR)
    {
        TL_ERROR();
    }

    // Frame transmission in Data Link layer complete, start receiving frames
    if (event & IFX_I2C_DL_EVENT_TX_SUCCESS)
    {
        if (m_state != TL_STATE_TX)
        {
            TL_ERROR();
        }

        if (m_buffer_pos < m_buffer_size)
        {
            // Transmission of one fragment complete, send next fragment
            LOG_TL("[IFX-TL]: TX Success -> send next\n");
            ifx_i2c_tl_send_next_fragment();
        }
        else
        {
            // Transmission of all fragments complete, start receiving fragments
            LOG_TL("[IFX-TL]: TX Success -> done\n");
            m_state       = TL_STATE_RX;
            m_buffer_size = 0;
            if (!(event & IFX_I2C_DL_EVENT_RX_SUCCESS))
            {
                // Received CTRL frame, trigger reception in Data Link layer
                if (ifx_i2c_dl_receive_frame())
                {
                    TL_ERROR();
                }
            }
        }
    }
    // Reception of frame from Data Link layer
    if (event & IFX_I2C_DL_EVENT_RX_SUCCESS)
    {
        // Message must contain at least the transport layer header
        if (data_len < TL_HEADER_SIZE)
        {
            TL_ERROR();
        }

        // Extract chaining type from packet control byte
        pctr = p_data[0];
        chaining = pctr & 0x07;

        // Peer has detected chaining error
        if (chaining == TL_CHAINING_ERROR)
        {
            TL_ERROR();
        }

        // When receiving a starting fragment the buffer must be empty
        if ((chaining == TL_CHAINING_NO || chaining == TL_CHAINING_FIRST) && m_buffer_size)
        {
            TL_ERROR();
        }

        // When receiving an intermediate or last fragment there must already be data in the buffer
        if ((chaining == TL_CHAINING_INTERMEDIATE || chaining == TL_CHAINING_LAST)
            && m_buffer_size == 0)
        {
            TL_ERROR();
        }

        // When the received frame is not the last one, it must have the maximum allowed size
        if ((chaining == TL_CHAINING_FIRST || chaining == TL_CHAINING_INTERMEDIATE)
            && data_len != TL_MAX_FRAGMENT_SIZE)
        {
            TL_ERROR();
        }

        // Check for possible receive buffer overflow
        if (m_buffer_size + data_len - 1 > TL_BUFFER_SIZE)
        {
            TL_ERROR();
        }

        // Copy frame payload to transport layer receive buffer
        memcpy(m_buffer + m_buffer_size, p_data + 1, data_len - 1);
        m_buffer_size += (data_len - 1);

        if (chaining == TL_CHAINING_NO || chaining == TL_CHAINING_LAST)
        {
            LOG_TL("[IFX-TL]: RX Success -> Inform UL\n");

            // Inform upper layer that a packet has arrived
            m_state = TL_STATE_IDLE;
            m_upper_layer_event_handler(IFX_I2C_STACK_SUCCESS, m_buffer, m_buffer_size);
        }
        else
        { // IFX_I2C_TL_CHAINING_FIRST or IFX_I2C_TL_CHAINING_INTERMEDIATE
            LOG_TL("[IFX-TL]: RX Success -> Continue RX\n");

            // Continue receiving frames until packet is complete
            if (ifx_i2c_dl_receive_frame())
            {
                TL_ERROR();
            }
        }
    }
}

// Transport Layer initialization function
uint16_t ifx_i2c_tl_init(ifx_i2c_event_handler_t handler)
{
    LOG_TL("[IFX-TL]: Init\n");

    // Check function argument, a higher layer event handler must be provided
    if (handler == NULL)
    {
        return IFX_I2C_STACK_ERROR;
    }

    // Initialize Data Link layer (and register event handler)
    if (ifx_i2c_dl_init(ifx_i2c_dl_event_handler) != IFX_I2C_STACK_SUCCESS)
    {
        return IFX_I2C_STACK_ERROR;
    }

    m_upper_layer_event_handler = handler;
    m_state                     = TL_STATE_IDLE;

    return IFX_I2C_STACK_SUCCESS;
}

// Transport Layer transmit and receive function
uint16_t ifx_i2c_tl_transceive(uint8_t* p_packet_header, uint16_t packet_header_len,
                               uint8_t* p_packet_payload, uint16_t packet_payload_len)
{
    uint16_t i;
    uint16_t num_fragments;
    uint16_t fragment_size_left;
    uint16_t header_pos = 0, payload_pos = 0;
    uint16_t packet_size = packet_header_len + packet_payload_len;

    LOG_TL("[IFX-TL]: Transceive txlen %d\n", packet_size);

    // Check function arguments
    if (p_packet_header == NULL || packet_header_len == 0)
    {
        return IFX_I2C_STACK_ERROR;
    }
    if ((packet_header_len + packet_payload_len) > TL_BUFFER_SIZE)
    {
        return IFX_I2C_STACK_ERROR;
    }

    // Transport Layer must be idle
    if (m_state != TL_STATE_IDLE)
    {
        return IFX_I2C_STACK_ERROR;
    }
    m_state = TL_STATE_TX;

    // Reset buffer (optional)
    memset(m_buffer, 0x00, sizeof(m_buffer));
    m_buffer_size = m_buffer_pos = 0;

    // Fragment the packet in multiple frames and store all concatenated in the buffer
    num_fragments = (packet_size + TL_MAX_FRAGMENT_SIZE - 1) / TL_MAX_FRAGMENT_SIZE;
    LOG_TL("[IFX-TL]: Transceive num_fragments %d\n", num_fragments);
    for (i = 0; i < num_fragments; i++)
    {
        // Set (or reset) available fragment size
        fragment_size_left = TL_MAX_FRAGMENT_SIZE - TL_HEADER_SIZE;

        // Append fragment header with chaining information
        if (num_fragments == 1)
        {
            m_buffer[m_buffer_size++] = TL_CHAINING_NO;
        }
        else if (i == 0)
        {
            m_buffer[m_buffer_size++] = TL_CHAINING_FIRST;
        }
        else if (i == (num_fragments - 1))
        {
            m_buffer[m_buffer_size++] = TL_CHAINING_LAST;
        }
        else
        {
            m_buffer[m_buffer_size++] = TL_CHAINING_INTERMEDIATE;
        }

        // Append as much as possible from packet header to fragment payload
        if (header_pos < packet_header_len)
        {
            uint16_t remaining = packet_header_len - header_pos;
            uint16_t bytes = fragment_size_left > remaining ? remaining : fragment_size_left;
            LOG_TL("[IFX-TL]: Transceive append %d bytes header\n", bytes);
            memcpy(m_buffer + m_buffer_size, p_packet_header + header_pos, bytes);
            header_pos         += bytes;
            m_buffer_size      += bytes;
            fragment_size_left -= bytes;
        }
        // Append as much as possible from packet payload to fragment payload
        if (payload_pos < packet_payload_len)
        {
            uint16_t remaining = packet_payload_len - payload_pos;
            uint16_t bytes = fragment_size_left > remaining ? remaining : fragment_size_left;
            LOG_TL("[IFX-TL]: Transceive append %d bytes payload\n", bytes);
            memcpy(m_buffer + m_buffer_size, p_packet_payload + payload_pos, bytes);
            payload_pos   += bytes;
            m_buffer_size += bytes;
        }
    }

    return ifx_i2c_tl_send_next_fragment();
}
/* lint -restore */
