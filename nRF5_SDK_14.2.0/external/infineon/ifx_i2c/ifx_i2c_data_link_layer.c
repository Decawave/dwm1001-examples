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
// IFX I2C Protocol Stack - Data Link Layer (source file)

#include "ifx_i2c_data_link_layer.h"
#include "ifx_i2c_physical_layer.h"  // include lower layer header
#include <string.h>

// Data Link layer internal states
#define DL_STATE_UINIT 0x00
#define DL_STATE_IDLE  0x01
#define DL_STATE_TX    0x02
#define DL_STATE_RX    0x03
#define DL_STATE_ACK   0x04

// Data Link Layer Frame Control Constants
#define DL_FCTR_CONTROL_FRAME     0x80
#define DL_FCTR_SEQCTR_MASK       0x60
#define DL_FCTR_SEQCTR_OFFSET     5
#define DL_FCTR_SEQCTR_VALUE_ACK  0x00
#define DL_FCTR_SEQCTR_VALUE_NACK 0x01
#define DL_FCTR_FRNR_MASK         0x0C
#define DL_FCTR_FRNR_OFFSET       2
#define DL_FCTR_ACKNR_MASK        0x03
#define DL_FCTR_ACKNR_OFFSET      0

// Data Link Layer frame counter max value
#define DL_MAX_FRAME_NUM 4

// Upper layer Event handler
static volatile ifx_i2c_event_handler_t m_upper_layer_event_handler;

// Data Link layer internal state variables
static volatile uint8_t m_state = DL_STATE_UINIT;
static volatile uint8_t m_tx_seq_nr;
static volatile uint8_t m_rx_seq_nr;
static volatile uint8_t m_action_rx_only;
static volatile uint8_t m_retransmit_counter;

// Data Link layer transmit and receive buffers
static uint8_t m_tx_buffer[DL_MAX_FRAME_SIZE];
static uint8_t m_rx_buffer[DL_MAX_FRAME_SIZE];
static volatile uint16_t m_tx_buffer_size;
static volatile uint16_t m_rx_buffer_size;

// Setup debug log statements
#if IFX_I2C_LOG_DL == 1
#define LOG_DL IFX_I2C_LOG
#else
#define LOG_DL(...)
#endif

// Helper function to calculate CRC of a byte
static uint16_t ifx_i2c_dl_calc_crc_byte(uint16_t wSeed, uint8_t bByte)
{
    uint16_t wh1;
    uint16_t wh2;
    uint16_t wh3;
    uint16_t wh4;

    wh1 = (wSeed ^ bByte) & 0xFF;
    wh2 = wh1 & 0x0F;
    wh3 = ((uint16_t)(wh2 << 4)) ^ wh1;
    wh4 = wh3 >> 4;

    return ((uint16_t)((((uint16_t)((((uint16_t)(wh3 << 1)) ^ wh4) << 4)) ^ wh2) << 3)) ^ wh4
        ^ (wSeed >> 8);
}

// Helper function to calculate CRC of a frame
static uint16_t ifx_i2c_dl_calc_crc(uint8_t* p_data, uint16_t data_len)
{
    uint16_t i;
    uint16_t crc = 0;

    for (i = 0; i < data_len; i++)
    {
        crc = ifx_i2c_dl_calc_crc_byte(crc, p_data[i]);
    }

    return crc;
}

static uint16_t ifx_i2c_dl_send_frame_internal(uint8_t* p_frame, uint16_t frame_len,
    uint8_t seqctr_value, uint8_t resend)
{
    uint16_t crc;
    uint16_t ack_nr = m_rx_seq_nr;
    LOG_DL("[IFX-DL]: TX Frame len %d\n", frame_len);

    // In case of sending a NACK the next frame is referenced
    if (seqctr_value == DL_FCTR_SEQCTR_VALUE_NACK)
    {
        ack_nr = (m_rx_seq_nr + 1) % DL_MAX_FRAME_NUM;
    }

    // Set sequence control value (ACK or NACK) and referenced frame number
    m_tx_buffer[0] = (ack_nr << DL_FCTR_ACKNR_OFFSET);
    m_tx_buffer[0] |= (seqctr_value << DL_FCTR_SEQCTR_OFFSET);

    if (frame_len) // Data frame
    {
        // Increment and set frame transmit sequence number
        if (!resend)
        {
            m_tx_seq_nr = (m_tx_seq_nr + 1) % DL_MAX_FRAME_NUM;
        }
        m_tx_buffer[0] |= (m_tx_seq_nr << DL_FCTR_FRNR_OFFSET);
    }
    else // Control frame
    {
        m_tx_buffer[0] |= DL_FCTR_CONTROL_FRAME;
    }

    // Set frame length
    m_tx_buffer[1] = frame_len >> 8;
    m_tx_buffer[2] = frame_len;

    // Copy frame in transmit buffer
    memcpy(m_tx_buffer + 3, p_frame, frame_len);

    // Calculate frame CRC
    crc = ifx_i2c_dl_calc_crc(m_tx_buffer, 3 + frame_len);
    m_tx_buffer[3 + frame_len] = crc >> 8;
    m_tx_buffer[4 + frame_len] = crc;

    // Transmit frame
    m_tx_buffer_size = DL_HEADER_SIZE + frame_len;
    return ifx_i2c_pl_send_frame(m_tx_buffer, m_tx_buffer_size);
}

// Helper Macro to report an error to the upper layer and return
#define DL_ERROR(void) { m_upper_layer_event_handler(IFX_I2C_DL_EVENT_ERROR, 0, 0); return; }

static void ifx_i2c_dl_resend_frame(uint8_t seqctr_value)
{
    if (m_retransmit_counter++ < DL_MAX_RETRIES)
    {
        LOG_DL("[IFX-DL]: Resend Frame\n");
        m_state = DL_STATE_TX;
        if (ifx_i2c_dl_send_frame_internal(m_tx_buffer + 3, m_tx_buffer_size - DL_HEADER_SIZE,
            seqctr_value, 1))
        {
            DL_ERROR();
        }
    }
    else // Retries exhausted, report to error to upper layer
    {
        DL_ERROR();
    }
}

// Helper macro to send a NACK control frame and return
#define DL_RESEND_FRAME(seqctr_value) { ifx_i2c_dl_resend_frame(seqctr_value); return; }

// Data Link Layer state machine
static void ifx_i2c_pl_event_handler(uint8_t event, uint8_t* p_data, uint16_t data_len)
{
    uint8_t fctr = 0, fr_nr, ack_nr, seqctr;
    uint16_t packet_len, crc_received, crc_calculated;

    if (m_state == DL_STATE_TX)
    {
        // If writing a frame failed retry sending
        if (event == IFX_I2C_STACK_ERROR)
        {
            DL_RESEND_FRAME(DL_FCTR_SEQCTR_VALUE_ACK);
        }

        // Transmission successful, start receiving frame
        m_state = DL_STATE_RX;
        if (ifx_i2c_pl_receive_frame())
        {
            DL_ERROR();
        }
    }
    else if (m_state == DL_STATE_RX)
    {
        // If no frame was received retry sending
        if (event == IFX_I2C_STACK_ERROR)
        {
            DL_RESEND_FRAME(DL_FCTR_SEQCTR_VALUE_NACK);
        }

        // Received frame from device, start analyzing
        LOG_DL("[IFX-DL]: RX Frame\n");

        // Check frame length
        if (data_len < DL_HEADER_SIZE)
        {
            LOG_DL("[IFX-DL]: received data_len < DL_HEADER_SIZE\n");
            DL_RESEND_FRAME(DL_FCTR_SEQCTR_VALUE_NACK);
        }
        packet_len = (p_data[1] << 8) | p_data[2];
        if (data_len != DL_HEADER_SIZE + packet_len)
        {
            LOG_DL("[IFX-DL]: data_len != DL_HEADER_SIZE + packet_size\n");
            DL_RESEND_FRAME(DL_FCTR_SEQCTR_VALUE_NACK);
        }

        // Check frame CRC value
        crc_received = (p_data[3 + packet_len] << 8) | p_data[4 + packet_len];
        crc_calculated = ifx_i2c_dl_calc_crc(p_data, 3 + packet_len);
        if (crc_received != crc_calculated)
        {
            LOG_DL("[IFX-DL]: CRC error\n");
            DL_RESEND_FRAME(DL_FCTR_SEQCTR_VALUE_NACK);
        }

        // Check transmit frame sequence number
        fctr = p_data[0];
        seqctr = (fctr & DL_FCTR_SEQCTR_MASK) >> DL_FCTR_SEQCTR_OFFSET;
        ack_nr = (fctr & DL_FCTR_ACKNR_MASK) >> DL_FCTR_ACKNR_OFFSET;
        if ((seqctr == DL_FCTR_SEQCTR_VALUE_NACK) && ack_nr == m_tx_seq_nr)
        {
            LOG_DL("[IFX-DL]: Received nack\n");
            DL_RESEND_FRAME(DL_FCTR_SEQCTR_VALUE_NACK);
        }
        if ((seqctr != DL_FCTR_SEQCTR_VALUE_ACK) || ack_nr != m_tx_seq_nr)
        {
            LOG_DL("[IFX-DL]: Sequence number error\n");
            DL_ERROR();
        }

        if (fctr & DL_FCTR_CONTROL_FRAME)
        {
            LOG_DL("[IFX-DL]: Read Control Frame\n");

            // Received control frame, payload length must be zero
            if (packet_len > 0)
            {
                DL_ERROR();
            }

            // Report frame reception to upper layer and go in idle state
            m_state = DL_STATE_IDLE;
            m_upper_layer_event_handler(IFX_I2C_DL_EVENT_TX_SUCCESS, 0, 0);
        }
        else
        { // Data Frame
          // Check frame receive sequence number and increment local copy
            fr_nr = (fctr & DL_FCTR_FRNR_MASK) >> DL_FCTR_FRNR_OFFSET;
            if (fr_nr != ((m_rx_seq_nr + 1) % DL_MAX_FRAME_NUM))
            {
                DL_ERROR();
            }
            m_rx_seq_nr = (m_rx_seq_nr + 1) % DL_MAX_FRAME_NUM;

            // Data frames must have payload, copy it to Data Link receive buffer
            if (packet_len == 0)
            {
                DL_ERROR();
            }
            memcpy(m_rx_buffer, p_data, data_len);
            m_rx_buffer_size = data_len;

            // Send control frame to acknowledge reception of this data frame
            LOG_DL("[IFX-DL]: Read Data Frame -> Send ACK\n");
            m_state = DL_STATE_ACK;
            m_retransmit_counter = 0;
            ifx_i2c_dl_send_frame_internal(0, 0, DL_FCTR_SEQCTR_VALUE_ACK, 0);
        }
    }
    else if (m_state == DL_STATE_ACK)
    {
        // If writing the ACK frame failed retry
        if (event == IFX_I2C_STACK_ERROR)
        {
            LOG_DL("[IFX-DL]: Physical Layer error -> Resend ACK\n");
            DL_RESEND_FRAME(DL_FCTR_SEQCTR_VALUE_ACK)
        }

        // Control frame successful transmitted
        m_state = DL_STATE_IDLE;
        if (m_action_rx_only)
        {
            m_upper_layer_event_handler(IFX_I2C_DL_EVENT_RX_SUCCESS, m_rx_buffer + 3,
                m_rx_buffer_size - DL_HEADER_SIZE);
        }
        else
        {
            m_upper_layer_event_handler(IFX_I2C_DL_EVENT_TX_SUCCESS | IFX_I2C_DL_EVENT_RX_SUCCESS,
                m_rx_buffer + 3, m_rx_buffer_size - DL_HEADER_SIZE);
        }
    }
}

uint16_t ifx_i2c_dl_init(ifx_i2c_event_handler_t handler)
{
    LOG_DL("[IFX-DL]: Init\n");

    // Check function argument, an upper layer event handler must be provided
    if (handler == NULL)
    {
        return IFX_I2C_STACK_ERROR;
    }

    // Initialize Physical Layer (and register event handler)
    if (ifx_i2c_pl_init(&ifx_i2c_pl_event_handler) != IFX_I2C_STACK_SUCCESS)
    {
        return IFX_I2C_STACK_ERROR;
    }

    // Initialize internal variables
    m_upper_layer_event_handler = handler;
    m_state = DL_STATE_IDLE;
    m_tx_seq_nr = DL_MAX_FRAME_NUM - 1;
    m_rx_seq_nr = DL_MAX_FRAME_NUM - 1;

    return IFX_I2C_STACK_SUCCESS;
}

uint16_t ifx_i2c_dl_send_frame(uint8_t* p_frame, uint16_t frame_len)
{
    // State must be idle and payload available
    if (m_state != DL_STATE_IDLE || !p_frame || !frame_len)
    {
        return IFX_I2C_STACK_ERROR;
    }

    m_state = DL_STATE_TX;
    m_retransmit_counter = 0;
    m_action_rx_only = 0;
    return ifx_i2c_dl_send_frame_internal(p_frame, frame_len, DL_FCTR_SEQCTR_VALUE_ACK, 0);
}

uint16_t ifx_i2c_dl_receive_frame(void)
{
    LOG_DL("[IFX-DL]: Start RX Frame\n");

    if (m_state != DL_STATE_IDLE)
    {
        return IFX_I2C_STACK_ERROR;
    }

    // Set internal state
    m_state = DL_STATE_RX;
    m_retransmit_counter = 0;
    m_action_rx_only = 1;

    return ifx_i2c_pl_receive_frame();
}
/* lint -restore */
