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
// IFX I2C Protocol Stack - Physical Layer (source file)

#include "ifx_i2c_physical_layer.h"
#include "ifx_i2c_hal.h"
#include <string.h> // functions memcpy, memset

// Setup debug log statements
#if IFX_I2C_LOG_PL == 1
#define LOG_PL IFX_I2C_LOG
#else
#define LOG_PL(...)
#endif

// Physical Layer Register addresses
#define PL_REG_DATA                     0x80
#define PL_REG_DATA_REG_LEN             0x81
#define PL_REG_I2C_STATE                0x82

// Physical Layer Register lengths
#define PL_REG_I2C_STATE_LEN            4

// Physical Layer State Register masks
#define PL_REG_I2C_STATE_RESPONSE_READY 0x40
#define PL_REG_I2C_STATE_STATUS_BUSY    0x80

// Physical Layer low level interface constants
#define PL_ACTION_READ_REGISTER         0x01
#define PL_ACTION_WRITE_REGISTER        0x02
#define PL_I2C_CMD_WRITE                0x01
#define PL_I2C_CMD_READ                 0x02

// Physical Layer low level interface variables
static          uint8_t m_buffer[DL_MAX_FRAME_SIZE];
static volatile uint16_t m_buffer_tx_len;
static volatile uint16_t m_buffer_rx_len;
static volatile uint8_t  m_register_action;
static volatile uint8_t  m_i2c_cmd;
static volatile uint16_t m_retry_counter;

// Physical Layer high level interface constants
#define PL_ACTION_WRITE_FRAME           0x01
#define PL_ACTION_READ_FRAME            0x02
#define PL_STATE_UNINIT                 0x00
#define PL_STATE_INIT                   0x01
#define PL_STATE_READY                  0x02
#define PL_STATE_POLL_STATUS            0x03
#define PL_STATE_RXTX                   0x04

// Physical Layer high level interface variables
static volatile uint8_t   m_frame_action;
static volatile uint8_t   m_frame_state = PL_STATE_UNINIT;
static volatile uint8_t   m_status_polling_counter;
static volatile uint8_t * m_tx_frame;
static volatile uint16_t  m_tx_frame_len;
static volatile uint8_t   m_max_frame_size[sizeof(uint16_t)] = { DL_MAX_FRAME_SIZE >> 8, DL_MAX_FRAME_SIZE };
static volatile ifx_i2c_event_handler_t m_upper_layer_event_handler;

// Physical Layer low level interface function
static void ifx_i2c_pl_read_register(uint8_t reg_addr, uint16_t reg_len)
{
    LOG_PL("[IFX-PL]: Read register %x len %d\n", reg_addr, reg_len);

    // Prepare transmit buffer to write register address
    m_buffer[0]     = reg_addr;
    m_buffer_tx_len = 1;

    // Set low level interface variables and start transmission
    m_buffer_rx_len   = reg_len;
    m_register_action = PL_ACTION_READ_REGISTER;
    m_retry_counter   = PL_POLLING_MAX_CNT;
    m_i2c_cmd         = PL_I2C_CMD_WRITE;
    ifx_i2c_transmit(m_buffer, m_buffer_tx_len);
}

// Physical Layer low level interface function
static void ifx_i2c_pl_write_register(uint8_t reg_addr, uint16_t reg_len, uint8_t* p_content)
{
    LOG_PL("[IFX-PL]: Write register %x len %d\n", reg_addr, reg_len);

    // Prepare transmit buffer to write register address and content
    m_buffer[0] = reg_addr;
    memcpy(m_buffer + 1, p_content, reg_len);
    m_buffer_tx_len = 1 + reg_len;

    // Set Physical Layer low level interface variables and start transmission
    m_register_action = PL_ACTION_WRITE_REGISTER;
    m_retry_counter   = PL_POLLING_MAX_CNT;
    m_i2c_cmd         = PL_I2C_CMD_WRITE;
    ifx_i2c_transmit(m_buffer, m_buffer_tx_len);
}

// Physical Layer high level interface timer callback (will be called after the timer expires)
static void ifx_i2c_pl_status_poll_callback(void)
{
    LOG_PL("[IFX-PL]: Timer -> Poll STATUS register\n");
    ifx_i2c_pl_read_register(PL_REG_I2C_STATE, PL_REG_I2C_STATE_LEN);
}

// Physical Layer high level interface state machine (read/write frames)
static void ifx_i2c_pl_frame_event_handler(uint8_t event)
{
    uint16_t frame_size;

    if (event == IFX_I2C_STACK_ERROR)
    {
        // I2C read or write failed, report to upper layer
        m_upper_layer_event_handler(IFX_I2C_STACK_ERROR, 0, 0);
    }

    if (m_frame_state == PL_STATE_INIT)
    {
        m_frame_state = PL_STATE_READY;
        ifx_i2c_pl_write_register(PL_REG_DATA_REG_LEN, sizeof(m_max_frame_size), (uint8_t*)&m_max_frame_size);
    }
    else if (m_frame_state == PL_STATE_READY)
    {
        // Start polling status register
        m_frame_state            = PL_STATE_POLL_STATUS;
        m_status_polling_counter = 0;
        ifx_i2c_pl_read_register(PL_REG_I2C_STATE, PL_REG_I2C_STATE_LEN);
    }
    // Retrieved content of STATUS register
    else if (m_frame_state == PL_STATE_POLL_STATUS)
    {
        if ((m_frame_action == PL_ACTION_READ_FRAME)
            && (m_buffer[0] & PL_REG_I2C_STATE_RESPONSE_READY))
        {
            frame_size = (m_buffer[2] << 8) | m_buffer[3];
            if (frame_size > 0 && frame_size <= DL_MAX_FRAME_SIZE)
            {
                m_frame_state = PL_STATE_RXTX;
                ifx_i2c_pl_read_register(PL_REG_DATA, frame_size);
            }
            else
            { // No data available or length field corrupted
                m_frame_state = PL_STATE_READY;
                m_upper_layer_event_handler(IFX_I2C_STACK_ERROR, 0, 0);
            }
        }
        else if ((m_frame_action == PL_ACTION_WRITE_FRAME)
            && !(m_buffer[0] & PL_REG_I2C_STATE_STATUS_BUSY))
        {
            // Write frame if device is not busy, otherwise wait and poll STATUS again later
            m_frame_state = PL_STATE_RXTX;
            ifx_i2c_pl_write_register(PL_REG_DATA, m_tx_frame_len, (uint8_t*)m_tx_frame);
        }
        else
        {
            // Continue polling STATUS register if retry limit is not reached
            if (m_status_polling_counter++ < PL_POLLING_MAX_CNT)
            {
                ifx_timer_setup(PL_POLLING_INVERVAL_US, ifx_i2c_pl_status_poll_callback);
            }
            else
            {
                m_frame_state = PL_STATE_READY;
                m_upper_layer_event_handler(IFX_I2C_STACK_ERROR, 0, 0);
            }
        }
    }
    else if (m_frame_state == PL_STATE_RXTX)
    {
        // Writing/reading of frame to/from DATA register complete
        m_frame_state = PL_STATE_READY;
        m_upper_layer_event_handler(IFX_I2C_STACK_SUCCESS, m_buffer, m_buffer_rx_len);
    }
}

// Physical Layer low level interface timer callback (will be called after the timer expires)
static void ifx_i2c_hal_poll_callback(void)
{
    if (m_i2c_cmd == PL_I2C_CMD_WRITE)
    {
        LOG_PL("[IFX-PL]: Timer -> Restart TX\n");
        ifx_i2c_transmit(m_buffer, m_buffer_tx_len);
    }
    else if (m_i2c_cmd == PL_I2C_CMD_READ)
    {
        LOG_PL("[IFX-PL]: Timer -> Restart Read Register -> Start TX\n");
        m_i2c_cmd = PL_I2C_CMD_WRITE;
		ifx_i2c_transmit(m_buffer, m_buffer_tx_len);
    }
}

// Physical Layer low level guard time callback
static void ifx_i2c_pl_guard_time_callback(void)
{
    if (m_register_action == PL_ACTION_READ_REGISTER)
    {
    	if (m_i2c_cmd == PL_I2C_CMD_WRITE)
    	{
    LOG_PL("[IFX-PL]: Guard Time elapsed -> Start RX\n");
    m_i2c_cmd = PL_I2C_CMD_READ;
    ifx_i2c_receive(m_buffer, m_buffer_rx_len);
}
    	else if (m_i2c_cmd == PL_I2C_CMD_READ)
    	{
    		LOG_PL("[IFX-PL]: I2C Success -> Done\n");
    		ifx_i2c_pl_frame_event_handler(IFX_I2C_STACK_SUCCESS);
    	}
    }
    else if (m_register_action == PL_ACTION_WRITE_REGISTER)
	{
    	LOG_PL("[IFX-PL]: I2C Success -> Done\n");
    	ifx_i2c_pl_frame_event_handler(IFX_I2C_STACK_SUCCESS);
	}
}

// Physical Layer low level interface state machine (read/write registers)
static void ifx_i2c_pl_hal_event_handler(uint8_t event)
{
    switch (event)
    {
        case IFX_I2C_HAL_ERROR:
            // Error event usually occurs when the device is in sleep mode and needs time to wake up
            if (m_retry_counter--)
            {
                ifx_timer_setup(PL_POLLING_INVERVAL_US, ifx_i2c_hal_poll_callback);
            }
            else
            {
                LOG_PL("[IFX-PL]: I2C Error -> Stop\n");
                ifx_i2c_pl_frame_event_handler(IFX_I2C_STACK_ERROR);
            }
            break;
        case IFX_I2C_HAL_TX_SUCCESS:
        case IFX_I2C_HAL_RX_SUCCESS:
                LOG_PL("[IFX-PL]: I2C Success -> Wait Guard Time\n");
                ifx_timer_setup(PL_GUARD_TIME_INTERVAL_US, ifx_i2c_pl_guard_time_callback);
            break;
    }
}

// Physical Layer high level interface function
uint16_t ifx_i2c_pl_init(ifx_i2c_event_handler_t handler)
{
    LOG_PL("[IFX-PL]: Init\n");

    // Check and register upper layer event handler
    if (handler == NULL)
    {
        return IFX_I2C_STACK_ERROR;
    }
    m_upper_layer_event_handler = handler;

    // Initialize I2C driver (reinit if used before)
    if (ifx_i2c_init(m_frame_state != PL_STATE_UNINIT,
        ifx_i2c_pl_hal_event_handler) != IFX_I2C_STACK_SUCCESS)
    {
        return IFX_I2C_STACK_ERROR;
    }

    // Set Physical Layer internal state
    m_frame_state = PL_STATE_INIT;

    return IFX_I2C_STACK_SUCCESS;
}

// Physical Layer high level interface function
uint16_t ifx_i2c_pl_send_frame(uint8_t* p_frame, uint16_t frame_len)
{
    // Physical Layer must be idle, set requested action
    if (m_frame_state != PL_STATE_INIT && m_frame_state != PL_STATE_READY)
    {
        return IFX_I2C_STACK_ERROR;
    }
    m_frame_action = PL_ACTION_WRITE_FRAME;

    // Store reference to frame for sending it later
    m_tx_frame     = p_frame;
    m_tx_frame_len = frame_len;

    ifx_i2c_pl_frame_event_handler(IFX_I2C_STACK_SUCCESS);
    return IFX_I2C_STACK_SUCCESS;
}

// Physical Layer high level interface function
uint16_t ifx_i2c_pl_receive_frame(void)
{
    // Physical Layer must be idle, set requested action
    if (m_frame_state != PL_STATE_INIT && m_frame_state != PL_STATE_READY)
    {
        return IFX_I2C_STACK_ERROR;
    }
    m_frame_action = PL_ACTION_READ_FRAME;

    ifx_i2c_pl_frame_event_handler(IFX_I2C_STACK_SUCCESS);
    return IFX_I2C_STACK_SUCCESS;
}
/* lint -restore */
