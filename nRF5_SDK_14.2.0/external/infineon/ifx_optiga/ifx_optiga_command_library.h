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

#ifndef OPTIGA_COMMAND_LIBRARY_H__
#define OPTIGA_COMMAND_LIBRARY_H__

#include "ifx_i2c_config.h"

/**
 * @defgroup ifx_optiga_library Infineon OPTIGA Trust E Command Library
 * @{
 * @ingroup ifx_optiga
 *
 * @brief Module for application-level commands for Infineon OPTIGA Trust E.
 */

/**
 * @brief Initialize the Infineon OPTIGA Trust E device and host library.
 *
 * This function initializes the Infineon OPTIGA Trust E command library and
 * sends the 'open application' command to the device.
 *
 * @retval  IFX_I2C_STACK_SUCCESS  If function was successful.
 * @retval  IFX_I2C_STACK_ERROR    If the operation failed.
 */
uint16_t optiga_open_application(void);

/**
 * @brief Get a random number.
 *
 * The function retrieves a cryptographic-quality random number
 * from the OPTIGA device. This function can be used as entropy
 * source for various security schemes.
 *
 * @param[in]  length       Length of the random number (range 8 to 256).
 * @param[out] p_random     Buffer to store the data.
 *
 * @retval  IFX_I2C_STACK_SUCCESS  If function was successful.
 * @retval  IFX_I2C_STACK_ERROR    If the operation failed.
 */
uint16_t optiga_get_random(uint16_t length, uint8_t* p_random);

/**
 * @brief Get the Infineon OPTIGA Trust E device certificate.
 *
 * The function retrieves the public X.509 certificate stored in the
 * Infineon OPTIGA Trust E device.
 * This certificate and the contained public key can be used to verify a signature from the device.
 * In addition, the receiver of the certificate can verify the chain of trust
 * by validating the issuer of the certificate and the issuer's signature on it.
 *
 * @param[out] pp_cert      Pointer to the buffer that will contain the output.
 * @param[out] p_length     Pointer to the variable that will contain the length.
 *
 * @retval  IFX_I2C_STACK_SUCCESS If function was successful.
 * @retval  IFX_I2C_STACK_ERROR If the operation failed.
 */
uint16_t optiga_read_certificate(uint8_t** pp_cert, uint32_t* p_length);

/**
 * @brief Set the authentication scheme.
 *
 * This function sets the authentication scheme for the OPTIGA device.
 * Currently only the ECDSA with the elliptic curve SECP256R1 and
 * hash algorithm SHA256 is supported.
 *
 * @attention This function must be called once before calling @ref optiga_sign
 *
 * @retval  IFX_I2C_STACK_SUCCESS If function was successful.
 * @retval  IFX_I2C_STACK_ERROR If the operation failed.
 */
uint16_t optiga_set_auth_scheme(void);

/**
 * @brief Sign a message using the OPTIGA device.
 *
 * The function creates a signature using the scheme selected with @ref
 * optiga_set_auth_scheme. This function can be used to implement device
 * or brand authentication schemes in applications.
 *
 * @param[in]  p_message        Pointer to the buffer containing the message to be signed.
 * @param[in]  message_length   Length of the message.
 * @param[out] pp_signature     Pointer to the buffer that will contain the signature.
 * @param[out] p_signature_len  Pointer to the variable which will contain the signature length.
 *
 * @retval  IFX_I2C_STACK_SUCCESS If function was successful.
 * @retval  IFX_I2C_STACK_ERROR If the operation failed.
 */
uint16_t optiga_sign(uint8_t* p_message, 
                     uint16_t message_length,
                     uint8_t** pp_signature, 
                     uint32_t* p_signature_len);

/**
 * @}
 **/

#endif /* OPTIGA_COMMAND_LIBRARY_H__ */
