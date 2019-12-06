/*
 * AVR library for handling a serial connection.
 */

#ifndef __AVR_SERIAL_H
#define __AVR_SERIAL_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

// -------------------------------------------------------------------------------------------------

/**
 * @brief      Initialize the UART interface and setup interrupts.
 */
void serial_initialize(void);

/**
 * @brief      Test whether the receive buffer has data in it.
 *
 * @return     true when there is data to read
 */
bool serial_is_data_available(void);

/**
 * @brief      Reads a single byte from the receive buffer. Blocks until data is available.
 *
 * @return     The byte read
 */
uint8_t serial_read(void);

/**
 * @brief      Reads data from the receive buffer into memory. Blocks until data is available.
 *
 * @param      buffer  Buffer to read into
 * @param[in]  length  Number of bytes to read
 */
void serial_read_data(void *buffer, size_t length);

/**
 * @brief      Sends a single byte.
 *
 * @param[in]  c     The byte to send
 */
void serial_write(uint8_t c);

/**
 * @brief      Sends the contents of a buffer.
 *
 * @param[in]  data    The buffer from which to send data
 * @param[in]  length  Number of bytes to send
 */
void serial_write_data(const void *data, size_t length);

#endif
