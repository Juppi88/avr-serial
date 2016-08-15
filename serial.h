/*
 * AVR library for handling a serial connection.
 */

#ifndef __AVR_SERIAL_H
#define __AVR_SERIAL_H

#include <stdbool.h>
#include <stddef.h>

void serial_initialize(void);
bool serial_is_data_available(void);
char serial_read(void);
void serial_read_data(void *buffer, size_t length);
void serial_write(char c);
void serial_write_data(const void *data, size_t length);

#endif
