#include <avr/io.h>
#include <util/setbaud.h>
#include "serial.h"

// --------------------------------------------------------------------------------

#if defined(__AVR_ATmega8A__) || defined(__AVR_ATmega8A__)
	#define SERIAL_STATUS UCSRA
	#define SERIAL_CONTROL_B UCSRB
	#define SERIAL_CONTROL_C UCSRC
	#define SERIAL_BAUD_HI UBRRH
	#define SERIAL_BAUD_LO UBRRL
	#define SERIAL_DATA UDR
	#define BIT_DOUBLE_SPEED U2X
	#define BIT_DATA_SIZE_0 UCSZ0
	#define BIT_DATA_SIZE_1 UCSZ1
	#define BIT_ENABLE_RECV RXEN
	#define BIT_ENABLE_TRANS TXEN
	#define BIT_RECV_COMPLETE RXC
	#define BIT_DATA_REGISTER_EMPTY UDRE
#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__)
	#define SERIAL_STATUS UCSR0A
	#define SERIAL_CONTROL_B UCSR0B
	#define SERIAL_CONTROL_C UCSR0C
	#define SERIAL_BAUD_HI UBRR0H
	#define SERIAL_BAUD_LO UBRR0L
	#define SERIAL_DATA UDR0
	#define BIT_DOUBLE_SPEED U2X0
	#define BIT_DATA_SIZE_0 UCSZ00
	#define BIT_DATA_SIZE_1 UCSZ01
	#define BIT_ENABLE_RECV RXEN0
	#define BIT_ENABLE_TRANS TXEN0
	#define BIT_RECV_COMPLETE RXC0
	#define BIT_DATA_REGISTER_EMPTY UDRE0
#endif

// --------------------------------------------------------------------------------

void serial_initialize(void)
{
	SERIAL_BAUD_HI = UBRRH_VALUE;
	SERIAL_BAUD_LO = UBRRL_VALUE;

#if USE_2X
	SERIAL_STATUS |= _BV(BIT_DOUBLE_SPEED);
#else
	SERIAL_STATUS &= ~(_BV(BIT_DOUBLE_SPEED));
#endif

	// Send and receive 8-bit data.
	SERIAL_CONTROL_C = _BV(BIT_DATA_SIZE_1) | _BV(BIT_DATA_SIZE_0);

	// Enable RX and TX pins.
	SERIAL_CONTROL_B = _BV(BIT_ENABLE_RECV) | _BV(BIT_ENABLE_TRANS);
}

bool serial_is_data_available(void)
{
	// Data is available for reading if the RXC0 bit is set in the UCSR0A register.
	return bit_is_set(SERIAL_STATUS, BIT_RECV_COMPLETE);
}

char serial_read(void)
{
	// Make sure there is data available before reading.
	loop_until_bit_is_set(SERIAL_STATUS, BIT_RECV_COMPLETE);

	// Return the value in the register.
	return SERIAL_DATA;
}

void serial_read_data(void *buffer, size_t length)
{
	char *p = (char *)buffer;

	while (length-- > 0) {
		*p = serial_read();
		++p;
	}
}

void serial_write(char c)
{
	// Wait until the send register is empty and new data can be sent.
	loop_until_bit_is_set(SERIAL_STATUS, BIT_DATA_REGISTER_EMPTY);

	// Write the data into the register.
	SERIAL_DATA = (int)c;
}

void serial_write_data(const void *data, size_t length)
{
	const char *p = (const char *)data;

	while (length-- > 0) {
		serial_write(*p++);
	}
}
