#include <avr/io.h>
#include <avr/interrupt.h>
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

// -------------------------------------------------------------------------------------------------

#ifdef SERIAL_USE_INTERRUPT

/**
 * The size of read buffer in bytes
 */
#define RX_BUFFER_SIZE 0x20 // 32 bytes
#define RX_BUFFER_MASK 0x1F

/**
 * Read buffer. Used as a ring buffer.
 */
static uint8_t rx_buffer[RX_BUFFER_SIZE];

/**
 * Current read and write offsets for the read buffer.
 */
static volatile uint8_t rx_read_offset = 0;
static volatile uint8_t rx_write_offset = 0;

/**
 * Interrupt handler for bytes received over UART.
 */
ISR(USART_RX_vect)
{
	// Read the received byte and advance the write offset.
	rx_buffer[rx_write_offset] = SERIAL_DATA;
	rx_write_offset = (rx_write_offset + 1) & RX_BUFFER_MASK;

	// If the read and write offsets become the same, the buffer has overflown.
	// Advance the read buffer as well to discard the oldest bytes.
	if (rx_write_offset == rx_read_offset) {
		rx_read_offset = (rx_read_offset + 1) & RX_BUFFER_MASK;
	}
}
#endif

// -------------------------------------------------------------------------------------------------

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

	   UCSR0B |= (1 << RXCIE0); // Enable the USART Recieve Complete interrupt (USART_RXC)
   sei(); // Enable the Global Interrupt
}

#ifdef SERIAL_USE_INTERRUPT

bool serial_is_data_available(void)
{
	return (rx_read_offset != rx_write_offset);
}

uint8_t serial_read(void)
{
	// Make sure there is data available before reading.
	while (!serial_is_data_available()) {}

	// Read the byte in the read buffer and advance the read offset.
	uint8_t value = rx_buffer[rx_read_offset];
	rx_read_offset = (rx_read_offset + 1) & RX_BUFFER_MASK;

	return value;
}

#else

bool serial_is_data_available(void)
{
	// Data is available for reading if the RXC0 bit is set in the UCSR0A register.
	return bit_is_set(SERIAL_STATUS, BIT_RECV_COMPLETE);
}

uint8_t serial_read(void)
{
	// Make sure there is data available before reading.
	loop_until_bit_is_set(SERIAL_STATUS, BIT_RECV_COMPLETE);

	// Return the value in the register.
	return SERIAL_DATA;
}

#endif

void serial_read_data(void *buffer, size_t length)
{
	uint8_t *p = (uint8_t *)buffer;

	while (length-- > 0) {
		*p = serial_read();
		++p;
	}
}

void serial_write(uint8_t c)
{
	// Wait until the send register is empty and new data can be sent.
	loop_until_bit_is_set(SERIAL_STATUS, BIT_DATA_REGISTER_EMPTY);

	// Write the data into the register.
	SERIAL_DATA = c;
}

void serial_write_data(const void *data, size_t length)
{
	const uint8_t *p = (const uint8_t *)data;

	while (length-- > 0) {
		serial_write(*p++);
	}
}
