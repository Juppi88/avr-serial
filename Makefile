TARGET = libavr-serial
MCU = atmega328p
MCU_PROGRAMMER = m328p
F_CPU = 16000000
BAUD = 9600

CFLAGS = -std=c99 -DF_CPU=$(F_CPU)L -mmcu=$(MCU) -DBAUD=$(BAUD) -Wall -Os

compile:
	# Compile the source for the library.
	avr-gcc $(CFLAGS) -c serial.c

	# Create the static library.
	ar rcs $(TARGET).a serial.o

asm:
	avr-gcc -S $(CFLAGS) -c serial.c

clean:
	rm -f *.o *.a

all:
	compile
