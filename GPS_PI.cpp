#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include "GPS_PI.h"

GPS::GPS() {
	lat = 0.0;
	lon = 0.0;
	speed = 0.0f;
	cmg = 0.0f;
	numSats = 0;
	hdop = 0.0f;
	uart_filestream = -1;
}

GPS::~GPS() {

}

void GPS::initialize() {
	// FROM: http://www.raspberry-projects.com/pi/programming-in-c/uart-serial-port/using-the-uart
	// Initializing the serial port
	uart_filestream = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY);

	if(uart_filestream == -1) {
		printf("Error - Can't open serial port.\n");
	}

	// Configuring the UART
	// Flags
	// Baud rate:- B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800, B500000, B576000, B921600, B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
	// CSIZE:- CS5, CS6, CS7, CS8
	// CLOCAL - Ignore modem status lines
	// CREAD - Enable receiver
	// IGNPAR = Ignore characters with parity errors
	// ICRNL - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for binary comms!)
	// PARENB - Parity enable
	// PARODD - Odd parity (else even)
	struct termios options;
	tcgetattr(uart_filestream, &options);
	options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;		//<Set baud rate
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(uart_filestream, TCIFLUSH);
	tcsetattr(uart_filestream, TCSANOW, &options);
}

void GPS::startGPS() {

}

void GPS::printRawData() {

}
