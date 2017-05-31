#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <thread>
#include <chrono>

#include "GPS_PI.h"
#include "SerialInterface.h"

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
	// Trying with Boost
	SerialInterface serial("/dev/ttyS0", 9600);
	serial.writeString(BAUDRATE_115200_CODE);
	serial.changeBaudrate(115200);
	serial.writeString(UPDATE_10HZ_CODE);

	// [ISSUES] 
	// I need to start the baudrate at 9600 and then switch it to 115200
	// I need to know how to read lines

	// FROM: http://www.raspberry-projects.com/pi/programming-in-c/uart-serial-port/using-the-uart
	// Initializing the serial port
	/*
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
	options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(uart_filestream, TCIFLUSH);
	tcsetattr(uart_filestream, TCSANOW, &options);

	unsigned char update_10_hz_code[17] = {'$', 'P', 'M', 'T', 'K', '2', '2', '0', ',', '1', '0', '0', '*', '2', 'F', '\r', '\n'};
	unsigned char baudrate_115200_code[20] = {'$', 'P', 'M', 'T', 'K', '2', '5', '1', ',', '1', '1', '5', '2', '0', '0', '*', '1', 'F', '\r', '\n'};

	write(uart_filestream, baudrate_115200_code, strlen(baudrate_115200_code));
	std::this_thread::sleep_for(std::chrono::seconds(1));

	write(uart_filestream, update_10_hz_code, strlen(update_10_hz_code));
	std::this_thread::sleep_for(std::chrono::seconds(1));
	*/
}

void GPS::startGPS() {

}

void GPS::printRawData() {
	while(true) {
		printf(serial.readLine()); printf("\n");
	}
}

int main() {
	GPS gps = GPS();
	gps.printRawData();
}
