#include "serial.h"
#include <string.h>
#include <thread>
#include <chrono>

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
	int baudrate = 9600;
	serial::bytesize_t databits = serial::eightbits;
	serial::parity_t parity = serial::parity_none;
	serial::stopbits_t stopbits = serial::stopbits_one;
	serial::flowcontrol_t flowcontrol = serial::flowcontrol_none;
	Serial* serialObj = new serial::Serial("/dev/ttyS0", baudrate, serial::Timeout(),
	                      databits, parity, stopbits, flowcontrol);

	while(true) {
		std::string rawData = serialObj->readline();
    	std::cout << rawData << std::endl;
	}
}

void GPS::startGPS() {

}

void GPS::printRawData() {
	while(true) {

	}
}

int main() {
	GPS gps = GPS();
	gps.initialize();
}
