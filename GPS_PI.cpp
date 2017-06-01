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
	serialInterface.writeString(BAUDRATE_115200_CODE);
	this_thread::sleep_for(chrono::seconds(1));
	serialInterface.changeBaudrate(115200);
	this_thread::sleep_for(chrono::seconds(1));
	serialInterface.writeString(UPDATE_10HZ_CODE);
	this_thread::sleep_for(chrono::seconds(1));
}

void GPS::startGPS() {

}

void GPS::printRawData() {
	while(true) {
		printf(serialInterface.readLine().c_str()); printf("\n");
	}
}

int main() {
	GPS gps = GPS();
	gps.initialize();
	gps.printRawData();
}
