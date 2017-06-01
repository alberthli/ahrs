#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <thread>
#include <chrono>
#include <sstream>
#include <vector>
#include <iterator>
#include <iostream>

#include "GPS_PI.h"
#include "SerialInterface.h"

GPS::GPS() {
	lat = 0.0;
	lon = 0.0;
	speed = 0.0f;
	cmg = 0.0f;
	numSats = 0;
	hdop = 0.0f;
}

GPS::~GPS() {

}

void GPS::initialize() {
	serialInterface.writeString(BAUDRATE_115200_CODE);
	std::this_thread::sleep_for(std::chrono::seconds(1));
	serialInterface.changeBaudrate(115200);
	std::this_thread::sleep_for(std::chrono::seconds(1));
	serialInterface.writeString(UPDATE_10HZ_CODE);
	std::this_thread::sleep_for(std::chrono::seconds(1));
}

void GPS::startGPS() {

}

void GPS::printRawData() {
	while(true) {
		// printf(serialInterface.readLine().c_str()); printf("\n");
		std::vector<std::string> lineData = split(serialInterface.readLine(), ',');
		for (std::vector<std::string>::const_iterator i = lineData.begin(); i != lineData.end(); ++i) {
    		printf(*i); printf("\n");
		}
	}
}

std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, std::back_inserter(elems));
    return elems;
}

int main() {
	GPS gps = GPS();
	gps.initialize();
	gps.printRawData();
}
