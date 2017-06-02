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
	while(true) {
		std::vector<std::string> lineData = splitString(serialInterface.readLine(), ',');

		if(!lineData.at(0).compare("$GPRMC")) {

			// Getting latitude
			std::vector<std::string> latString = lineData.at(3);

			if(latString.length() > 0) {
				float lat1 = ::atof(std::string::substr(0, 3).c_str());
				float lat2 = ::atof(std::string::substr(2).c_str()) / 60.0f;

				lat = lat1 + lat2;

				if(!lineData.at(4)compare("S")) {
					lat *= -1;
				}

			} else {
				continue;
			}

			// Getting longitude
			std::vector<std::string> lonString = lineData.at(5);

			if(latString.length() > 0) {
				float lon1 = ::atof(std::string::substr(0, 4).c_str());
				float lon2 = ::atof(std::string::substr(3).c_str()) / 60.0f;

				lon = lon1 + lon2;

				if(!lineData.at(6)compare("W")) {
					lon *= -1;
				}

			} else {
				continue;
			}

			// Getting speed
			if(lineData.at(7).length() > 0) {
				speed = ::atof(lineData.at(7).c_str()) * 0.51444444f;
			} else {
				continue;
			}

			// Getting course
			if(lineData.at(8).length() > 0) {
				speed = ::atof(lineData.at(8).c_str());
			} else {
				continue;
			}

		} else if(!lineData.at(0).compare("$GPGGA")) {

			// Getting number of satellites
			if(lineData.at(7).length() > 0) {
				numSats = ::stoi(lineData.at(7).c_str());
			} else {
				continue;
			}

		} else if(!lineData.at(0).compare("$GPGSA")) {

			// Getting the horizontal dilution of precision
			if(lineData.at(16).length() > 0) {
				hdop = ::atof(lineData.at(16).c_str());
			} else {
				continue;
			}

		} else if(!lineData.at(0).compare("$GPVTG")) {

		} else {
			continue;
		}
	}
}

void GPS::printRawData() {
	while(true) {
		// std::cout << serialInterface.readLine().c_str()[0] << "\n";
		std::vector<std::string> lineData = splitString(serialInterface.readLine(), ',');
		std::cout << lineData.at(0) << "\n";
	}
}

std::vector<std::string> GPS::splitString(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, std::back_inserter(elems));
    return elems;
}

int main() {
	GPS gps = GPS();
	gps.initialize();
	gps.printRawData();
}
