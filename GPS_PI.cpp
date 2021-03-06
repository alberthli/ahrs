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

using namespace std;

GPS::GPS() {
	lat = 0.0;
	lon = 0.0;
	speed = 0.0f;
	cmg = 0.0f;
	numSats = 0;
	hdop = 0.0f;
	valid = false;
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
	// We receive 4 different sentence types that we can parse: 

	// GPGGA - Time of Fix, Lat, Long, Fix Qual, Sats Tracked, HDOP, Alt (m) Above Mean Sea Level, Height of Geoid, Time Since Updating
	// GPGSA - A/M Fix Selection, 3D Fix, PRN of Fix Sats, PDOP, HDOP, VDOP
	// GPRMC - Time of Fix, A/V Status, Lat, Long, Speed (Knots), Track Angle (Deg), Date, Mag Variation
	// GPVTG - True Track Made Good, Magnetic Track Made Good, Speed (Knots), Speed (KM/H)

	// We basically can survive off of just GPRMC sentences, but it's useful to parse other sentences for potential weighting

	try {
		while(true) {
			vector<string> lineData = splitString(serialInterface.readLine(), ',');

			// GPRMC CODES
			if(!lineData.at(0).compare("$GPRMC")) {

				// Determining validity of satellite data
				if(lineData.at(2).length() > 0) {
					
					if(!lineData.at(2).compare("A")) {
						valid = true;

					} else {
						valid = false;
						continue;
					}
					
				}

				// Getting latitude
				string latString = lineData.at(3);

				if(latString.length() > 0) {
					float lat1 = ::atof(latString.substr(0, 2).c_str());
					float lat2 = ::atof(latString.substr(2).c_str()) / 60.0f;

					lat = lat1 + lat2;

					if(!lineData.at(4).compare("S")) {
						lat *= -1;
					}

				} else {
					continue;
				}

				// Getting longitude
				string lonString = lineData.at(5);

				if(latString.length() > 0) {
					float lon1 = ::atof(lonString.substr(0, 3).c_str());
					float lon2 = ::atof(lonString.substr(3).c_str()) / 60.0f;

					lon = lon1 + lon2;

					if(!lineData.at(6).compare("W")) {
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
					cmg = ::atof(lineData.at(8).c_str());

				} else {
					continue;
				}

			// GPGGA CODES
			} else if(!lineData.at(0).compare("$GPGGA")) {

				// Getting number of satellites
				if(lineData.at(7).length() > 0) {
					numSats = ::stoi(lineData.at(7).c_str());

				} else {
					continue;
				}

			// GPGSA CODES
			} else if(!lineData.at(0).compare("$GPGSA")) {

				// Getting the horizontal dilution of precision
				if(lineData.at(16).length() > 0) {
					hdop = ::atof(lineData.at(16).c_str());

				} else {
					continue;
				}

			// GPVTG CODES
			} else if(!lineData.at(0).compare("$GPVTG")) {

			} else {
				continue;
			}

			// Debug prints
			/*
			printf("valid: %s\n", valid ? "true" : "false");
			printf("lat: %f\n", lat);
			printf("lon: %f\n", lon);
			printf("speed: %f\n", speed);
			printf("cmg: %f\n", cmg);
			printf("numSats: %i\n", numSats);
			printf("hdop: %f\n\n", hdop);
			*/

		}

	// Catching a parsing error - happens when comm accidentally skips bytes
	} catch(std::invalid_argument) {
		startGPS();
	}
}

// Debug print method
void GPS::printRawData() {
	while(true) {
		cout << serialInterface.readLine().c_str() << "\n";
	}
}

// String splitting method
vector<string> GPS::splitString(const string &s, char delim) {
    vector<string> elems;
    split(s, delim, back_inserter(elems));
    return elems;
}

int main() {
	GPS gps = GPS();
	gps.initialize();
	gps.startGPS();
}
