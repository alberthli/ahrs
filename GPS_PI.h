#ifndef GPS_H
#define GPS_H

#include "SerialInterface.h"

#define UPDATE_10HZ_CODE "$PMTK220,100*2F\r\n"
#define BAUDRATE_115200_CODE "$PMTK251,115200*1F\r\n"

class GPS {
public:
	GPS();
	virtual ~GPS();

	void initialize();
	void startGPS();
	void printRawData();

private:
	double lat;
	double lon;
	float speed;
	float cmg;
	int numSats;
	float hdop;
	int uart_filestream;
	SerialInterface serial;
};

#endif