#ifndef GPS_H
#define GPS_H

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
};

#endif