#ifndef GPS_H
#define GPS_H

#define UPDATE_10HZ_CODE "$PMTK220,100*2F\r\n"
#define BAUDRATE_115200_CODE "$PMTK251,115200*1F\r\n"

class GPS {
public:
	GPS();
	virtual ~GPS();

	void initialize();
	void startGPS();
	void printRawData();
	std::vector<std::string> splitString(const std::string &s, char delim);

	// Custom string splitter and vectorizer
	template<typename Out>
	void split(const std::string &s, char delim, Out result) {
	    std::stringstream ss;
	    ss.str(s);
	    std::string item;
	    while (std::getline(ss, item, delim)) {
	        *(result++) = item;
	    }
	}

private:
	double lat;
	double lon;
	float speed;
	float cmg;
	int numSats;
	float hdop;
	bool valid;
};

#endif
