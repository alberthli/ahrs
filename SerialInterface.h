#ifndef SerialInterface_H
#define SerialInterface_H

#include <boost/asio.hpp>
#include <string.h>

class SerialInterface {
public:
	SerialInterface(std::string port, unsigned int baudrate);
	virtual ~SerialInterface();
	void writeString(std::string str);
	std::string readLine(); 
	void changeBaudrate(unsigned int newBaudrate);
private:
	boost::asio::io_service io;
    boost::asio::serial_port serial;
};

#endif
