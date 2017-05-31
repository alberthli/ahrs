#include "SerialInterface.h"
#include <string.h>

using namespace std;
using namespace boost;

SerialInterface::SerialInterface(string port, unsigned int baudrate) 
: io(), serial(io,port) {
	serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
}

SerialInterface::~SerialInterface() {

}

void changeBaudrate(unsigned int newBaudrate) {
	serial.set_option(boost::asio::serial_port_base::baud_rate(newBaudrate));
}

void SerialInterface::writeString(string str) {
	asio::write(serial, boost::asio::buffer(str.c_str(), str.size()));
}

string SerialInterface::readLine() {
	char c;
    string result;

    for(;;) {
        asio::read(serial, asio::buffer(&c, 1));

        switch(c) {
            case '\r':
                break;
            case '\n':
                return result;
            default:
                result += c;
        }
    }
}
