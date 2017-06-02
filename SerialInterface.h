#ifndef SerialInterface_H
#define SerialInterface_H

#include <boost/asio.hpp>

class SerialInterface {
public:
    SerialInterface(std::string port, unsigned int baud_rate)
    : io(), serial(io,port)
    {
        serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
    }

    void changeBaudrate(unsigned int newBaudrate) {
    	serial.set_option(boost::asio::serial_port_base::baud_rate(newBaudrate));
    }

    void writeString(std::string s) {
        boost::asio::write(serial,boost::asio::buffer(s.c_str(),s.size()));
    }

    std::string readLine() {
        //Reading data char by char, code is optimized for simplicity, not speed
        using namespace boost;

        char c;
        std::string result;

        for(;;) {
            asio::read(serial,asio::buffer(&c,1));

            switch(c) {
                case '\r':
                    break;
                case '\n':
                    return result;
                default:
                    result+=c;
            }
        }
    }

private:
    boost::asio::io_service io;
    boost::asio::serial_port serial;
};

SerialInterface serialInterface("/dev/ttyS0", 9600);

#endif
