#include "LSM9DS0_PI.h"
#include <iostream>
#include <chrono>
// #include "I2C8Bit.h" // Someone's custom I2C Library
using namespace std;

LSM9DS0::LSM9DS0() {

}

LSM9DS0::~LSM9DS0() {

}

////////////////////////////
// INITIALIZATION METHODS //
////////////////////////////

void LSM9DS0::initialize() {
	
	
}

// Initializing desired settings on the XM
void LSM9DS0::initXM() {
	// Bit Register Configuration Info in Header File
	writeXM(CTRL_REG0_XM, 0x00); // All disabled, defaults
	writeXM(CTRL_REG1_XM, 0x57); // 100 Hz Accel. Sampling Rate, Continuous Update, All Axes Enabled
	writeXM(CTRL_REG2_XM, 0x08); // 773 Hz AAFB, +/- 4g, Normal Self-Test, 4 Wire Interface
	writeXM(CTRL_REG3_XM, 0x00); // Disable interrupts for now
	writeXM(CTRL_REG4_XM, 0x00);
	writeXM(CTRL_REG5_XM, 0xF4); // Temperature Enabled, High Mag Res, 100 Hz Sampling, No Latched Ints
	writeXM(CTRL_REG6_XM, 0x00); // +/- 2 Gauss Scale
	writeXM(CTRL_REG7_XM, 0x00); // Defaults

	// These values need to be changed if you change the operating range of the sensors!
	accelGain = 0.000122;
	magGain = 0.00008;
}

// Initializing desired settings on the G
void LSM9DS0::initG() {
	// Bit Register Configuration Info in Header File
	writeG(CTRL_REG_1_G, 0x0F); // Default ODR and Bandwidth, Normal Mode, All Axes Enabled
	writeG(CTRL_REG_2_G, 0x00); // Normal Mode, 7.2 Hz HPF Cutoff Frequency
	writeG(CTRL_REG_3_G, 0x00); // Defaults
	writeG(CTRL_REG_4_G, 0x00); // Defaults, Gyro Scale +/- 245 DPS
	writeG(CTRL_REG_5_G, 0x00); // Defaults

	// This value needs to be changed if you change the operating range of the gyro!
	gyroGain = 0.00875;
}

/////////////////
// I2C Methods //
/////////////////

// Reads data from the XM device and returns it as an unsigned int
uint8_t LSM9DS0::readXM(uint8_t reg_address) {
	I2C8Bit xm(XM_ADDRESS, string("/dev/i2c-1"));

	unsigned char data = 0;
	xm.readReg(reg_address, data);
	return (uint8_t)data;
}

// Reads data from the G device and returns it as an unsigned int
uint8_t LSM9DS0::readG(uint8_t reg_address) {
	I2C8Bit g(G_ADDRESS, string("/dev/i2c-1"));

	unsigned char data = 0;
	g.readReg(reg_address, data);
	return (uint8_t)data;
}

// Writes data to the XM device
void LSM9DS0::writeXM(uint8_t reg_address, uint8_t data) {
	I2C8Bit xm(XM_ADDRESS, string("/dev/i2c-1"));
	xm.writeReg((unsigned char)reg_address, (unsigned char)data);
}

// Writes data to the G device
void LSM9DS0::writeG(uint8_t reg_address, uint8_t data) {
	I2C8Bit g(G_ADDRESS, string("/dev/i2c-1"));
	g.writeReg((unsigned char)reg_address, (unsigned char)data);
}

//////////////////////////////
// Sensor Retrieval Methods //
//////////////////////////////

// Retrieves the temperature (deg C)
float LSM9DS0::getTemp() {
	uint8_t temp_MSBs = readXM(OUT_TEMP_H_XM);
	uint8_t temp_LSBs = readXM(OUT_TEMP_L_XM);
	// 12 bit resolution, right-justified
	int16_t bitTemp = ( ((uint16_t) temp_MSBs) << 8 | temp_LSBs) & 0x0FFF;

	if(bitTemp > 2047) {
		bitTemp -= 4096;
	}

	return TEMP_INTERCEPT + (float)bitTemp * TEMP_GAIN;
}

// Retrives the x acceleration (m*s^-2)
float LSM9DS0::getxAccel() {
	uint8_t xAccel_MSBs = readXM(OUT_X_H_A);
	uint8_t xAccel_LSBs = readXM(OUT_X_L_A);
	// 16 bit resolution, left-justified
	int16_t xBitAccel = (uint16_t) xAccel_MSBs << 8 | xAccel_LSBs;

	return xBitAccel * accelGain * GRAV_ACCEL;
}

// Retrives the y acceleration (m*s^-2)
float LSM9DS0::getyAccel() {
	uint8_t yAccel_MSBs = readXM(OUT_Y_H_A);
	uint8_t yAccel_LSBs = readXM(OUT_Y_L_A);
	// 16 bit resolution, left-justified
	int16_t yBitAccel = (uint16_t) yAccel_MSBs << 8 | yAccel_LSBs;
  
	return yBitAccel * accelGain * GRAV_ACCEL;
}

// Retrives the z acceleration (m*s^-2)
float LSM9DS0::getzAccel() {
	uint8_t zAccel_MSBs = readXM(OUT_Z_H_A);
	uint8_t zAccel_LSBs = readXM(OUT_Z_L_A);
	// 16 bit resolution, left-justified
	int16_t zBitAccel = (uint16_t) zAccel_MSBs << 8 | zAccel_LSBs;

	return zBitAccel * accelGain * GRAV_ACCEL;
}

// Retrieves the x magnetic field value (gauss)
float LSM9DS0::getxMag() {
	uint8_t xMag_MSBs = readXM(OUT_X_H_M);
	uint8_t xMag_LSBs = readXM(OUT_X_L_M);
	// 16 bit resolution, left-justified
	int16_t xBitMag = (uint16_t) xMag_MSBs << 8 | xMag_LSBs;

	return xBitMag * magGain;
}

// Retrieves the y magnetic field value (gauss)
float LSM9DS0::getyMag() {
	uint8_t yMag_MSBs = readXM(OUT_Y_H_M);
	uint8_t yMag_LSBs = readXM(OUT_Y_L_M);
	// 16 bit resolution, left-justified
	int16_t yBitMag = (uint16_t) yMag_MSBs << 8 | yMag_LSBs;

	return yBitMag * magGain;
}

// Retrieves the z magnetic field value (gauss)
float LSM9DS0::getzMag() {
	uint8_t zMag_MSBs = readXM(OUT_Z_H_M);
	uint8_t zMag_LSBs = readXM(OUT_Z_L_M);
	// 16 bit resolution, left-justified
	int16_t zBitMag = (uint16_t) zMag_MSBs << 8 | zMag_LSBs;

	return zBitMag * magGain;
}

// Retrieves the x gyro value (DPS)
float LSM9DS0::getxGyro() {
	uint8_t xGyro_MSBs = readG(OUT_X_H_G);
	uint8_t xGyro_LSBs = readG(OUT_X_L_G);
	// 16 bit resolution, left-justified
	int16_t xBitGyro = (uint16_t) xGyro_MSBs << 8 | xGyro_LSBs;

	return xBitGyro * gyroGain;
}

// Retrieves the y gyro value (DPS)
float LSM9DS0::getyGyro() {
	uint8_t yGyro_MSBs = readG(OUT_Y_H_G);
	uint8_t yGyro_LSBs = readG(OUT_Y_L_G);
	// 16 bit resolution, left-justified
	int16_t yBitGyro = (uint16_t) yGyro_MSBs << 8 | yGyro_LSBs;

	return yBitGyro * gyroGain;
}

// Retrieves the z gyro value (DPS)
float LSM9DS0::getzGyro() {
	uint8_t zGyro_MSBs = readG(OUT_Z_H_G);
	uint8_t zGyro_LSBs = readG(OUT_Z_L_G);
	// 16 bit resolution, left-justified
	int16_t zBitGyro = (uint16_t) zGyro_MSBs << 8 | zGyro_LSBs;

	return zBitGyro * gyroGain;
}

/////////////////////////
// Calibration Methods //
/////////////////////////

// Calibration method for accelerometer bias
// Use this calibration protocol (pg 3): http://kionixfs.kionix.com/en/document/AN012%20Accelerometer%20Errors.pdf
void LSM9DS0::calibrateAccelOffsets() {
	int n = 0;

	float ax1 = 0.0f;
	float ax3 = 0.0f;
	float ax5 = 0.0f;
	float ax6 = 0.0f;

	float ay2 = 0.0f;
	float ay4 = 0.0f;
	float ay5 = 0.0f;
	float ay6 = 0.0f;

	float az1 = 0.0f;
	float az2 = 0.0f;
	float az3 = 0.0f;
	float az4 = 0.0f;

	cout << "*** ACCELEROMETER CALIBRATION PROTOCOL STARTED ***\n";
	cout << "There are going to be six positions to orient the sensor in. It must be still while calibrating.\n";
	cout << "It may help to have a corner so you are as close to perpendicular as possible.\n\n";

	cout << "Position 1 is like such:\n";
	cout << "-------\n";
	cout << "|.    |\n";
	cout << "|     |\n";
	cout << "|     |\n";
	cout << "-------\n";
	cout << "Press ENTER to continue.\n";
	cin.ignore();
	cout << "Gathering data. Please wait...\n\n";

	while(n < ACCEL_CALIB_SAMPLES) {
		n += 1;
		ax1 += getxAccel();
		az1 += getzAccel();
	}

	n = 0;
	ax1 /= ACCEL_CALIB_SAMPLES;
	az1 /= ACCEL_CALIB_SAMPLES;


	// POSITION 2 //
	cout << "Position 2 is like such:\n";
	cout << "|----------|\n";
	cout << "|          |\n";
	cout << "|.         |\n";
	cout << "|----------|\n";
	cout << "Press ENTER to continue.\n";
	cin.ignore();
	cout << "Gathering data. Please wait...\n\n";

	while(n < ACCEL_CALIB_SAMPLES) {
		n += 1;
		ay2 += getyAccel();
		az2 += getzAccel();
	}

	n = 0;
	ay2 /= ACCEL_CALIB_SAMPLES;
	az2 /= ACCEL_CALIB_SAMPLES;


	// POSITION 3 //
	cout << "Position 3 is like such:\n";
	cout << "-------\n";
	cout << "|     |\n";
	cout << "|     |\n";
	cout << "|    .|\n";
	cout << "-------\n";
	cout << "Press ENTER to continue.\n";
	cin.ignore();
	cout << "Gathering data. Please wait...\n\n";

	while(n < ACCEL_CALIB_SAMPLES) {
		n += 1;
		ax3 += getxAccel();
		az3 += getzAccel();
	}

	n = 0;
	ax3 /= ACCEL_CALIB_SAMPLES;
	az3 /= ACCEL_CALIB_SAMPLES;


	// POSITION 4 //
	cout << "Position 4 is like such:\n";
	cout << "|----------|\n";
	cout << "|         .|\n";
	cout << "|          |\n";
	cout << "|----------|\n";
	cout << "Press ENTER to continue.\n";
	cin.ignore();
	cout << "Gathering data. Please wait...\n\n";

	while(n < ACCEL_CALIB_SAMPLES) {
		n += 1;
		ay4 += getyAccel();
		az4 += getzAccel();
	}

	n = 0;
	ay4 /= ACCEL_CALIB_SAMPLES;
	az4 /= ACCEL_CALIB_SAMPLES;

	// POSITION 5 //
	cout << "Position 5 is like such:\n";
	cout << "    TOP    \n";
	cout << "-.---------\n";
	cout << "|         |\n";
	cout << "-----------\n";
	cout << "   BOTTOM  \n";
	cout << "Press ENTER to continue.\n";
	cin.ignore();
	cout << "Gathering data. Please wait...\n\n";

	while(n < ACCEL_CALIB_SAMPLES) {
		n += 1;
		ax5 += getxAccel();
		ay5 += getyAccel();
	}

	n = 0;
	ax5 /= ACCEL_CALIB_SAMPLES;
	ay5 /= ACCEL_CALIB_SAMPLES;


	// POSITION 6 //
	cout << "Position 6 is like such:\n";
	cout << "   BOTTOM  \n";
	cout << "-----------\n";
	cout << "|         |\n";
	cout << "-.---------\n";
	cout << "    TOP    \n";
	cout << "Press ENTER to continue.\n";
	cin.ignore();
	cout << "Gathering data. Please wait...\n\n";

	while(n < ACCEL_CALIB_SAMPLES) {
		n += 1;
		ax6 += getxAccel();
		ay6 += getyAccel();
	}

	n = 0;
	ax6 /= ACCEL_CALIB_SAMPLES;
	ay6 /= ACCEL_CALIB_SAMPLES;


	// Calculating 0g biases
	X_AB_OFFSET = (ax1 + ax3 + ax5 + ax6) / 4.0f;
	Y_AB_OFFSET = (ay2 + ay4 + ay5 + ay6) / 4.0f;
	Z_AB_OFFSET = (az1 + az2 + az3 + az4) / 4.0f;

	cout << "Calibration complete!\n";
	cout << "We've already set these values for you in the system, but the offsets are printed for your convenience.\n\n";

	cout << "X Offset: " << X_AB_OFFSET << "\n";
	cout << "Y Offset: " << Y_AB_OFFSET << "\n";
	cout << "Z Offset: " << Z_AB_OFFSET << "\n\n";
}

// Calibration method for hard and soft iron effects
void LSM9DS0::calibrateHardSoftIronEffect() {
	float xmax = 0.0f;
	float xmin = 0.0f;
	float ymax = 0.0f;
	float ymin = 0.0f;
	float zmax = 0.0f;
	float zmin = 0.0f;

	int n = 0;

	cout << "*** MAGNETOMETER CALIBRATION PROTOCOL STARTED ***\n";
	cout << "Please turn the device through the air in a figure 8 fashion until calibration finishes.\n\n";
	cout << "Press ENTER to continue.\n";
	cin.ignore();
	cout << "Calibrating. Continue turning...\n";

	float xtemp;
	float ytemp;
	float ztemp;

	while (n < MAG_CALIB_SAMPLES) {
		n += 1;

		xtemp = getxMag();
		ytemp = getyMag();
		ztemp = getzMag();

		if(xtemp > xmax) {
			xmax = xtemp;
		} else if(xtemp < xmin) {
			xmin = xtemp;
		}

		if(ytemp > ymax) {
			ymax = ytemp;
		} else if(ytemp < ymin) {
			ymin = ytemp;
		}

		if(ztemp > zmax) {
			zmax = ztemp;
		} else if(ztemp < zmin) {
			zmin = ztemp;
		}
	}

	// Hard Iron Offsets
	float xAvg = (xmax + xmin) / 2.0f;
	float yAvg = (xmax + xmin) / 2.0f;
	float zAvg = (xmax + xmin) / 2.0f;

	X_HI_OFFSET = xAvg;
	Y_HI_OFFSET = yAvg;
	Z_HI_OFFSET = zAvg;

	// Soft Iron Scaling
	float allAvg = (xmax - xmin + ymax - ymin + zmax - zmin) / 3.0f;

	X_SI_SCALE = allAvg / (xmax - xmin);
	Y_SI_SCALE = allAvg / (ymax - ymin);
	Z_SI_SCALE = allAvg / (zmax - zmin);

	cout << "Calibration complete!\n";
	cout << "We've already set these values for you in the system, but the offsets are printed for your convenience.\n";

	cout << "X Hard-Iron Offset: " << X_HI_OFFSET << "\n";
	cout << "Y Hard-Iron Offset: " << Y_HI_OFFSET << "\n";
	cout << "Z Hard-Iron Offset: " << Z_HI_OFFSET << "\n\n";

	cout << "X Soft-Iron Scale: " << X_SI_SCALE << "\n";
	cout << "Y Soft-Iron Scale: " << Y_SI_SCALE << "\n";
	cout << "Z Soft-Iron Scale: " << Z_SI_SCALE << "\n\n";
}

// Calibration method for gyro offsets
void LSM9DS0::calibrateGyroOffsets() {
	int n = 0;

	float sumx = 0.0f;
	float sumy = 0.0f;
	float sumz = 0.0f;

	cout << "\n*** GYROSCOPE CALIBRATION PROTOCOL STARTED ***\n";
	cout << "Please keep the device still for calibration. Press ENTER to continue.\n";
	cin.ignore();
	cout << "Calibrating. Please Wait...\n"; 

	while (n < GYRO_CALIB_SAMPLES) {
		n += 1;
		sumx += getxGyro();
		sumy += getyGyro();
		sumz += getzGyro();
	}

	X_GB_OFFSET = sumx / GYRO_CALIB_SAMPLES;
	Y_GB_OFFSET = sumy / GYRO_CALIB_SAMPLES;
	Z_GB_OFFSET = sumz / GYRO_CALIB_SAMPLES;

	cout << "Calibration complete!\n";
	cout << "We've already set these values for you in the system, but the offsets are printed for your convenience.\n";

	cout << "X Offset: " << X_GB_OFFSET << "\n";
	cout << "Y Offset: " << Y_GB_OFFSET << "\n";
	cout << "Z Offset: " << Z_GB_OFFSET << "\n";
}

// Debugging method for raw sensor values
void LSM9DS0::printRawData() {
	float xacc = getxAccel();
	float yacc = getyAccel();
	float zacc = getzAccel();
	float xmag = getxMag();
	float ymag = getyMag();
	float zmag = getzMag();
	float xgyr = getxGyro();
	float ygyr = getyGyro();
	float zgyr = getzGyro();
	float temp = getTemp();

	cout << "X Accel: " << xacc << "\n";
	cout << "Y Accel: " << yacc << "\n";
	cout << "Z Accel: " << zacc << "\n";
	cout << "X Mag: " << xmag << "\n";
	cout << "Y Mag: " << ymag << "\n";
	cout << "Z Mag: " << zmag << "\n";
	cout << "X Gyro: " << xgyr << "\n";
	cout << "Y Gyro: " << ygyr << "\n";
	cout << "Z Gyro: " << zgyr << "\n";
	cout << "Temp: " << temp << "\n";
}

/////////////////
// I2C Methods //
/////////////////

/*****************************************************************
 * This is the default constructor for the class. It assigns
 * all private variables to default values and calls the openI2C()
 * function to open the default I2C device "/dev/i2c-0".
 *****************************************************************/
I2C8Bit::I2C8Bit(void){
    this->i2cFileName = "/dev/i2c-0";
    this->deviceAddress= 0;
        this->i2cDescriptor = -1;
        cout << " Opening I2C Device" << endl;
        this->openI2C();

}

/*******************************************************************
 * This is the overloaded constructor. It allows the programmer to
 * specify a custom I2C device & device address
 * The device descriptor is determined by the openI2C() private member
 * function call.
 * *****************************************************************/

I2C8Bit::I2C8Bit(unsigned char dev_addr, std::string i2c_file_name){
    this->i2cFileName = i2c_file_name;
    this->deviceAddress = dev_addr;
        this->i2cDescriptor = -1;
        cout << " Opening I2C Device" << endl;
    this->openI2C();
}
/**********************************************************************
 * This is the class destructor it simply closes the open I2C device
 * by calling the closeI2C() which in turn calls the close() system call
 * *********************************************************************/

I2C8Bit::~I2C8Bit(void){
        cout << " Closing I2C Device" << endl;
    this->closeI2C();
}

/**********************************************************************
 * This function opens the I2C device by simply calling the open system
 * call on the I2C device specified in the i2cFileName string. The I2C
 * device is opened for writing and reading. The i2cDescriptor private
 * variable is set by the return value of the open() system call.
 * This variable will be used to reference the opened I2C device by the
 * ioctl() & close() system calls.
 * ********************************************************************/

int I2C8Bit::openI2C(){
    this->i2cDescriptor = open(i2cFileName.c_str(), O_RDWR);
    if(this->i2cDescriptor < 0){
        perror("Could not open file (1)");
        exit(1);
    }

    return i2cDescriptor;
}

/*********************************************************************
 * This function closes the I2C device by calling the close() system call
 * on the I2C device descriptor.
 * *******************************************************************/

int I2C8Bit::closeI2C(){
                int retVal = -1;
        retVal = close(this->i2cDescriptor);
    if(retVal < 0){
        perror("Could not close file (1)");
        exit(1);
    }
return retVal;
}
/********************************************************************
 *This function writes a byte of data "data" to a specific register
 *"reg_addr" in the I2C device This involves sending these two bytes
 *in order to the i2C device by means of the ioctl() command. Since
 *both bytes are written (no read/write switch), both pieces
 *of information can be sent in a single message (i2c_msg structure)
 ********************************************************************/
int I2C8Bit::writeReg(unsigned char reg_addr, unsigned char data){

    unsigned char buff[2];
    int retVal = -1;
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[1];

    buff[0] = reg_addr;
    buff[1] = data;

    messages[0].addr = deviceAddress;
    messages[0].flags = 0;
    messages[0].len = sizeof(buff);
    messages[0].buf = buff;

    packets.msgs = messages;
    packets.nmsgs = 1;

    retVal = ioctl(this->i2cDescriptor, I2C_RDWR, &packets);
    if(retVal < 0)
        perror("Write to I2C Device failed");

    return retVal;
}

/********************************************************************
 *This function reads a byte of data "data" from a specific register
 *"reg_addr" in the I2C device. This involves sending the register address
 *byte "reg_Addr" with "write" asserted and then instructing the
 *I2C device to read a byte of data from that address ("read asserted").
 *This necessitates the use of two i2c_msg structs. One for the register
 *address write and another for the read from the I2C device i.e.
 *I2C_M_RD flag is set. The read data is then saved into the reference
 *variable "data".
 ********************************************************************/

int I2C8Bit::readReg(unsigned char reg_addr, unsigned char &data){

    unsigned char *inbuff, outbuff;
    int retVal = -1;
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[2];

    outbuff = reg_addr;
    messages[0].addr = deviceAddress;
    messages[0].flags= 0;
    messages[0].len = sizeof(outbuff);
    messages[0].buf = &outbuff;

    inbuff = &data;
    messages[1].addr = deviceAddress;
    messages[1].flags = I2C_M_RD;
    messages[1].len = sizeof(*inbuff);
    messages[1].buf = inbuff;

    packets.msgs = messages;
    packets.nmsgs = 2;

    retVal = ioctl(this->i2cDescriptor, I2C_RDWR, &packets);
    if(retVal < 0)
        perror("Read from I2C Device failed");

    return retVal;
}

/////////////////////////////
// Madgwick Filter Methods //
/////////////////////////////

void LSM9DS0::startLSM() {
	prevTime = std::chrono::steady_clock::now();
	ax = getxAccel() - X_AB_OFFSET;
	ay = getyAccel() - Y_AB_OFFSET;
	az = -(getzAccel() - Z_AB_OFFSET);
	mx = (getxMag() - X_HI_OFFSET) * X_SI_SCALE;
	mx = (getyMag() - Y_HI_OFFSET) * Y_SI_SCALE;
	mx = (getzMag() - Z_HI_OFFSET) * Z_SI_SCALE;
	wx = getxGyro() - X_GB_OFFSET;
	wx = getyGyro() - Y_GB_OFFSET;
	wx = getzGyro() - Z_GB_OFFSET;

	startTime = prevTime;
	lastPrintTime = prevTime;

	madgwickFilterUpdate(); // [TODO] Make this run on a different thread!
}

void LSM9DS0::madgwickFilterUpdate() {

}

// -------------------- //

int main() {
	LSM9DS0 lsm = LSM9DS0();
	lsm.initialize();

	lsm.printRawData();

	lsm.calibrateGyroOffsets();
	lsm.calibrateHardSoftIronEffect();
	lsm.calibrateAccelOffsets();
}
