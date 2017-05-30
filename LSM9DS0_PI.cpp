#include <iostream>
#include <cmath>
#include <arm_neon.h>
#include <thread>

#include "LSM9DS0_PI.h"

using namespace std;

LSM9DS0::LSM9DS0() {
	// Sensor Default Offsets
	X_HI_OFFSET = -0.06f;
	Y_HI_OFFSET = -0.06f;
	Z_HI_OFFSET = 0.08f;

	X_SI_SCALE = 1.0f;
	Y_SI_SCALE = 0.95f;
	Z_SI_SCALE = 1.1f;

	X_GB_OFFSET = 0.5f;
	Y_GB_OFFSET = -0.3f;
	Z_GB_OFFSET = -4.5f;

	X_AB_OFFSET = -0.4f;
	Y_AB_OFFSET = -0.016f;
	Z_AB_OFFSET = -0.23f;

	// Madgwick Variables Initialization
	BETA = 12.5f;
	ZETA = 0.01f;
	bx = 1.0f;
	bz = 0.0f;

	// Euler Angles
	roll = 0.0f;
	pitch = 0.0f;
	yaw = 0.0f;
}

LSM9DS0::~LSM9DS0() {

}

////////////////////////////
// INITIALIZATION METHODS //
////////////////////////////

void LSM9DS0::initialize() {
	initXM();
	initG();
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
	accelGain = 0.000122f;
	magGain = 0.00008f;
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
	gyroGain = 0.00875f;
}

/////////////////
// I2C Methods //
/////////////////

// Reads data from the XM device and returns it as an unsigned int
uint8_t LSM9DS0::readXM(uint8_t reg_address) {
	unsigned char data = 0;
	I2CInterface.readRegister(XM_ADDRESS, reg_address, &data, 1);
	return (uint8_t)data;
}

// Reads data from the G device and returns it as an unsigned int
uint8_t LSM9DS0::readG(uint8_t reg_address) {
	unsigned char data = 0;
	I2CInterface.readRegister(G_ADDRESS, reg_address, &data, 1);
	return (uint8_t)data;
}

// Writes data to the XM device
void LSM9DS0::writeXM(uint8_t reg_address, uint8_t data) {
	I2CInterface.writeRegister(XM_ADDRESS, reg_address, &data, 1);
}

// Writes data to the G device
void LSM9DS0::writeG(uint8_t reg_address, uint8_t data) {
	I2CInterface.writeRegister(G_ADDRESS, reg_address, &data, 1);
}

////////////////////////////////////////
// Sensor Update Methods (Calibrated) //
////////////////////////////////////////

// Retrieves the temperature (deg C)
void LSM9DS0::updateTemp() {
	uint8_t readBuffer[2];
	I2CInterface.readRegister(XM_ADDRESS, OUT_TEMP_L_XM, readBuffer, 2);
	int16_t bitTemp = ((uint16_t)readBuffer[1] << 8 | readBuffer[0]) & 0x0FFF;

	if(bitTemp > 2047) {
		bitTemp -= 4096;
	}

	temperature = TEMP_INTERCEPT + (float)bitTemp * TEMP_GAIN;
}

// Updates the sensor accelerations
void LSM9DS0::updateAccel() {
	uint8_t readBuffer[6];
	I2CInterface.readRegister(XM_ADDRESS, OUT_X_L_A, readBuffer, 6);
	
	ax = static_cast<uint16_t>(readBuffer[1] << 8 | readBuffer[0]) * accelGain * GRAV_ACCEL - X_AB_OFFSET;
	ay = static_cast<uint16_t>(readBuffer[3] << 8 | readBuffer[2]) * accelGain * GRAV_ACCEL - Y_AB_OFFSET;
	az = -(static_cast<uint16_t>(readBuffer[5] << 8 | readBuffer[4]) * accelGain * GRAV_ACCEL - Z_AB_OFFSET);
}

// Updates the magnetometer values
void LSM9DS0::updateMag() {
	uint8_t readBuffer[6];
	I2CInterface.readRegister(XM_ADDRESS, OUT_X_L_M, readBuffer, 6);
	
	mx = (static_cast<uint16_t>(readBuffer[1] << 8 | readBuffer[0]) * magGain - X_HI_OFFSET) * X_SI_SCALE;
	my = (static_cast<uint16_t>(readBuffer[3] << 8 | readBuffer[2]) * magGain - Y_HI_OFFSET) * Y_SI_SCALE;
	mz = (static_cast<uint16_t>(readBuffer[5] << 8 | readBuffer[4]) * magGain - Z_HI_OFFSET) * Z_SI_SCALE;
}

// Updates the gyro values
void LSM9DS0::updateGyro() {
	uint8_t readBuffer[6];
	I2CInterface.readRegister(G_ADDRESS, OUT_X_L_G, readBuffer, 6);
	
	wx = (static_cast<uint16_t>(readBuffer[1]) << 8 | readBuffer[0]) * gyroGain - X_GB_OFFSET;
	wy = (static_cast<uint16_t>(readBuffer[3]) << 8 | readBuffer[2]) * gyroGain - Y_GB_OFFSET;
	wz = (static_cast<uint16_t>(readBuffer[5]) << 8 | readBuffer[4]) * gyroGain - Z_GB_OFFSET;
}

void LSM9DS0::printUpdateData() {
	updateAccel();
	updateMag();
	updateGyro();

	printf("%f\n", ax);
	printf("%f\n", ay);
	printf("%f\n", az);
	printf("%f\n", mx);
	printf("%f\n", my);
	printf("%f\n", mz);
	printf("%f\n", wx);
	printf("%f\n", wy);
	printf("%f\n\n", wz);
}

////////////////////////////////////
// Sensor Retrieval Methods (Raw) //
////////////////////////////////////

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
	cout << "Temp: " << temp << "\n\n";
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
	float yAvg = (ymax + ymin) / 2.0f;
	float zAvg = (zmax + zmin) / 2.0f;

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
	cout << "Z Offset: " << Z_GB_OFFSET << "\n\n";
}

/////////////////////////////
// Madgwick Filter Methods //
/////////////////////////////

void LSM9DS0::startLSM() {
	currTime = std::chrono::steady_clock::now();
	prevTime = std::chrono::steady_clock::now();

	updateAccel();
	updateMag();
	updateGyro();

	/*
	ax = getxAccel() - X_AB_OFFSET;
	ay = getyAccel() - Y_AB_OFFSET;
	az = -(getzAccel() - Z_AB_OFFSET);
	mx = (getxMag() - X_HI_OFFSET) * X_SI_SCALE;
	my = (getyMag() - Y_HI_OFFSET) * Y_SI_SCALE;
	mz = (getzMag() - Z_HI_OFFSET) * Z_SI_SCALE;
	wx = getxGyro() - X_GB_OFFSET;
	wy = getyGyro() - Y_GB_OFFSET;
	wz = getzGyro() - Z_GB_OFFSET;
	*/

	startTime = prevTime;
	lastPrintTime = prevTime;

	madgwickFilterUpdate();
}

void LSM9DS0::madgwickFilterUpdate() {

	while(true) {
		currTime = std::chrono::steady_clock::now();
		dt = std::chrono::duration_cast<std::chrono::microseconds>(currTime - prevTime).count() / 1000000.0f;
		prevTime = currTime;

		// Poll new values
		updateAccel();
		updateMag();
		updateGyro();

		/*********************************/
		/* Useful Variable Manipulations */
		/*********************************/
		// float32x4_t hSEq_vec = {0.5f * SEq[0], 0.5f * SEq[1], 0.5f * SEq[2], 0.5f * SEq[3]};
		// float32x4_t dSEq_vec = {2.0f * SEq[0], 2.0f * SEq[1], 2.0f * SEq[2], 2.0f * SEq[3]};

		float hSEq0 = 0.5f * SEq[0];
		float hSEq1 = 0.5f * SEq[1];
		float hSEq2 = 0.5f * SEq[2];
		float hSEq3 = 0.5f * SEq[3];
		
		float dSEq0 = 2.0f * SEq[0];
		float dSEq1 = 2.0f * SEq[1];
		float dSEq2 = 2.0f * SEq[2];
		float dSEq3 = 2.0f * SEq[3];

		float sSEq2 = SEq[2] * SEq[2];

		float dbx = 2.0f * bx;
		float dbz = 2.0f * bz;

		float dbxSEq0 = dbx * SEq[0];
		float dbxSEq1 = dbx * SEq[1];
		float dbxSEq2 = dbx * SEq[2];
		float dbxSEq3 = dbx * SEq[3];
		float dbzSEq0 = dbz * SEq[0];
		float dbzSEq1 = dbz * SEq[1];
		float dbzSEq2 = dbz * SEq[2];
		float dbzSEq3 = dbz * SEq[3];

		float SEq0SEq2 = SEq[0] * SEq[2];
		float SEq1SEq3 = SEq[1] * SEq[3];

		/**************************/
		/* Beginning of Algorithm */
		/**************************/

		// Normalize acceleration and magnetometer values
		float sqrtOf = ax * ax + ay * ay + az * az;
		float tempNorm = invSqrt(sqrtOf);
		ax *= tempNorm;
		ay *= tempNorm;
		az *= tempNorm;

		sqrtOf = mx * mx + my * my + mz * mz;
		tempNorm = invSqrt(sqrtOf);
		mx *= tempNorm;
		my *= tempNorm;
		mz *= tempNorm;

		float dmx = 2 * mx;
		float dmy = 2 * my;
		float dmz = 2 * mz;

		// Combined cost function + Jacobian
		// Functions from g-field
		float f1 = dSEq1 * SEq[3] - dSEq0 * SEq[2] - ax;
		float f2 = dSEq0 * SEq[1] + dSEq2 * SEq[3] - ay;
		float f3 = 1.0f - dSEq1 * SEq[1] - dSEq2 * SEq[2] - az;

		// Functions from b-field
		float f4 = dbx * (0.5f - sSEq2 - SEq[3] * SEq[3]) + dbz * (SEq1SEq3 - SEq0SEq2) - mx;
		float f5 = dbx * (SEq[1] * SEq[2] - SEq[0] * SEq[3]) + dbz * (SEq[0] * SEq[1] + SEq[2] * SEq[3]) - my;
		float f6 = dbx * (SEq0SEq2 + SEq1SEq3) + dbz * (0.5f - SEq[1] * SEq[1] - sSEq2) - mz;

		// Jacobian entries
		float J1124 = dSEq2;
		float J1223 = dSEq3;
		float J1322 = dSEq0;
		float J1421 = dSEq1;
		float J32 = 2.0f * J1421;
		float J33 = 2.0f * J1124;
		float J41 = dbzSEq2;
		float J42 = dbzSEq3;
		float J43 = 2.0f * dbxSEq2 + dbzSEq0;
		float J44 = 2.0f * dbxSEq3 - dbzSEq1;
		float J51 = dbxSEq3 - dbzSEq1;
		float J52 = dbxSEq2 + dbzSEq0;
		float J53 = dbxSEq1 + dbzSEq3;
		float J54 = dbxSEq0 - dbzSEq2;
		float J61 = dbxSEq2;
		float J62 = dbxSEq3 - 2.0f * dbzSEq1;
		float J63 = dbxSEq0 - 2.0f * dbzSEq2;
		float J64 = dbxSEq1;

		// Gradient Descent Optimization
		// Gradients
		float SEqhatdot0 = -J1124 * f1 + J1421 * f2 - J41 * f4 - J51 * f5 + J61 * f6;
		float SEqhatdot1 = J1223 * f1 + J1322 * f2 - J32 * f3 + J42 * f4 + J52 * f5 + J62 * f6;
		float SEqhatdot2 = -J1322 * f1 + J1223 * f2 - J33 * f3 - J43 * f4 + J53 * f5 + J63 * f6;
		float SEqhatdot3 = J1421 * f1 + J1124 * f2 - J44 * f4 - J54 * f5 + J64 * f6;

		// Normalizing Gradients
		sqrtOf = SEqhatdot0 * SEqhatdot0 + SEqhatdot1 * SEqhatdot1 + SEqhatdot2 * SEqhatdot2 + SEqhatdot3 * SEqhatdot3;
		tempNorm = invSqrt(sqrtOf);
		SEqhatdot0 *= tempNorm;
		SEqhatdot1 *= tempNorm;
		SEqhatdot2 *= tempNorm;
		SEqhatdot3 *= tempNorm;

		// Angular estimated direction of gyro error
		float wex = dSEq0 * SEqhatdot1 - dSEq1 * SEqhatdot0 - dSEq2 * SEqhatdot3 + dSEq3 * SEqhatdot2;
		float wey = dSEq0 * SEqhatdot2 + dSEq1 * SEqhatdot3 - dSEq2 * SEqhatdot0 - dSEq3 * SEqhatdot1;
		float wez = dSEq0 * SEqhatdot3 - dSEq1 * SEqhatdot2 + dSEq2 * SEqhatdot1 - dSEq3 * SEqhatdot0;

		// Remove gyro bias
		gyroBiases[0] += wex * dt * ZETA;
		gyroBiases[1] += wey * dt * ZETA;
		gyroBiases[2] += wez * dt * ZETA;
		wx -= gyroBiases[0];
		wy -= gyroBiases[1];
		wz -= gyroBiases[2];

		// Quaternion rate of change (gyro)
		float SEqdot0 = -hSEq1 * wx - hSEq2 * wy - hSEq3 * wz;
		float SEqdot1 = hSEq0 * wx + hSEq2 * wz - hSEq3 * wy;
		float SEqdot2 = hSEq0 * wy - hSEq1 * wz + hSEq3 * wx;
		float SEqdot3 = hSEq0 * wz + hSEq1 * wy - hSEq2 * wx;

		// Update orientation quaternion
		SEq[0] += (SEqdot0 - (BETA * SEqhatdot0)) * dt;
		SEq[1] += (SEqdot1 - (BETA * SEqhatdot1)) * dt;
		SEq[2] += (SEqdot2 - (BETA * SEqhatdot2)) * dt;
		SEq[3] += (SEqdot3 - (BETA * SEqhatdot3)) * dt;

		// Normalize orientation quaternion
		sqrtOf = SEq[0] * SEq[0] + SEq[1] * SEq[1] + SEq[2] * SEq[2] + SEq[3] * SEq[3];
		tempNorm = invSqrt(sqrtOf);
		SEq[0] *= tempNorm;
		SEq[1] *= tempNorm;
		SEq[2] *= tempNorm;
		SEq[3] *= tempNorm;

		// b-field in earth frame
		float SEq0SEq1 = SEq[0] * SEq[1];
		SEq0SEq2 = SEq[0] * SEq[2];
		float SEq0SEq3 = SEq[0] * SEq[3];
		float SEq2SEq3 = SEq[2] * SEq[3];
		float SEq1SEq2 = SEq[1] * SEq[2];
		SEq1SEq3 = SEq[1] * SEq[3];

		float hx = dmx * (0.5f - SEq[2] * SEq[2] - SEq[3] * SEq[3]) + dmy * (SEq1SEq2 - SEq0SEq3) + dmz * (SEq1SEq3 + SEq0SEq2);
		float hy = dmx * (SEq1SEq2 + SEq0SEq3) + dmy * (0.5f - SEq[1] * SEq[1] - SEq[3] * SEq[3]) + dmz * (SEq2SEq3 - SEq0SEq1);
		float hz = dmx * (SEq1SEq3 - SEq0SEq2) + dmy * (SEq2SEq3 + SEq0SEq1) + dmz * (0.5f - SEq[1] * SEq[1] - SEq[2] * SEq[2]);

		// Normalize flux vector to eliminate y component
		bx = sqrt(hx * hx + hy * hy);
		bz = hz;

		calculateRPY();

		// Debug Print
		printf("dt: %f\n\n", dt);
	}
}

void LSM9DS0::calculateRPY() {
	yaw = atan2(2.0f * (SEq[1] * SEq[2] - SEq[0] * SEq[3]), 2.0f * (SEq[0] * SEq[0] + SEq[1] * SEq[1]) - 1.0f);
	pitch = asin(2.0f * (SEq[0] * SEq[2] - SEq[1] * SEq[3]));
	roll = atan2(2.0f * (SEq[0] * SEq[1] + SEq[2] * SEq[3]), 1.0f - 2.0f * (SEq[1] * SEq[1] + SEq[2] * SEq[2]));

	// Angle conversion
	yaw *= 180 / PI;
	yaw += DECLINATION_ANGLE;
	pitch *= 180 / PI;
	roll = 180 - (roll * 180 / PI);
	if(roll > 180) {
		roll -= 360;
	}

	// Debug prints
	printf("Yaw: %f\n", yaw);
	printf("Pitch: %f\n", pitch);
	printf("Roll: %f\n", roll);
}

/////////////////////
// Math Operations //
/////////////////////

float LSM9DS0::invSqrt(float x) {
   uint32_t i = 0x5F1F1412 - (*(uint32_t*)&x >> 1);
   float tmp = *(float*)&i;
   return tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
}

// DIRECT CALL TO MAIN MEANS WE ARE DEBUGGING. COMMENT OUT OTHERWISE.
int main() {
	LSM9DS0 lsm = LSM9DS0();
	lsm.initialize();

	while(true) {
		lsm.printUpdateData();
	}
	/*
	lsm.calibrateGyroOffsets();
	lsm.calibrateHardSoftIronEffect();
	lsm.calibrateAccelOffsets();
	*/
	
	// lsm.startLSM();
}
