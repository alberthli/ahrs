#include "LSM9DS0.h"
#include <Wire.h>
#include <Arduino.h>

LSM9DS0::LSM9DS0() {

}

LSM9DS0::~LSM9DS0() {

}

void LSM9DS0::start() {
  Wire.begin();

  initXM(XM_ADDRESS);
  initG(G_ADDRESS);
}

// Initializing desired settings on the XM
void LSM9DS0::initXM(uint8_t xmaddress) {
	// Bit Register Configuration Info in Header File
	writeByte(xmaddress, CTRL_REG0_XM, 0x00); // All disabled, defaults
	writeByte(xmaddress, CTRL_REG1_XM, 0x57); // 100 Hz Accel. Sampling Rate, Continuous Update, All Axes Enabled
	writeByte(xmaddress, CTRL_REG2_XM, 0x08); // 773 Hz AAFB, +/- 4g, Normal Self-Test, 4 Wire Interface
	writeByte(xmaddress, CTRL_REG3_XM, 0x00); // Disable interrupts for now
	writeByte(xmaddress, CTRL_REG4_XM, 0x00);
	writeByte(xmaddress, CTRL_REG5_XM, 0xF4); // Temperature Enabled, High Mag Res, 100 Hz Sampling, No Latched Ints
	writeByte(xmaddress, CTRL_REG6_XM, 0x00); // +/- 2 Gauss Scale
	writeByte(xmaddress, CTRL_REG7_XM, 0x00); // Defaults

	writeByte(xmaddress, FIFO_CTRL_REG, 0x00); // Defaults
	// These values need to be changed if you change the operating range of the sensors!
	accelGain = 0.000122;
	magGain = 0.00008;
}

// Initializing desired settings on the G
void LSM9DS0::initG(uint8_t gaddress) {
	// Bit Register Configuration Info in Header File
	writeByte(gaddress, CTRL_REG_1_G, 0x0F); // Default ODR and Bandwidth, Normal Mode, All Axes Enabled
	writeByte(gaddress, CTRL_REG_2_G, 0x00); // Normal Mode, 7.2 Hz HPF Cutoff Frequency
	writeByte(gaddress, CTRL_REG_3_G, 0x00); // Defaults
	writeByte(gaddress, CTRL_REG_4_G, 0x00); // Defaults, Gyro Scale +/- 245 DPS
	writeByte(gaddress, CTRL_REG_5_G, 0x00); // Defaults

	// This value needs to be changed if you change the operating range of the gyro!
	gyroGain = 0.00875;
}

// Reads a byte from device with devAddress from the regAddress register
uint8_t LSM9DS0::readByte(uint8_t devAddress, uint8_t regAddress) {
	Wire.beginTransmission(devAddress);
	Wire.write(regAddress);
	Wire.endTransmission();
	Wire.requestFrom(devAddress, (uint8_t) 1);
	while(Wire.available() < 1);

	uint8_t read = Wire.read();
	Wire.endTransmission();
	return read;
}

// Writes a byte to device with devAddress to the regAddress register
void LSM9DS0::writeByte(uint8_t devAddress, uint8_t regAddress, uint8_t byte) {
	Wire.beginTransmission(devAddress);
	Wire.write(regAddress);
	Wire.write(byte);
	Wire.endTransmission();
}

// Retrieves the temperature (deg C)
double LSM9DS0::getTemp() {
	uint8_t temp_MSBs = readByte(XM_ADDRESS, OUT_TEMP_H_XM);
	uint8_t temp_LSBs = readByte(XM_ADDRESS, OUT_TEMP_L_XM);
	// 12 bit resolution, right-justified
	int16_t bitTemp = ( ((uint16_t) temp_MSBs) << 8 | temp_LSBs) & 0x0FFF;

	if(bitTemp > 2047) {
		bitTemp -= 4096;
	}

	return TEMP_INTERCEPT + (double)bitTemp * TEMP_GAIN;
}

// Retrives the x acceleration (m*s^-2)
double LSM9DS0::getxAccel() {
	uint8_t xAccel_MSBs = readByte(XM_ADDRESS, OUT_X_H_A);
	uint8_t xAccel_LSBs = readByte(XM_ADDRESS, OUT_X_L_A);
	// 16 bit resolution, left-justified
	int16_t xBitAccel = (uint16_t) xAccel_MSBs << 8 | xAccel_LSBs;

	return xBitAccel * accelGain * GRAV_ACCEL;
}

// Retrives the y acceleration (m*s^-2)
double LSM9DS0::getyAccel() {
	uint8_t yAccel_MSBs = readByte(XM_ADDRESS, OUT_Y_H_A);
	uint8_t yAccel_LSBs = readByte(XM_ADDRESS, OUT_Y_L_A);
	// 16 bit resolution, left-justified
	int16_t yBitAccel = (uint16_t) yAccel_MSBs << 8 | yAccel_LSBs;
  
	return yBitAccel * accelGain * GRAV_ACCEL;
}

// Retrives the z acceleration (m*s^-2)
double LSM9DS0::getzAccel() {
	uint8_t zAccel_MSBs = readByte(XM_ADDRESS, OUT_Z_H_A);
	uint8_t zAccel_LSBs = readByte(XM_ADDRESS, OUT_Z_L_A);
	// 16 bit resolution, left-justified
	int16_t zBitAccel = (uint16_t) zAccel_MSBs << 8 | zAccel_LSBs;

	return zBitAccel * accelGain * GRAV_ACCEL;
}

// Retrieves the x magnetic field value (gauss)
double LSM9DS0::getxMag() {
	uint8_t xMag_MSBs = readByte(XM_ADDRESS, OUT_X_H_M);
	uint8_t xMag_LSBs = readByte(XM_ADDRESS, OUT_X_L_M);
	// 16 bit resolution, left-justified
	int16_t xBitMag = (uint16_t) xMag_MSBs << 8 | xMag_LSBs;

	return xBitMag * magGain;
}

// Retrieves the y magnetic field value (gauss)
double LSM9DS0::getyMag() {
	uint8_t yMag_MSBs = readByte(XM_ADDRESS, OUT_Y_H_M);
	uint8_t yMag_LSBs = readByte(XM_ADDRESS, OUT_Y_L_M);
	// 16 bit resolution, left-justified
	int16_t yBitMag = (uint16_t) yMag_MSBs << 8 | yMag_LSBs;

	return yBitMag * magGain;
}

// Retrieves the z magnetic field value (gauss)
double LSM9DS0::getzMag() {
	uint8_t zMag_MSBs = readByte(XM_ADDRESS, OUT_Z_H_M);
	uint8_t zMag_LSBs = readByte(XM_ADDRESS, OUT_Z_L_M);
	// 16 bit resolution, left-justified
	int16_t zBitMag = (uint16_t) zMag_MSBs << 8 | zMag_LSBs;

	return zBitMag * magGain;
}

// Retrieves the x gyro value (DPS)
double LSM9DS0::getxGyro() {
	uint8_t xGyro_MSBs = readByte(G_ADDRESS, OUT_X_H_G);
	uint8_t xGyro_LSBs = readByte(G_ADDRESS, OUT_X_L_G);
	// 16 bit resolution, left-justified
	int16_t xBitGyro = (uint16_t) xGyro_MSBs << 8 | xGyro_LSBs;

	return xBitGyro * gyroGain;
}

// Retrieves the y gyro value (DPS)
double LSM9DS0::getyGyro() {
	uint8_t yGyro_MSBs = readByte(G_ADDRESS, OUT_Y_H_G);
	uint8_t yGyro_LSBs = readByte(G_ADDRESS, OUT_Y_L_G);
	// 16 bit resolution, left-justified
	int16_t yBitGyro = (uint16_t) yGyro_MSBs << 8 | yGyro_LSBs;

	return yBitGyro * gyroGain;
}

// Retrieves the z gyro value (DPS)
double LSM9DS0::getzGyro() {
	uint8_t zGyro_MSBs = readByte(G_ADDRESS, OUT_Z_H_G);
	uint8_t zGyro_LSBs = readByte(G_ADDRESS, OUT_Z_L_G);
	// 16 bit resolution, left-justified
	int16_t zBitGyro = (uint16_t) zGyro_MSBs << 8 | zGyro_LSBs;

	return zBitGyro * gyroGain;
}

// Calibration method for gyro offsets
void LSM9DS0::calibrateGyroOffsets() {
	int n = 0;

	double sumx = 0.0;
	double sumy = 0.0;
	double sumz = 0.0;

	Serial.println("*** GYROSCOPE CALIBRATION PROTOCOL STARTED ***");
	Serial.println("Please keep the device still for calibration. When ready, send Y. To exit, send anything else.");
	while(Serial.available() == 0) {
		char read = Serial.read();

		if(read == 'y' || read == 'Y') {
			break;
			
		} else {
			Serial.println("Calibration exited.");
			return;
		}
	}
	Serial.println("Calibrating. Please wait...");

	while (n < GYRO_CALIB_SAMPLES) {
		n += 1;
		sumx += getxGyro();
		sumy += getxGyro();
		sumz += getxGyro();
	}

	X_GB_OFFSET = sumx / GYRO_CALIB_SAMPLES;
	Y_GB_OFFSET = sumy / GYRO_CALIB_SAMPLES;
	Z_GB_OFFSET = sumz / GYRO_CALIB_SAMPLES;

	Serial.println("Calibration complete!");
	Serial.println("We've already set these values for you in the system, but the offsets are printed for your convenience.\n");

	Serial.print("X Offset: "); Serial.println(X_GB_OFFSET);
	Serial.print("Y Offset: "); Serial.println(Y_GB_OFFSET);
	Serial.print("Z Offset: "); Serial.println(Z_GB_OFFSET);
	Serial.println("\n");
}

// Debugging method for raw sensor values
void LSM9DS0::printRawData() {
	double xacc = getxAccel();
	double yacc = getyAccel();
	double zacc = getzAccel();
	double xmag = getxMag();
	double ymag = getyMag();
	double zmag = getzMag();
	double xgyr = getxGyro();
	double ygyr = getyGyro();
	double zgyr = getzGyro();
	double temp = getTemp();

	Serial.print("X Accel: "); Serial.println(xacc);
	Serial.print("Y Accel: "); Serial.println(yacc);
	Serial.print("Z Accel: "); Serial.println(zacc);
	Serial.print("X Mag: "); Serial.println(xmag);
	Serial.print("Y Mag: "); Serial.println(ymag);
	Serial.print("Z Mag: "); Serial.println(zmag);
	Serial.print("X Gyro: "); Serial.println(xgyr);
	Serial.print("Y Gyro: "); Serial.println(ygyr);
	Serial.print("Z Gyro: "); Serial.println(zgyr);
	Serial.print("Temp: "); Serial.println(temp);
	Serial.println();
}
