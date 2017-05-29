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
float LSM9DS0::getTemp() {
	uint8_t temp_MSBs = readByte(XM_ADDRESS, OUT_TEMP_H_XM);
	uint8_t temp_LSBs = readByte(XM_ADDRESS, OUT_TEMP_L_XM);
	// 12 bit resolution, right-justified
	int16_t bitTemp = ( ((uint16_t) temp_MSBs) << 8 | temp_LSBs) & 0x0FFF;

	if(bitTemp > 2047) {
		bitTemp -= 4096;
	}

	return TEMP_INTERCEPT + (float)bitTemp * TEMP_GAIN;
}

// Retrives the x acceleration (m*s^-2)
float LSM9DS0::getxAccel() {
	uint8_t xAccel_MSBs = readByte(XM_ADDRESS, OUT_X_H_A);
	uint8_t xAccel_LSBs = readByte(XM_ADDRESS, OUT_X_L_A);
	// 16 bit resolution, left-justified
	int16_t xBitAccel = (uint16_t) xAccel_MSBs << 8 | xAccel_LSBs;

	return xBitAccel * accelGain * GRAV_ACCEL;
}

// Retrives the y acceleration (m*s^-2)
float LSM9DS0::getyAccel() {
	uint8_t yAccel_MSBs = readByte(XM_ADDRESS, OUT_Y_H_A);
	uint8_t yAccel_LSBs = readByte(XM_ADDRESS, OUT_Y_L_A);
	// 16 bit resolution, left-justified
	int16_t yBitAccel = (uint16_t) yAccel_MSBs << 8 | yAccel_LSBs;
  
	return yBitAccel * accelGain * GRAV_ACCEL;
}

// Retrives the z acceleration (m*s^-2)
float LSM9DS0::getzAccel() {
	uint8_t zAccel_MSBs = readByte(XM_ADDRESS, OUT_Z_H_A);
	uint8_t zAccel_LSBs = readByte(XM_ADDRESS, OUT_Z_L_A);
	// 16 bit resolution, left-justified
	int16_t zBitAccel = (uint16_t) zAccel_MSBs << 8 | zAccel_LSBs;

	return zBitAccel * accelGain * GRAV_ACCEL;
}

// Retrieves the x magnetic field value (gauss)
float LSM9DS0::getxMag() {
	uint8_t xMag_MSBs = readByte(XM_ADDRESS, OUT_X_H_M);
	uint8_t xMag_LSBs = readByte(XM_ADDRESS, OUT_X_L_M);
	// 16 bit resolution, left-justified
	int16_t xBitMag = (uint16_t) xMag_MSBs << 8 | xMag_LSBs;

	return xBitMag * magGain;
}

// Retrieves the y magnetic field value (gauss)
float LSM9DS0::getyMag() {
	uint8_t yMag_MSBs = readByte(XM_ADDRESS, OUT_Y_H_M);
	uint8_t yMag_LSBs = readByte(XM_ADDRESS, OUT_Y_L_M);
	// 16 bit resolution, left-justified
	int16_t yBitMag = (uint16_t) yMag_MSBs << 8 | yMag_LSBs;

	return yBitMag * magGain;
}

// Retrieves the z magnetic field value (gauss)
float LSM9DS0::getzMag() {
	uint8_t zMag_MSBs = readByte(XM_ADDRESS, OUT_Z_H_M);
	uint8_t zMag_LSBs = readByte(XM_ADDRESS, OUT_Z_L_M);
	// 16 bit resolution, left-justified
	int16_t zBitMag = (uint16_t) zMag_MSBs << 8 | zMag_LSBs;

	return zBitMag * magGain;
}

// Retrieves the x gyro value (DPS)
float LSM9DS0::getxGyro() {
	uint8_t xGyro_MSBs = readByte(G_ADDRESS, OUT_X_H_G);
	uint8_t xGyro_LSBs = readByte(G_ADDRESS, OUT_X_L_G);
	// 16 bit resolution, left-justified
	int16_t xBitGyro = (uint16_t) xGyro_MSBs << 8 | xGyro_LSBs;

	return xBitGyro * gyroGain;
}

// Retrieves the y gyro value (DPS)
float LSM9DS0::getyGyro() {
	uint8_t yGyro_MSBs = readByte(G_ADDRESS, OUT_Y_H_G);
	uint8_t yGyro_LSBs = readByte(G_ADDRESS, OUT_Y_L_G);
	// 16 bit resolution, left-justified
	int16_t yBitGyro = (uint16_t) yGyro_MSBs << 8 | yGyro_LSBs;

	return yBitGyro * gyroGain;
}

// Retrieves the z gyro value (DPS)
float LSM9DS0::getzGyro() {
	uint8_t zGyro_MSBs = readByte(G_ADDRESS, OUT_Z_H_G);
	uint8_t zGyro_LSBs = readByte(G_ADDRESS, OUT_Z_L_G);
	// 16 bit resolution, left-justified
	int16_t zBitGyro = (uint16_t) zGyro_MSBs << 8 | zGyro_LSBs;

	return zBitGyro * gyroGain;
}

void LSM9DS0::startLSM() {

}

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

	char read;

	Serial.println("*** ACCELEROMETER CALIBRATION PROTOCOL STARTED ***");
	Serial.println("There are going to be six positions to orient the sensor in. It must be still while calibrating.");
	Serial.println("It may help to have a corner so you are as close to perpendicular as possible.\n");

	// POSITION 1 //
	Serial.println("Position 1 is like such:");
	Serial.println("-------");
	Serial.println("|.    |");
	Serial.println("|     |");
	Serial.println("|     |");
	Serial.println("-------");
	Serial.println("Enter Y when ready. Enter anything else to exit calibration.");
	
	while(Serial.available() == 0) {}

		read = Serial.read();

		if(read != 'y' && read != 'Y') {
			Serial.println("Calibration exited.");
			return;
		}

	Serial.println("Gathering data. Please wait...\n");

	while(n < ACCEL_CALIB_SAMPLES) {
		n += 1;
		ax1 += getxAccel();
		az1 += getzAccel();
	}

	n = 0;
	ax1 /= ACCEL_CALIB_SAMPLES;
	az1 /= ACCEL_CALIB_SAMPLES;


	// POSITION 2 //
	Serial.println("Position 2 is like such:");
	Serial.println("|----------|");
	Serial.println("|          |");
	Serial.println("|.         |");
	Serial.println("|----------|");
	Serial.println("Enter Y when ready. Enter anything else to exit calibration.");
	
	while(Serial.available() == 0) {}

		read = Serial.read();

		if(read != 'y' && read != 'Y') {
			Serial.println("Calibration exited.");
			return;
		}

	Serial.println("Gathering data. Please wait...\n");

	while(n < ACCEL_CALIB_SAMPLES) {
		n += 1;
		ay2 += getyAccel();
		az2 += getzAccel();
	}

	n = 0;
	ay2 /= ACCEL_CALIB_SAMPLES;
	az2 /= ACCEL_CALIB_SAMPLES;


	// POSITION 3 //
	Serial.println("Position 3 is like such:");
	Serial.println("-------");
	Serial.println("|     |");
	Serial.println("|     |");
	Serial.println("|    .|");
	Serial.println("-------");
	Serial.println("Enter Y when ready. Enter anything else to exit calibration.");
	
	while(Serial.available() == 0) {}

		read = Serial.read();

		if(read != 'y' && read != 'Y') {
			Serial.println("Calibration exited.");
			return;
		}

	Serial.println("Gathering data. Please wait...\n");

	while(n < ACCEL_CALIB_SAMPLES) {
		n += 1;
		ax3 += getxAccel();
		az3 += getzAccel();
	}

	n = 0;
	ax3 /= ACCEL_CALIB_SAMPLES;
	az3 /= ACCEL_CALIB_SAMPLES;


	// POSITION 4 //
	Serial.println("Position 4 is like such:");
	Serial.println("|----------|");
	Serial.println("|         .|");
	Serial.println("|          |");
	Serial.println("|----------|");
	Serial.println("Enter Y when ready. Enter anything else to exit calibration.");
	
	while(Serial.available() == 0) {}

		read = Serial.read();

		if(read != 'y' && read != 'Y') {
			Serial.println("Calibration exited.");
			return;
		}

	Serial.println("Gathering data. Please wait...\n");

	while(n < ACCEL_CALIB_SAMPLES) {
		n += 1;
		ay4 += getyAccel();
		az4 += getzAccel();
	}

	n = 0;
	ay4 /= ACCEL_CALIB_SAMPLES;
	az4 /= ACCEL_CALIB_SAMPLES;

	// POSITION 5 //
	Serial.println("Position 5 is like such:");
	Serial.println("    TOP    ");
	Serial.println("-.---------");
	Serial.println("|         |");
	Serial.println("-----------");
	Serial.println("   BOTTOM  ");
	Serial.println("Enter Y when ready. Enter anything else to exit calibration.");
	
	while(Serial.available() == 0) {}

		read = Serial.read();

		if(read != 'y' && read != 'Y') {
			Serial.println("Calibration exited.");
			return;
		}

	Serial.println("Gathering data. Please wait...\n");

	while(n < ACCEL_CALIB_SAMPLES) {
		n += 1;
		ax5 += getxAccel();
		ay5 += getyAccel();
	}

	n = 0;
	ax5 /= ACCEL_CALIB_SAMPLES;
	ay5 /= ACCEL_CALIB_SAMPLES;


	// POSITION 6 //
	Serial.println("Position 6 is like such:");
	Serial.println("   BOTTOM  ");
	Serial.println("-----------");
	Serial.println("|         |");
	Serial.println("-.---------");
	Serial.println("    TOP    ");
	Serial.println("Enter Y when ready. Enter anything else to exit calibration.");
	
	while(Serial.available() == 0) {}

		read = Serial.read();

		if(read != 'y' && read != 'Y') {
			Serial.println("Calibration exited.");
			return;
		}

	Serial.println("Gathering data. Please wait...\n");

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

	Serial.println("Calibration complete!");
	Serial.println("We've already set these values for you in the system, but the offsets are printed for your convenience.\n");

	Serial.print("X Offset: "); Serial.println(X_AB_OFFSET);
	Serial.print("Y Offset: "); Serial.println(Y_AB_OFFSET);
	Serial.print("Z Offset: "); Serial.println(Z_AB_OFFSET);
	Serial.println();
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

	Serial.println("*** MAGNETOMETER CALIBRATION PROTOCOL STARTED ***");
	Serial.println("Please turn the device through the air in a figure 8 fashion until calibration finishes.\n");
	Serial.println("When ready, send Y. To exit, send anything else.");

	while(Serial.available() == 0) {}

	char read = Serial.read();

	if(read != 'y' && read != 'Y') {
		Serial.println("Calibration exited.");
		return;
	}

	Serial.println("Calibrating. Continue turning...");

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

	Serial.println("Calibration complete!");
	Serial.println("We've already set these values for you in the system, but the offsets are printed for your convenience.\n");

	Serial.print("X Hard-Iron Offset: "); Serial.println(X_HI_OFFSET);
	Serial.print("Y Hard-Iron Offset: "); Serial.println(Y_HI_OFFSET);
	Serial.print("Z Hard-Iron Offset: "); Serial.println(Z_HI_OFFSET);
	Serial.println();

	Serial.print("X Soft-Iron Scale: "); Serial.println(X_SI_SCALE);
	Serial.print("Y Soft-Iron Scale: "); Serial.println(Y_SI_SCALE);
	Serial.print("Z Soft-Iron Scale: "); Serial.println(Z_SI_SCALE);
	Serial.println();
}

// Calibration method for gyro offsets
void LSM9DS0::calibrateGyroOffsets() {
	int n = 0;

	float sumx = 0.0f;
	float sumy = 0.0f;
	float sumz = 0.0f;

	Serial.flush();
	Serial.println("\n*** GYROSCOPE CALIBRATION PROTOCOL STARTED ***");
	Serial.println("Please keep the device still for calibration. When ready, send Y. To exit, send anything else.");

	while(Serial.available() == 0) {}

	char read = Serial.read();

	if(read != 'y' && read != 'Y') {
		Serial.println("Calibration exited.");
		return;
	}
	
	Serial.println("Calibrating. Please wait...");

	while (n < GYRO_CALIB_SAMPLES) {
		n += 1;
		sumx += getxGyro();
		sumy += getyGyro();
		sumz += getzGyro();
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
