#ifndef LSM9DS0_H
#define LSM9DS0_H

#include <Arduino.h>

/* 
/////////////////////////////////////////
// CONTROL BIT REGISTER CONFIGURATIONS //
/////////////////////////////////////////

Reference: https://cdn-shop.adafruit.com/datasheets/LSM9DS0.pdf

*** CTRL_REG0_XM Configuration ***

Boot Configuration (Bit 1):
0 | Normal Mode
1 | Boot Memory Content

FIFO Enable (Bit 2):
0 | Disable
1 | Enable

FIFO Programmable Watermark Enable (Bit 3):
0 | Disable
1 | Enable

*RESERVED* MUST BE 0 (Bits 4-5)

High Pass Filter for Click Function (Bit 6):
0 | Disable
1 | Enable

High Pass Filter for Interrupt Generator 1 (Bit 7):
0 | Disable
1 | Enable

High Pass Filter for Interrupt Generator 2 (Bit 8):
0 | Disable
1 | Enable


*** CTRL_REG1_XM Configuration ***

Acceleration Data Rate (Bits 1-4):
0 0 0 0 | Power Down Mode
0 0 0 1 | 3.125 Hz
0 0 1 0 | 6.25 Hz
0 0 1 1 | 12.5 Hz
0 1 0 0 | 25 Hz
0 1 0 1 | 50 Hz
0 1 1 0 | 100 Hz
0 1 1 1 | 200 Hz
1 0 0 0 | 400 Hz
1 0 0 1 | 800 Hz
1 0 1 0 | 1600 Hz

Block Data Update Rate (Bit 5):
0 | Continuous Update
1 | Output Registers Not Updated Until MSB & LSB Read

Acceleration Z Axis Enable (Bit 6):
0 | Disable
1 | Enable

Acceleration Y Axis Enable (Bit 6):
0 | Disable
1 | Enable

Acceleration X Axis Enable (Bit 6):
0 | Disable
1 | Enable


*** CTRL_REG2_XM Configuration ***

Anti-Aliasing Filter Bandwidth (Bits 1-2):
0 0 | 773 Hz
0 1 | 194 Hz
1 0 | 362 Hz
1 1 | 50 Hz

Acceleration Scale Selection (Bits 3-5):
0 0 0 | +/- 2g
0 0 1 | +/- 4g
0 1 0 | +/- 6g
0 1 1 | +/- 8g
1 0 0 | +/- 16g

Self-Test Mode Configuration (Bits 6-7):
0 0 | Normal Mode
0 1 | Positive Sign Self-Test
1 0 | Negative Sign Self-Test
1 1 | Not Allowed

Serial Interface Mode Selection (Bit 8):
0 | 4 Wire Interface
1 | 3 Wire Interface

--------------------
Accelerometer Gain Configuration
NOTE: YOU MUST CHANGE THE ACCELEROMETER GAIN IF YOU CHANGE THE SCALE SELECTION!
+/- 2g  | 0.000061
+/- 4g  | 0.000122
+/- 6g  | 0.000183
+/- 8g  | 0.000244
+/- 16g | 0.000732


*** CTRL_REG3_XM/CTRL_REG4_XM Configuration ***

Boot on INT1_XM Pin Enable (Bit 1):
0 | Disable
1 | Enable

Tap Generator Interrupt on INT1_XM (Bit 2):
0 | Disable
1 | Enable

Inertial Interrupt Generator 1 on INT1_XM (Bit 3):
0 | Disable
1 | Enable

Inertial Interrupt Generator 2 on INT1_XM (Bit 4):
0 | Disable
1 | Enable

Magnetic Interrupt Generator on INT1_XM (Bit 5):
0 | Disable
1 | Enable

Accelerometer Data-Ready Signal on INT1_XM (Bit 6):
0 | Disable
1 | Enable

Magnetometer Data-Ready Signal on INT1_XM (Bit 7):
0 | Disable
1 | Enable

FIFO Empty Indication on INT1_XM (Bit 8):
0 | Disable
1 | Enable

-[!] CTRL_REG4_XM IS IDENTICAL EXCEPT ON PIN INT2_XM -


*** CTRL_REG5_XM Configuration ***

Temperature Sensor Enable (Bit 1):
0 | Disable
1 | Enable

Magnetic Resolution Selection (Bits 2-3):
0 0 | Low Resolution
1 1 | High Resolution

Magnetic Data Rate Selection (Bits 4-6):
0 0 0 | 3.125 Hz
0 0 1 | 6.25 Hz
0 1 0 | 12.5 Hz
0 1 1 | 25 Hz
1 0 0 | 50 Hz
1 0 1 | 100 Hz * ONLY for accelerometer ODR > 50 Hz OR accelerometer in power down mode

Latch Interrupt Request On INT2_SRC Register
0 | Not Latched
1 | Latched

Latch Interrupt Request On INT1_SRC Register
0 | Not Latched
1 | Latched
--------------------
Temperature Gain: ALWAYS 1/8. DO NOT CHANGE.


*** CTRL_REG6_XM Configuration ***

*RESERVED* MUST BE 0 (Bits 1, 4-8)

Magnetic Full Scale Selection (Bits 2-3):
0 0 | +/- 2 Gauss
0 1 | +/- 4 Gauss
1 0 | +/- 8 Gauss
1 1 | +/- 12 Gauss
--------------------
Magnetometer Gain Configuration
NOTE: YOU MUST CHANGE THE MAGNETOMETER GAIN IF YOU CHANGE THE SCALE SELECTION!
+/- 2 Gauss  | 0.00008
+/- 4 Gauss  | 0.00016
+/- 8 Gauss  | 0.00032
+/- 12 Gauss | 0.00048


*** CTRL_REG7_XM Configuration ***

High-Pass Filter Mode for Acceleration Data (Bits 1-2):
0 0 | Normal Mode (Resets the accelerometer filter reference registers)
0 1 | Reference Signal for Filtering
1 0 | Normal Mode
1 1 | Autoreset on Interrupt

Filtered Acceleration Data Selection (Bit 3):
0 | Internal Filter Bypassed
1 | Data from Internal Filter Sent to Output Register and FIFO

*RESERVED* MUST BE 0 (Bits 4-5)

Magnetic Data Low-Power Mode (Bit 6):
0 | Magnetic Data Rate Selection Set by CTRL_REG5_XM
1 | Magnetic Data Rate Forced to 3.125 Hz

Magnetic Sensor Mode Selection (Bits 7-8):
0 0 | Continuous Conversion Mode
0 1 | Single Conversion Mode
1 0 | Power Down Mode
1 1 | Power Down Mode



*** CTRL_REG1_G Configuration ***

-THESE 4 BITS ARE DEPENDENT ON EACH OTHER- 
Output Data Rate Selection (Bits 1-2), Bandwidth Selection (Bits 3-4):
0 0 0 0 | 95 Hz, 12.5 Hz
0 0 0 1 | 95 Hz, 25 Hz
0 0 1 0 | 95 Hz, 12.5 Hz
0 0 1 1 | 95 Hz, 25 Hz
0 1 0 0 | 190 Hz, 12.5 Hz
0 1 0 1 | 190 Hz, 25 Hz
0 1 1 0 | 190 Hz, 50 Hz
0 1 1 1 | 190 Hz, 70 Hz
1 0 0 0 | 380 Hz, 20 Hz
1 0 0 1 | 380 Hz, 25 Hz
1 0 1 0 | 380 Hz, 50 Hz
1 0 1 1 | 380 Hz, 100 Hz
1 1 0 0 | 760 Hz, 30 Hz
1 1 0 1 | 760 Hz, 35 Hz
1 1 1 0 | 760 Hz, 50 Hz
1 1 1 1 | 760 Hz, 100 Hz

Power Down Mode Enable (Bit 5):
0 | Power Down Mode
1 | Normal/Sleep Mode

Z Axis Enable (Bit 6):
0 | Disable
1 | Enable

Y Axis Enable (Bit 7):
0 | Disable
1 | Enable

X Axis Enable (Bit 8):
0 | Disable
1 | Enable



*** CTRL_REG2_G Configuration ***

*RESERVED* MUST BE 0 (Bits 1-2)

High Pass Filter Mode Selection (Bits 3-4):
0 0 | Normal Mode (Reset Reading HP_RESET_FILTER)
0 1 | Reference Signal for Filtering
1 0 | Normal Mode
1 1 | Autoreset on Interrupt Event

# NEXT 4 BITS DEPENDENT ON THE ODR #
High Pass Filter Cutoff Frequency (Hz) (Bits 5-8):
BITS | ODR=95 Hz | ODR=190 Hz | ODR=380 Hz | ODR=760 Hz
-----|-----------|------------|------------|-----------
0000 |    7.2    |    13.5    |     27     |    51.4
0001 |    3.5    |    7.2     |    13.5    |    27
0010 |    1.8    |    3.5     |    7.2     |    13.5
0011 |    0.9    |    1.8     |    3.5     |    7.2
0100 |    0.45   |    0.9     |    1.8     |    3.5
0101 |    0.18   |    0.45    |    0.9     |    1.8
0110 |    0.09   |    0.18    |    0.45    |    0.9
0111 |    0.045  |    0.09    |    0.18    |    0.45
1000 |    0.018  |    0.045   |    0.09    |    0.18
1001 |    0.009  |    0.018   |    0.045   |    0.09



*** CTRL_REG3_G Configuration ***

Interrupt Enable on INT_G Pin (Bit 1):
0 | Disable
1 | Enable

Boot Status Available on INT_G Pin (Bit 1):
0 | Disable
1 | Enable

Interrupt Active Configuration on INT_G Pin (Bit 1):
0 | High
1 | Low

Push-Pull/Open Drain (Bit 1):
0 | Push-Pull
1 | Open Drain

Data Ready on DRDY_G Pin (Bit 1):
0 | Disable
1 | Enable

FIFO Watermark Interrupt on INT_G Pin (Bit 1):
0 | Disable
1 | Enable

FIFO Overrun Interrupt on INT_G Pin (Bit 1):
0 | Disable
1 | Enable

FIFO Empty Interrupt on INT_G Pin (Bit 1):
0 | Disable
1 | Enable



*** CTRL_REG4_G Configuration ***

Block Data Update (Bit 1):
0 | Continuous Update
1 | Output Registers Not Updated Until MSB & LSB Read

Big/Little Endian Data Selection (Bit 2):
0 | Data LSB @ Lower Address
1 | Data MSB @ Lower Address

Full-Scale Selection (Bits 3-4) [SEE NOTE BELOW ABOUT THIS]:
0 0 | 245 dps
0 1 | 500 dps
1 0 | 2000 dps
1 1 | 2000 dps

*RESERVED* MUST BE ZERO (Bit 5)

Self-Test Enable (Bits 6-7):
0 0 | Normal Mode
0 1 | Self Test 0, X Positive, Y/Z Negative
1 1 | Self Test 1, X Negative, Y/Z Positive

Serial Interface Mode Selection (Bit 8):
0 | 4 Wire Interface
1 | 3 Wire Interface

--------------------
Gyro Gain Configuration
NOTE: YOU MUST CHANGE THE GYRO GAIN IF YOU CHANGE THE FULL SCALE SELECTION!
245 dps  | gyroGain = 0.00875
500 dps  | gyroGain = 0.0175
2000 dps | gyroGain = 0.07


*** CTRL_REG5_G Configuration ***

Reboot Memory Content (Bit 1):
0 | Normal Mode
1 | Reboot Memory Content

FIFO Enable (Bit 2):
0 | Disable
1 | Enable

*RESERVED* MUST BE 0 (Bit 3)

High Pass Filter Enable (Bit 4):
0 | Disable
1 | Enable

INT1 Selection Configuration (Bits 5-6):
0 0 | Default

OUT Selection Configuration (Bits 7-8):
0 0 | Default


*** FIFO_CTRL_REG Configuration ***

FIFO Mode Configuration (Bits 1-3):
0 0 0 | Bypass Mode
0 0 1 | FIFO Mode
0 1 0 | Stream Mode
0 1 1 | Stream-to-FIFO Mode
1 0 0 | Bypass-to-Stream Mode

FIFO Watermark Level (Bits 4-8)

*/

// Device addresses & their bit registers
const uint8_t XM_ADDRESS = 0x1D;
const uint8_t G_ADDRESS = 0x6B;

//////////////////
// XM REGISTERS //
//////////////////

// Temperature Sensor Data. 12 bit, two's complement, right-justified
const uint8_t OUT_TEMP_L_XM = 0x05; // Low - 8 bits are the 8 LSBs.
const uint8_t OUT_TEMP_H_XM = 0x06; // High - Last 4 bits are MSBs. First 4 are useless, need to be masked out

// Magnetometer Status Info (OVERRUN AND AVAILABILITY)
const uint8_t STATUS_REG_M = 0x07;

// Magnetic Data Values. 16 bit, two's complement, left-justified
const uint8_t OUT_X_L_M = 0x08; // X
const uint8_t OUT_X_H_M = 0x09;
const uint8_t OUT_Y_L_M = 0x0A; // Y
const uint8_t OUT_Y_H_M = 0x0B;
const uint8_t OUT_Z_L_M = 0x0C; // Z
const uint8_t OUT_Z_H_M = 0x0D;

// Device Identification for Accel/Mag
const uint8_t WHO_AM_I_XM = 0x0F;

// Interrupt Registers
const uint8_t INT_CTRL_REG_M = 0x12;
const uint8_t INT_SRC_REG_M = 0x13;
const uint8_t INT_THS_L_M= 0x14;
const uint8_t INT_THS_H_M= 0x15;

// Magnetic Offset values, 16 bit, two's complement, left-justified
const uint8_t OFFSET_X_L_M = 0x16; // X
const uint8_t OFFSET_X_H_M = 0x17;
const uint8_t OFFSET_Y_L_M = 0x18; // Y
const uint8_t OFFSET_Y_H_M = 0x19;
const uint8_t OFFSET_Z_L_M = 0x1A; // Z
const uint8_t OFFSET_Z_H_M = 0x1B;

// Reference Values for high pass filters for X, Y, and Z accel data
const uint8_t REFERENCE_X = 0x1C;
const uint8_t REFERENCE_Y = 0x1D;
const uint8_t REFERENCE_Z = 0x1E;

// Control Registers
const uint8_t CTRL_REG0_XM = 0x1F;
const uint8_t CTRL_REG1_XM = 0x20;
const uint8_t CTRL_REG2_XM = 0x21;
const uint8_t CTRL_REG3_XM = 0x22;
const uint8_t CTRL_REG4_XM = 0x23;
const uint8_t CTRL_REG5_XM = 0x24;
const uint8_t CTRL_REG6_XM = 0x25;
const uint8_t CTRL_REG7_XM = 0x26;

// Accelerometer Status Info (OVERRUN AND AVAILABILITY)
const uint8_t STATUS_REG_A = 0x27;

// Acceleration Data Values. 16 bit, two's complement, left-justified
const uint8_t OUT_X_L_A = 0x28;
const uint8_t OUT_X_H_A = 0x29;
const uint8_t OUT_Y_L_A = 0x2A;
const uint8_t OUT_Y_H_A = 0x2B;
const uint8_t OUT_Z_L_A = 0x2C;
const uint8_t OUT_Z_H_A = 0x2D;

// FIFO Registers
const uint8_t FIFO_CTRL_REG = 0x2E;
const uint8_t FIFO_SRC_REG = 0x2F;

// Inertial Interrupt Generator 1 Registers
const uint8_t INT_GEN_1_REG = 0x30;
const uint8_t INT_GEN_1_SRC = 0x31;
const uint8_t INT_GEN_1_THS = 0x32;
const uint8_t INT_GEN_1_DURATION = 0x33;

// Inertial Interrupt Generator 2 Registers
const uint8_t INT_GEN_2_REG = 0x34;
const uint8_t INT_GEN_2_SRC = 0x35;
const uint8_t INT_GEN_2_THS = 0x36;
const uint8_t INT_GEN_2_DURATION = 0x37;

// Interrupt Click Registers
const uint8_t CLICK_CFG = 0x38;
const uint8_t CLICK_SRC = 0x39;
const uint8_t CLICK_THS = 0x3A;

// Time Data Registers
const uint8_t TIME_LIMIT = 0x3B;
const uint8_t TIME_LATENCY = 0x3C;
const uint8_t TIME_WINDOW = 0x3D;

// Activation Registers
const uint8_t ACT_THS = 0x3E;
const uint8_t ACT_DUR = 0x3F;

/////////////////
// G REGISTERS //
/////////////////

// Device Identification for Gyro
const uint8_t WHO_AM_I_G = 0x0F;

// Control Registers
const uint8_t CTRL_REG_1_G = 0x20;
const uint8_t CTRL_REG_2_G = 0x21;
const uint8_t CTRL_REG_3_G = 0x22;
const uint8_t CTRL_REG_4_G = 0x23;
const uint8_t CTRL_REG_5_G = 0x24;

// References for Interrupts
const uint8_t DATACAPTURE_G = 0x25;

// Gyro Status Info (OVERRUN AND AVAILABILITY)
const uint8_t STATUS_REG_G = 0x27;

// Angular Rate Data
const uint8_t OUT_X_L_G = 0x28; // X
const uint8_t OUT_X_H_G = 0x29;
const uint8_t OUT_Y_L_G = 0x2A; // Y
const uint8_t OUT_Y_H_G = 0x2B;
const uint8_t OUT_Z_L_G = 0x2C; // Z
const uint8_t OUT_Z_H_G = 0x2D;

// FIFO Registers
const uint8_t FIFO_CTRL_REG_G = 0x2E;
const uint8_t FIFO_SRC_REG_G = 0x2F;

// Interrupt Registers
const uint8_t INT1_CFG_G = 0x30;
const uint8_t INT1_SRC_G = 0x31;
const uint8_t INT1_THS_XH_G = 0x32;
const uint8_t INT1_THS_XL_G = 0x33;
const uint8_t INT1_THS_YH_G = 0x34;
const uint8_t INT1_THS_YL_G = 0x35;
const uint8_t INT1_THS_ZH_G = 0x36;
const uint8_t INT1_THS_ZL_G = 0x37;
const uint8_t INT1_DURATION_G = 0x38;

////////////////////////
// PHYSICAL CONSTANTS //
////////////////////////

const float TEMP_INTERCEPT = 24.0;
const double GRAV_ACCEL = 9.80665;

class LSM9DS0 {
public:
	LSM9DS0();
	virtual ~LSM9DS0();

	void initXM(uint8_t xmaddress);
	void initG(uint8_t gaddress);

	double getTemp();

	double getxAccel();
	double getyAccel();
	double getzAccel();

	double getxMag();
	double getyMag();
	double getzMag();

	double getxGyro();
	double getyGyro();
	double getzGyro();

	uint8_t readByte(uint8_t devAddress, uint8_t regAddress);
	void writeByte(uint8_t devAddress, uint8_t regAddress, uint8_t byte);

private:
	// Private XM variables
	double accelxoffset, accelyoffset, accelzoffset;
	double magxoffset, magyoffset, magzoffset;
	double accelGain, magGain;
	const double TEMP_GAIN;

	// Private G variables
	double gyroxoffset, gyroyoffset, gyrozoffset;
	double gyroGain;
}

extern LSM9DS0 lsm9ds0;

#endif
