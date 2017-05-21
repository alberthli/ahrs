"""
*** FLIGHT CONTROLLER ***

Albert Li | 2017

Description: Flight Controller for a Raspberry Pi-controlled quadcopter

Sensors:
LSM9DS0 9 DOF Accel/Gyro/Mag Board | Using I2C for communication/configuration

*** CONFIGURATION ***
	You must manually enter the hex code corresponding to the desired settings
	in the control bit registers. The accelerometer and magnetometer seem to be
	on one shared board, while the gyro is on its own

Adafruit Ultimate GPS Breakout (MTK3339 Chipset)
"""

import Adafruit_Python_GPIO.Adafruit_GPIO.I2C as i2c

# Addresses for the XM and G when the SCL/SDA lines are pulled up (THEY SHOULD ALWAYS BE)
XM_ADDRESS = 0x1D
G_ADDRESS = 0x6B

# Guess at the intercept for the temperature sensor
TEMP_INTERCEPT = 24.0

# Physical Constants
GRAV_ACCEL = 9.80665 # Value of acceleration due to gravity (m*s^-2)

# Class Definition for the Accelerometer/Magnetometer part of the LSM9DS0
class LSM9DS0_XM:

    ###################################################################
    # ACCELEROMETER/MAGNETOMETER BIT REGISTERS                        #
    # Reference: https://cdn-shop.adafruit.com/datasheets/LSM9DS0.pdf #
	# 																  #
    # *** Register names: L Stands for LOW BYTE, H for HIGH ***       #
    ###################################################################

    # Temperature Sensor Data. 12 bit, two's complement, right-justified
    OUT_TEMP_L_XM = 0x05 # Low - 8 bits are the 8 LSBs.
    OUT_TEMP_H_XM = 0x06 # High - Last 4 bits are MSBs. First 4 are useless.

    STATUS_REG_M = 0x07

    # Magnetic Data Values. 16 bit, two's complement, left-justified
    OUT_X_L_M = 0x08 # X
    OUT_X_H_M = 0x09
    OUT_Y_L_M = 0x0A # Y
    OUT_Y_H_M = 0x0B
    OUT_Z_L_M = 0x0C # Z
    OUT_Z_H_M = 0x0D

    # Device Identification for Accel/Mag
    WHO_AM_I_XM = 0x0F

    INT_CTRL_REG_M = 0x12
    INT_SRC_REG_M = 0x13
    INT_THS_L_M= 0x14
    INT_THS_H_M= 0x15

    # Magnetic Offset values, 16 bit, two's complement, left-justified
    OFFSET_X_L_M = 0x16 # X
    OFFSET_X_H_M = 0x17
    OFFSET_Y_L_M = 0x18 # Y
    OFFSET_Y_H_M = 0x19
    OFFSET_Z_L_M = 0x1A # Z
    OFFSET_Z_H_M = 0x1B

    # Reference Values for high pass filters for X, Y, and Z accel data
    REFERENCE_X = 0x1C
    REFERENCE_Y = 0x1D
    REFERENCE_Z = 0x1E

    # Control Registers
    CTRL_REG0_XM = 0x1F
    CTRL_REG1_XM = 0x20
    CTRL_REG2_XM = 0x21
    CTRL_REG3_XM = 0x22
    CTRL_REG4_XM = 0x23
    CTRL_REG5_XM = 0x24
    CTRL_REG6_XM = 0x25
    CTRL_REG7_XM = 0x26

    STATUS_REG_A = 0x27

    # Acceleration Data Values. 16 bit, two's complement, left-justified
    OUT_X_L_A = 0x28
    OUT_X_H_A = 0x29
    OUT_Y_L_A = 0x2A
    OUT_Y_H_A = 0x2B
    OUT_Z_L_A = 0x2C
    OUT_Z_H_A = 0x2D

    FIFO_CTRL_REG = 0x2E
    FIFO_SRC_REG = 0x2F

    INT_GEN_1_REG = 0x30
    INT_GEN_1_SRC = 0x31
    INT_GEN_1_THS = 0x32
    INT_GEN_1_DURATION = 0x33

    INT_GEN_2_REG = 0x34
    INT_GEN_2_SRC = 0x35
    INT_GEN_2_THS = 0x36
    INT_GEN_2_DURATION = 0x37

    CLICK_CFG = 0x38
    CLICK_SRC = 0x39
    CLICK_THS = 0x3A

    TIME_LIMIT = 0x3B
    TIME_LATENCY = 0x3C
    TIME_WINDOW = 0x3D

    ACT_THS = 0x3E
    ACT_DUR = 0x3F

    def __init__(self, address = XM_ADDRESS):
    	self.device = i2c.get_i2c_device(address)

    	self.accelxoffset = 0.0
    	self.accelyoffset = 0.0
    	self.accelzoffset = 0.0

    	self.magxoffset = 0.0
    	self.magyoffset = 0.0
    	self.magzoffset = 0.0

    	#######################################
    	# CONTROL BIT REGISTER CONFIGURATIONS #
    	#######################################

    	"""
    	CTRL_REG0_XM Configuration:

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
    	"""
    	self.device.write8(self.CTRL_REG0_XM, 0x00) # All Disabled, Defaults

    	"""
    	CTRL_REG1_XM Configuration:

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
    	"""
    	self.device.write8(self.CTRL_REG1_XM, 0x57) # 100 Hz Accel. Sampling Rate, Continuous Update, All Axes Enabled

    	"""
		CTRL_REG2_XM Configuration:

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
    	"""
    	self.device.write8(self.CTRL_REG2_XM, 0x08) # 773 Hz AAFB, +/- 4g, Normal Self-Test, 4 Wire Interface
    	self.accelGain = 0.000122

    	"""
    	CTRL_REG3_XM/CTRL_REG4_XM Configuration:

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

		*** CTRL_REG4_XM IS IDENTICAL EXCEPT ON PIN INT2_XM ***
    	"""
    	self.device.write8(self.CTRL_REG3_XM, 0x00) # All disable, defaults
    	self.device.write8(self.CTRL_REG4_XM, 0x00) # All disable, defaults

    	"""
    	CTRL_REG5_XM Configuration:

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
    	"""
    	self.device.write8(self.CTRL_REG5_XM, 0xF4) # Temperature Enabled, High Mag Res, 100 Hz Sampling, No Latched Ints
    	self.TEMP_GAIN = 0.125

    	"""
    	CTRL_REG6_XM Configuration:

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
    	"""
    	self.device.write8(self.CTRL_REG6_XM, 0x00) # +/- 2 Gauss Scale
    	self.magGain = 0.00008

    	"""
    	CTRL_REG7_XM Configuration:

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
    	"""
    	self.device.write8(self.CTRL_REG7_XM, 0x00) # Defaults

    	"""
    	FIFO_CTRL_REG Configuration:

    	FIFO Mode Configuration (Bits 1-3):
    	0 0 0 | Bypass Mode
    	0 0 1 | FIFO Mode
    	0 1 0 | Stream Mode
    	0 1 1 | Stream-to-FIFO Mode
    	1 0 0 | Bypass-to-Stream Mode

    	FIFO Watermark Level (Bits 4-8)
    	"""
    	self.device.write8(self.FIFO_CTRL_REG, 0x00) # Bypass Mode by Default

    # Returns Temperature (deg C)
    def getTemp(self):
    	# 12 Bit Precision, Right-Justified
    	temp_MSBs = self.device.readU8(self.OUT_TEMP_H_XM)
    	temp_LSBs = self.device.readU8(self.OUT_TEMP_L_XM)
    	bitTemp = (temp_MSBs << 8 | temp_LSBs) & 0xFFF

    	if bitTemp > 2047:
    		bitTemp -= 4096

    	return TEMP_INTERCEPT + bitTemp * self.TEMP_GAIN

    # Returns x Acceleration (m*s^-2)
    def getxAccel(self):
    	# 16 Bit Precision, Left-Justified
    	xAccel_MSBs = self.device.readU8(self.OUT_X_H_A)
    	xAccel_LSBs = self.device.readU8(self.OUT_X_L_A)
    	xBitAccel = xAccel_MSBs << 8 | xAccel_LSBs

    	if xBitAccel > 32767:
    		xBitAccel -= 65536

    	return xBitAccel * self.accelGain * GRAV_ACCEL

    # Returns y Acceleration (m*s^-2)
    def getyAccel(self):
    	# 16 Bit Precision, Left-Justified
    	yAccel_MSBs = self.device.readU8(self.OUT_Y_H_A)
    	yAccel_LSBs = self.device.readU8(self.OUT_Y_L_A)
    	yBitAccel = yAccel_MSBs << 8 | yAccel_LSBs

    	if yBitAccel > 32767:
    		yBitAccel -= 65536

    	return yBitAccel * self.accelGain * GRAV_ACCEL

    # Returns z Acceleration (m*s^-2)
    def getzAccel(self):
    	# 16 Bit Precision, Left-Justified
    	zAccel_MSBs = self.device.readU8(self.OUT_Z_H_A)
    	zAccel_LSBs = self.device.readU8(self.OUT_Z_L_A)
    	zBitAccel = zAccel_MSBs << 8 | zAccel_LSBs

    	if zBitAccel > 32767:
    		zBitAccel -= 65536

    	return zBitAccel * self.accelGain * GRAV_ACCEL

    # Returns x Magnetometer Data (mgauss)
    def getxMag(self):
    	# 16 Bit Precision, Left-Justified
    	xMag_MSBs = self.device.readU8(self.OUT_X_H_M)
    	xMag_LSBs = self.device.readU8(self.OUT_X_L_M)
    	xBitMag = xMag_MSBs << 8 | xMag_LSBs

    	if xBitMag > 32767:
    		xBitMag -= 65536

    	return xBitMag * self.magGain

    # Returns y Magnetometer Data (mgauss)
    def getyMag(self):
    	# 16 Bit Precision, Left-Justified
    	yMag_MSBs = self.device.readU8(self.OUT_Y_H_M)
    	yMag_LSBs = self.device.readU8(self.OUT_Y_L_M)
    	yBitMag = yMag_MSBs << 8 | yMag_LSBs

    	if yBitMag > 32767:
    		yBitMag -= 65536

    	return yBitMag * self.magGain

    # Returns z Magnetometer Data (mgauss)
    def getzMag(self):
    	# 16 Bit Precision, Left-Justified
    	zMag_MSBs = self.device.readU8(self.OUT_Z_H_M)
    	zMag_LSBs = self.device.readU8(self.OUT_Z_L_M)
    	zBitMag = zMag_MSBs << 8 | zMag_LSBs

    	if zBitMag > 32767:
    		zBitMag -= 65536

    	return zBitMag * self.magGain

# Class Definition for the Gyro part of the LSM9DS0
class LSM9DS0_G:

    ###################################################################
    # GYRO BIT REGISTERS 											  #
    # Reference: https://cdn-shop.adafruit.com/datasheets/LSM9DS0.pdf #
	# 																  #
    # *** Register names: L Stands for LOW BYTE, H for HIGH ***       #
    ###################################################################
    
    # Device Identification for Gyro
    WHO_AM_I_G = 0x0F
    
    # Control Registers
    CTRL_REG_1_G = 0x20 
    CTRL_REG_2_G = 0x21
    CTRL_REG_3_G = 0x22
    CTRL_REG_4_G = 0x23
    CTRL_REG_5_G = 0x24
    
    DATACAPTURE_G = 0x25
    
    STATUS_REG_G = 0x27

    # Angular Rate Data
    OUT_X_L_G = 0x28 # X
    OUT_X_H_G = 0x29
    OUT_Y_L_G = 0x2A # Y
    OUT_Y_H_G = 0x2B
    OUT_Z_L_G = 0x2C # Z
    OUT_Z_H_G = 0x2D

    FIFO_CTRL_REG_G = 0x2E
    FIFO_SRC_REG_G = 0x2F

    INT1_CFG_G = 0x30
    INT1_SRC_G = 0x31
    INT1_THS_XH_G = 0x32
    INT1_THS_XL_G = 0x33
    INT1_THS_YH_G = 0x34
    INT1_THS_YL_G = 0x35
    INT1_THS_ZH_G = 0x36
    INT1_THS_ZL_G = 0x37
    INT1_DURATION_G = 0x38

    def __init__(self, address = G_ADDRESS):
    	self.device = i2c.get_i2c_device(address)

    	self.gyroxoffset = 0.0
    	self.gyroyoffset = 0.0
    	self.gyrozoffset = 0.0

    	#######################################
    	# CONTROL BIT REGISTER CONFIGURATIONS #
    	#######################################

    	"""
    	CTRL_REG1_G Configuration:

		# ***THESE 4 BITS ARE DEPENDENT ON EACH OTHER*** #
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
    	"""
    	self.device.write8(self.CTRL_REG_1_G, 0x0F) # Default ODR and Bandwidth, Normal Mode, All Axes Enabled

    	"""
    	CTRL_REG2_G Configuration:

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
    	"""
    	self.device.write8(self.CTRL_REG_2_G, 0x00) # Normal Mode, 7.2 Hz HPF Cutoff Frequency

    	"""
    	CTRL_REG3_G Configuration:

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
    	"""
    	self.device.write8(self.CTRL_REG_3_G, 0x00) # All Defaults

    	"""
    	CTRL_REG4_G Configuration:

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
    	"""
    	self.device.write8(self.CTRL_REG_4_G, 0x00) # All Defaults, scale set to 245 dps
    	self.gyroGain = 0.00875

    	"""
    	CTRL_REG5_G Configuration:

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
    	"""
    	self.device.write8(self.CTRL_REG_5_G, 0x00) # All Defaults

    # Returns x Gyro Data
    def getxGyro(self):
    	xGyro_MSBs = self.device.readU8(self.OUT_X_H_G)
    	xGyro_LSBs = self.device.readU8(self.OUT_X_L_G)
    	xBitGyro = xGyro_MSBs << 8 | xGyro_LSBs

    	if xBitGyro > 32767:
    		xBitGyro -= 65536

    	return xBitGyro * self.gyroGain

    # Returns y Gyro Data
    def getyGyro(self):
    	yGyro_MSBs = self.device.readU8(self.OUT_Y_H_G)
    	yGyro_LSBs = self.device.readU8(self.OUT_Y_L_G)
    	yBitGyro = yGyro_MSBs << 8 | yGyro_LSBs

    	if yBitGyro > 32767:
    		yBitGyro -= 65536

    	return yBitGyro * self.gyroGain

    # Returns z Gyro Data
    def getzGyro(self):
    	zGyro_MSBs = self.device.readU8(self.OUT_Z_H_G)
    	zGyro_LSBs = self.device.readU8(self.OUT_Z_L_G)
    	zBitGyro = zGyro_MSBs << 8 | zGyro_LSBs

    	if zBitGyro > 32767:
    		zBitGyro -= 65536

    	return zBitGyro * self.gyroGain
