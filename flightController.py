"""
*** FLIGHT CONTROLLER ***

Albert Li | 2017

Description: Flight Controller for a Raspberry Pi-controlled quadcopter

Sensors:
(1) LSM9DS0 9 DOF Accel/Gyro/Mag Board | Using I2C for communication/configuration

	*** CONFIGURATION ***
	You must manually enter the hex code corresponding to the desired settings
	in the control bit registers. The accelerometer and magnetometer seem to be
	on one shared board, while the gyro is on its own

	The LSM9DS0 class combines both of the separated sensor classes. 

(2) Adafruit Ultimate GPS Breakout (MTK3339 Chipset)
"""

import Adafruit_Python_GPIO.Adafruit_GPIO.I2C as i2c
import numpy as np
import time
from math import atan2, sqrt, asin
import serial

# Physical Constants
GRAV_ACCEL = 9.80665 # Value of acceleration due to gravity (m*s^-2)
PI = 3.14159265358979323846

#####################
# LSM9DS0 CONSTANTS #
#####################

# Addresses for the XM and G when the SCL/SDA lines are pulled up (THEY SHOULD ALWAYS BE)
XM_ADDRESS = 0x1D
G_ADDRESS = 0x6B

# Predefined Constants
TEMP_INTERCEPT = 24.0 # Guess at the intercept for the temperature sensor
MAG_CALIB_SAMPLES = 10000 # We want to use 10000 magnetometer samples to calibrate for the hard-iron effect
BETA = 12.5 # Beta value for Madgwick filter
ZETA = 0.01 # Zeta value for Madgwick filter

#################
# GPS CONSTANTS #
#################

UPDATE_10HZ_CODE = "$PMTK220,100*2F\r\n" # PMTK code for 10Hz update rate
BAUDRATE_115200_CODE = "$PMTK251,115200*1F\r\n"

# The GPS class
class GPS:

    def __init__(self):
        self.gpsSer = serial.Serial("/dev/ttyS0", 9600) # Setting up GPS serial with baud rate of 9600 bps

        self.gpsSer.write(BAUDRATE_115200_CODE.encode())
        time.sleep(1)
        self.gpsSer.baudrate = 115200
        self.gpsSer.write(UPDATE_10HZ_CODE.encode())
        time.sleep(1)

    def readRawData(self):
        try:
            while True:
                self.gpsSer.flushInput()
                while self.gpsSer.inWaiting() == 0:
                    pass
                data = self.gpsSer.readLine()
                print(data)
        except KeyboardInterrupt:
            print("Stream Interrupted!")

# Combined IMU class
class LSM9DS0:

    def __init__(self):
        # Sensor components
        self.xm = LSM9DS0_XM()
        self.g = LSM9DS0_G()

        # Timing for sampling
        self.prevTime = 0

        # Hard-Iron Offsets (Tune this with the calibrateHardIronEffect() method!)
        # These are values that I tested myself, but you should calibrate right before flight.
        X_HI_OFFSET = -0.05
        Y_HI_OFFSET = -0.045
        Z_HI_OFFSET = 0.2

        # Euler angles
        self.yaw = 0
        self.roll = 0
        self.pitch = 0

        ##############################
        # *** Madgwick Variables *** #
        ##############################

        # Initial orientation quaternion variables (Earth wrt sensor)
        self.SEq1 = 1
        self.SEq2 = 0
        self.SEq3 = 0
        self.SEq4 = 0

        # Sampling Time - Will change with every loop
        self.dt = .01
        self.firstTime = True

        self.xSampled = False
        self.mSampled = False
        self.gSampled = False

        # Setting zeta and beta
        self.beta = BETA
        self.zeta = ZETA

        # Dummy values for sensor readings
        self.ax = 0
        self.ay = 0
        self.az = 0
        self.mx = 0
        self.my = 0
        self.mz = 0
        self.wx = 0
        self.wy = 0
        self.wz = 0

        # Earth mag field reference
        self.bx = 1
        self.bz = 0

        # Gyro bias error estimates
        self.wbx = 0
        self.wby = 0
        self.wbz = 0

        # DEBUG
        self.lastPrintTime = 0
        self.startTime = 0

    ################################################################################################################
    # This algorithm adapted from Madgwick's provided code: http://x-io.co.uk/res/doc/madgwick_internal_report.pdf #
    #                                                                                                              #
    # An error was corrected: define dmx, dmy, and dmz AFTER normalizing the fields!                               #
    ################################################################################################################
    def madgwickFilterUpdate(self):

        # Update Time
        currTime = time.clock()
        self.dt = currTime - self.prevTime
        self.prevTime = currTime

        # Update Values - fixed for NED frame and hard-iron effect
        self.ax = self.xm.getxAccel()
        self.ay = self.xm.getyAccel()
        self.az = -self.xm.getzAccel()
        self.mx = self.xm.getxMag() - X_HI_OFFSET
        self.my = self.xm.getyMag() - Y_HI_OFFSET
        self.mz = self.xm.getzMag() - Z_HI_OFFSET
        self.wx = self.g.getxGyro()
        self.wy = self.g.getyGyro()
        self.wz = self.g.getzGyro()

        #################################
        # Useful Variable Manipulations #
        #################################
        hSEq1 = 0.5 * self.SEq1
        hSEq2 = 0.5 * self.SEq2
        hSEq3 = 0.5 * self.SEq3
        hSEq4 = 0.5 * self.SEq4

        dSEq1 = 2 * self.SEq1
        dSEq2 = 2 * self.SEq2
        dSEq3 = 2 * self.SEq3
        dSEq4 = 2 * self.SEq4

        sSEq3 = self.SEq3 * self.SEq3

        dbx = 2 * self.bx
        dbz = 2 * self.bz

        dbxSEq1 = dbx * self.SEq1
        dbxSEq2 = dbx * self.SEq2
        dbxSEq3 = dbx * self.SEq3
        dbxSEq4 = dbx * self.SEq4
        dbzSEq1 = dbz * self.SEq1
        dbzSEq2 = dbz * self.SEq2
        dbzSEq3 = dbz * self.SEq3
        dbzSEq4 = dbz * self.SEq4

        SEq1SEq3 = self.SEq1 * self.SEq3
        SEq2SEq4 = self.SEq2 * self.SEq4

        ##########################
        # Beginning of Algorithm #
        ##########################

        # Normalize acceleration and magnetometer values
        tempNorm = sqrt(self.ax * self.ax + self.ay * self.ay + self.az * self.az)
        self.ax /= tempNorm
        self.ay /= tempNorm
        self.az /= tempNorm

        tempNorm = sqrt(self.mx * self.mx + self.my * self.my + self.mz * self.mz)
        self.mx /= tempNorm
        self.my /= tempNorm
        self.mz /= tempNorm

        dmx = 2 * self.mx
        dmy = 2 * self.my
        dmz = 2 * self.mz

        # Combined cost function + Jacobian
        # Functions from g-field
        f1 = dSEq2 * self.SEq4 - dSEq1 * self.SEq3 - self.ax
        f2 = dSEq1 * self.SEq2 + dSEq3 * self.SEq4 - self.ay
        f3 = 1 - dSEq2 * self.SEq2 - dSEq3 * self.SEq3 - self.az

        # Functions from b-field
        f4 = dbx * (0.5 - sSEq3 - self.SEq4 * self.SEq4) + dbz * (SEq2SEq4 - SEq1SEq3) - self.mx
        f5 = dbx * (self.SEq2 * self.SEq3 - self.SEq1 * self.SEq4) + dbz * (self.SEq1 * self.SEq2 + self.SEq3 * self.SEq4) - self.my
        f6 = dbx * (SEq1SEq3 + SEq2SEq4) + dbz * (0.5 - self.SEq2 * self.SEq2 - sSEq3) - self.mz

        # Jacobian entries
        J1124 = dSEq3
        J1223 = dSEq4
        J1322 = dSEq1
        J1421 = dSEq2
        J32 = 2 * J1421
        J33 = 2 * J1124
        J41 = dbzSEq3
        J42 = dbzSEq4
        J43 = 2 * dbxSEq3 + dbzSEq1
        J44 = 2 * dbxSEq4 - dbzSEq2
        J51 = dbxSEq4 - dbzSEq2
        J52 = dbxSEq3 + dbzSEq1
        J53 = dbxSEq2 + dbzSEq4
        J54 = dbxSEq1 - dbzSEq3
        J61 = dbxSEq3
        J62 = dbxSEq4 - 2 * dbzSEq2
        J63 = dbxSEq1 - 2 * dbzSEq3
        J64 = dbxSEq2

        # Gradient Descent Optimization
        # Gradients
        SEqhatdot1 = -J1124 * f1 + J1421 * f2 - J41 * f4 - J51 * f5 + J61 * f6
        SEqhatdot2 = J1223 * f1 + J1322 * f2 - J32 * f3 + J42 * f4 + J52 * f5 + J62 * f6
        SEqhatdot3 = -J1322 * f1 + J1223 * f2 - J33 * f3 - J43 * f4 + J53 * f5 + J63 * f6
        SEqhatdot4 = J1421 * f1 + J1124 * f2 - J44 * f4 - J54 * f5 + J64 * f6

        # Normalizing Gradients
        tempNorm = sqrt(SEqhatdot1 * SEqhatdot1 + SEqhatdot2 * SEqhatdot2 + SEqhatdot3 * SEqhatdot3 + SEqhatdot4 * SEqhatdot4)
        SEqhatdot1 /= tempNorm
        SEqhatdot2 /= tempNorm
        SEqhatdot3 /= tempNorm
        SEqhatdot4 /= tempNorm

        # Angular estimated direction of gyro error
        wex = dSEq1 * SEqhatdot2 - dSEq2 * SEqhatdot1 - dSEq3 * SEqhatdot4 + dSEq4 * SEqhatdot3
        wey = dSEq1 * SEqhatdot3 + dSEq2 * SEqhatdot4 - dSEq3 * SEqhatdot1 - dSEq4 * SEqhatdot2
        wez = dSEq1 * SEqhatdot4 - dSEq2 * SEqhatdot3 + dSEq3 * SEqhatdot2 - dSEq4 * SEqhatdot1

        # Remove gyro bias
        self.wbx += wex * self.dt * self.zeta
        self.wby += wey * self.dt * self.zeta
        self.wbz += wez * self.dt * self.zeta
        self.wx -= self.wbx
        self.wy -= self.wby
        self.wz -= self.wbz

        # Quaternion rate of change (gyro)
        SEqdot1 = -hSEq2 * self.wx - hSEq3 * self.wy - hSEq4 * self.wz
        SEqdot2 = hSEq1 * self.wx + hSEq3 * self.wz - hSEq4 * self.wy
        SEqdot3 = hSEq1 * self.wy - hSEq2 * self.wz + hSEq4 * self.wx
        SEqdot4 = hSEq1 * self.wz + hSEq2 * self.wy - hSEq3 * self.wx

        # Update orientation quaternion
        self.SEq1 += (SEqdot1 - (self.beta * SEqhatdot1)) * self.dt
        self.SEq2 += (SEqdot2 - (self.beta * SEqhatdot2)) * self.dt
        self.SEq3 += (SEqdot3 - (self.beta * SEqhatdot3)) * self.dt
        self.SEq4 += (SEqdot4 - (self.beta * SEqhatdot4)) * self.dt

        # Normalize orientation quaternion
        tempNorm = sqrt(self.SEq1 * self.SEq1 + self.SEq2 * self.SEq2 + self.SEq3 * self.SEq3 + self.SEq4 * self.SEq4)
        self.SEq1 /= tempNorm
        self.SEq2 /= tempNorm
        self.SEq3 /= tempNorm
        self.SEq4 /= tempNorm

        # b-field in earth frame
        SEq1SEq2 = self.SEq1 * self.SEq2
        SEq1SEq3 = self.SEq1 * self.SEq3
        SEq1SEq4 = self.SEq1 * self.SEq4
        SEq3SEq4 = self.SEq3 * self.SEq4
        SEq2SEq3 = self.SEq2 * self.SEq3
        SEq2SEq4 = self.SEq2 * self.SEq4

        hx = dmx * (0.5 - self.SEq3 * self.SEq3 - self.SEq4 * self.SEq4) + dmy * (SEq2SEq3 - SEq1SEq4) + dmz * (SEq2SEq4 + SEq1SEq3)
        hy = dmx * (SEq2SEq3 + SEq1SEq4) + dmy * (0.5 - self.SEq2 * self.SEq2 - self.SEq4 * self.SEq4) + dmz * (SEq3SEq4 - SEq1SEq2)
        hz = dmx * (SEq2SEq4 - SEq1SEq3) + dmy * (SEq3SEq4 + SEq1SEq2) + dmz * (0.5 - self.SEq2 * self.SEq2 - self.SEq3 * self.SEq3)

        # Normalize flux vector to eliminate y component
        self.bx = sqrt(hx * hx + hy * hy)
        self.bz = hz

    # Activating the sensor
    def activateSensor(self):
        if self.firstTime:
            self.prevTime = time.clock()
            self.ax = self.xm.getxAccel()
            self.ay = self.xm.getyAccel()
            self.az = self.xm.getzAccel()
            self.mx = self.xm.getxMag()
            self.my = self.xm.getyMag()
            self.mz = self.xm.getzMag()
            self.wx = self.g.getxGyro()
            self.wy = self.g.getyGyro()
            self.wz = self.g.getzGyro()

            self.firstTime = False

            self.startTime = self.prevTime
            self.lastPrintTime = self.prevTime

        try:
            while True:
                self.madgwickFilterUpdate()

                self.yaw = atan2(2 * (self.SEq2 * self.SEq3 - self.SEq1 * self.SEq4), 2 * (self.SEq1 * self.SEq1 + self.SEq2 * self.SEq2) - 1)
                self.pitch = asin(2 * (self.SEq1 * self.SEq3 - self.SEq2 * self.SEq4))
                self.roll = atan2(2 * (self.SEq1 * self.SEq2 + self.SEq3 * self.SEq4), 1 - 2 * (self.SEq2 * self.SEq2 + self.SEq3 * self.SEq3))

                # Convert to degrees for readability
                self.yaw *= 180 / PI
                self.pitch *= 180 / PI
                self.roll = 180 - (roll * 180 / PI)
                if self.roll > 180:
                    self.roll -= 360

                # Print every ~.25 seconds
                now = time.clock()
                if now - self.lastPrintTime >= 0.25:
                    print("Time: " + str(now - self.startTime))
                    print(" | dt: " + str(self.dt), end = "")
                    print(" | Yaw (No reference): " + str(self.yaw), end = "")
                    print(" | Pitch: " + str(self.pitch), end = "")
                    print(" | Roll: " + str(self.roll))

                    self.lastPrintTime = now

        except KeyboardInterrupt:
            print("Exited Test")

    def calibrateHardIronEffect(self):
        # Here, I use an interesting and obscure regression method
        # for solving for the equation of a sphere from a cloud
        # of data points (pg 17-18): 
        # https://www.scribd.com/document/14819165/Regressions-coniques-quadriques-circulaire-spherique

        # Least Squares Variables
        n = 0
        sumxk = 0
        sumyk = 0
        sumzk = 0
        sumxksq = 0
        sumyksq = 0
        sumzksq = 0
        sumxkyk = 0
        sumxkzk = 0
        sumykzk = 0
        sumpksq = 0
        sumpksqxk = 0
        sumpksqyk = 0
        sumpksqzk = 0

        try:
            while n < MAG_CALIB_SAMPLES:
                xk = self.xm.getxMag()
                yk = self.xm.getyMag()
                zk = self.xm.getzMag()

                # Auxillary variable
                pksq = xk * xk + yk * yk + zk * zk

                n += 1
                sumxk += xk
                sumyk += yk
                sumzk += zk
                sumxksq += xk * xk
                sumyksq += yk * yk
                sumzksq += zk * zk
                sumxkyk += xk * yk
                sumxkzk += xk * zk
                sumykzk += yk * zk
                sumpksq += pksq
                sumpksqxk += pksq * xk
                sumpksqyk += pksq * yk
                sumpksqzk += pksq * zk

        except KeyboardInterrupt:
            print("Calibration Interrupted!")

        # Calculating the A values
        Amat = np.array([[n, sumxk, sumyk, sumzk],
                      [sumxk, sumxksq, sumxkyk, sumxkzk],
                      [sumyk, sumxkyk, sumyksq, sumykzk],
                      [sumyk, sumxkzk, sumykzk, sumzksq]])
        b = np.array([-sumpksq, -sumpksqxk, -sumpksqyk, -sumpksqzk])
        A0, A1, A2, A3 = np.linalg.solve(Amat, b)

        x0 = -A1 / 2
        y0 = -A2 / 2
        z0 = -A3 / 2
        R = sqrt(x0 * x0 + y0 * y0 + z0 * z0 - A0)

        # Debug print statements
        """
        print(Amat)
        print(A0)
        print(A1)
        print(A2)
        print(A3)
        print("x0 = " + str(x0))
        print("y0 = " + str(y0))
        print("z0 = " + str(z0))
        print("R = " + str(R))
        """

    # Printing method - will print all sensor values at once
    def printData(self):
        xacc = self.xm.getxAccel()
        yacc = self.xm.getyAccel()
        zacc = self.xm.getzAccel()
        xmag = self.xm.getxMag()
        ymag = self.xm.getyMag()
        zmag = self.xm.getzMag()
        xgyr = self.g.getxGyro()
        ygyr = self.g.getyGyro()
        zgyr = self.g.getzGyro()
        t = self.xm.getTemp()

        print("X Accel: " + str(xacc))
        print("Y Accel: " + str(yacc))
        print("Z Accel: " + str(zacc))
        print("X Mag: " + str(xmag))
        print("Y Mag: " + str(ymag))
        print("Z Mag: " + str(zmag))
        print("X Gyro: " + str(xgyr))
        print("Y Gyro: " + str(ygyr))
        print("Z Gyro: " + str(zgyr))
        print("Temp: " + str(t))
        print()


##############################
# SENSOR CONFIGURATION BELOW #
##############################


# Class Definition for the Accelerometer/Magnetometer part of the LSM9DS0
class LSM9DS0_XM:

    ###################################################################
    # ACCELEROMETER/MAGNETOMETER BIT REGISTERS                        #
    # Reference: https://cdn-shop.adafruit.com/datasheets/LSM9DS0.pdf #
    # *** Register names: L Stands for LOW BYTE, H for HIGH ***       #
    ###################################################################

    # Temperature Sensor Data. 12 bit, two's complement, right-justified
    OUT_TEMP_L_XM = 0x05 # Low - 8 bits are the 8 LSBs.
    OUT_TEMP_H_XM = 0x06 # High - Last 4 bits are MSBs. First 4 are useless, need to be masked out

    # Magnetometer Status Info (OVERRUN AND AVAILABILITY)
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

    # Interrupt Registers
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

    # Accelerometer Status Info (OVERRUN AND AVAILABILITY)
    STATUS_REG_A = 0x27

    # Acceleration Data Values. 16 bit, two's complement, left-justified
    OUT_X_L_A = 0x28
    OUT_X_H_A = 0x29
    OUT_Y_L_A = 0x2A
    OUT_Y_H_A = 0x2B
    OUT_Z_L_A = 0x2C
    OUT_Z_H_A = 0x2D

    # FIFO Registers
    FIFO_CTRL_REG = 0x2E
    FIFO_SRC_REG = 0x2F

    # Inertial Interrupt Generator 1 Registers
    INT_GEN_1_REG = 0x30
    INT_GEN_1_SRC = 0x31
    INT_GEN_1_THS = 0x32
    INT_GEN_1_DURATION = 0x33

    # Inertial Interrupt Generator 2 Registers
    INT_GEN_2_REG = 0x34
    INT_GEN_2_SRC = 0x35
    INT_GEN_2_THS = 0x36
    INT_GEN_2_DURATION = 0x37

    # Interrupt Click Registers
    CLICK_CFG = 0x38
    CLICK_SRC = 0x39
    CLICK_THS = 0x3A

    # Time Data Registers
    TIME_LIMIT = 0x3B
    TIME_LATENCY = 0x3C
    TIME_WINDOW = 0x3D

    # Activation Registers
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

    # Returns x Magnetometer Data (gauss)
    def getxMag(self):
    	# 16 Bit Precision, Left-Justified
    	xMag_MSBs = self.device.readU8(self.OUT_X_H_M)
    	xMag_LSBs = self.device.readU8(self.OUT_X_L_M)
    	xBitMag = xMag_MSBs << 8 | xMag_LSBs

    	if xBitMag > 32767:
    		xBitMag -= 65536

    	return xBitMag * self.magGain

    # Returns y Magnetometer Data (gauss)
    def getyMag(self):
    	# 16 Bit Precision, Left-Justified
    	yMag_MSBs = self.device.readU8(self.OUT_Y_H_M)
    	yMag_LSBs = self.device.readU8(self.OUT_Y_L_M)
    	yBitMag = yMag_MSBs << 8 | yMag_LSBs

    	if yBitMag > 32767:
    		yBitMag -= 65536

    	return yBitMag * self.magGain

    # Returns z Magnetometer Data (gauss)
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
    # GYRO BIT REGISTERS 					      #
    # Reference: https://cdn-shop.adafruit.com/datasheets/LSM9DS0.pdf #
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
    
    # References for Interrupts
    DATACAPTURE_G = 0x25
    
    # Gyro Status Info (OVERRUN AND AVAILABILITY)
    STATUS_REG_G = 0x27

    # Angular Rate Data
    OUT_X_L_G = 0x28 # X
    OUT_X_H_G = 0x29
    OUT_Y_L_G = 0x2A # Y
    OUT_Y_H_G = 0x2B
    OUT_Z_L_G = 0x2C # Z
    OUT_Z_H_G = 0x2D

    # FIFO Registers
    FIFO_CTRL_REG_G = 0x2E
    FIFO_SRC_REG_G = 0x2F

    # Interrupt Registers
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
