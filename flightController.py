"""
*** FLIGHT CONTROLLER ***

Albert Li | 2017

Description: Flight Controller for a Raspberry Pi-controlled quadcopter

The constants that users should tune to change controller performance are at the
top of this file. In particular, the declination angle is time AND location
dependent, so if you are using this code in the future, change that parameter.
Use this webapp: https://www.ngdc.noaa.gov/geomag-web/

Sensors:
(1) LSM9DS0 9 DOF Accel/Gyro/Mag Board | Using I2C for communication/configuration

	*** CONFIGURATION ***
	You must manually enter the hex code corresponding to the desired settings
	in the control bit registers. The accelerometer and magnetometer seem to be
	on one shared board, while the gyro is on its own

	The LSM9DS0 class combines both of the separated sensor classes. All the sensor
    calibration methods are in this class.

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
DECLINATION_ANGLE = 13.55 # [USER-DEFINED] Declination angle (deg) for Berkeley, CA | Summer of 2017

#####################
# LSM9DS0 CONSTANTS #
#####################

# I2C Addresses for the XM and G
XM_ADDRESS = 0x1D
G_ADDRESS = 0x6B

# [USER-DEFINED CONSTANTS HERE]
TEMP_INTERCEPT = 24.0 # Guess at the intercept for the temperature sensor. Probably shouldn't change.

MAG_CALIB_SAMPLES = 10000 # Use 10000 magnetometer samples to calibrate for the hard-iron effect
GYRO_CALIB_SAMPLES = 5000 # Use 5000 gyro samples to calculate its offset
ACCEL_CALIB_SAMPLES = 1000 # Use 1000 samples per position to calibrate for 0g offset

BETA = 12.5 # Beta value for Madgwick filter
ZETA = 0.01 # Zeta value for Madgwick filter

#################
# GPS CONSTANTS #
#################

UPDATE_10HZ_CODE = "$PMTK220,100*2F\r\n" # PMTK code for 10Hz update rate
BAUDRATE_115200_CODE = "$PMTK251,115200*1F\r\n" # PMTK code for 115200 bps baudrate

#################
# CLASSES BELOW #
#################

# The complete Attitude Heading and Reference System class
class AHRS:

    def __init__(self):

        # The sensors that make up our AHRS
        self.lsm = LSM9DS0()
        self.gps = GPS()

        # Euler angles
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

    def updateEulerAngles(self):

        # LSM9DS0 orientation quaternion values
        q1 = self.lsm.SEq1
        q2 = self.lsm.SEq2
        q3 = self.lsm.SEq3
        q4 = self.lsm.SEq4

        self.yaw = atan2(2 * (q2 * q3 - q1 * q4), 2 * (q1 * q1 + q2 * q2) - 1)
        self.pitch = asin(2 * (q1 * q3 - q2 * q4))
        self.roll = atan2(2 * (q1 * q2 + q3 * q4), 1 - 2 * (q2 * q2 + q3 * q3))

        # Convert to degrees for readability
        # The roll here is a little weird - when I converted our coordinates to NED,
        # the roll axis flipped upside-down (which isn't super surprising, maybe). 
        # This is the best way to deal with it for now because we still get the right
        # Euler angles.
        self.yaw *= 180 / PI
        self.yaw += DECLINATION_ANGLE # Correcting for declination.
        self.pitch *= 180 / PI
        self.roll = 180 - (self.roll * 180 / PI)
        if self.roll > 180:
            self.roll -= 360

# The GPS class
class GPS:

    def __init__(self):
        self.gpsSer = serial.Serial("/dev/ttyS0", 9600) # Setting up GPS serial with baud rate of 9600 bps

        # Initialize the GPS module to a baudrate of 115200 bps and 10Hz update rate
        self.gpsSer.write(BAUDRATE_115200_CODE.encode())
        time.sleep(1)
        self.gpsSer.baudrate = 115200
        self.gpsSer.write(UPDATE_10HZ_CODE.encode())
        time.sleep(1)

        # GPS variables. We can use numSats and hdop to perhaps dynamically weight our complementary speed filter.
        self.lat = 0.0 # (+) N, (-) S in degrees
        self.long = 0.0 # (+) E, (-) W in degrees
        self.speed = 0.0 # NOT velocity. Measured in m/s
        self.cmg = 0.0 # Course made good, degrees from true north (basically direction for speed vector)
        self.numSats = 0 # How many satellites we have fixes for (the more the better)
        self.hdop = 0 # Horizontal dilution of precision (lower the better)

    # Starts the GPS polling process
    def startGPS(self):
        # We receive 4 different sentence types that we can parse: 

        # GPGGA - Time of Fix, Lat, Long, Fix Qual, Sats Tracked, HDOP, Alt (m) Above Mean Sea Level, Height of Geoid, Time Since Updating
        # GPGSA - A/M Fix Selection, 3D Fix, PRN of Fix Sats, PDOP, HDOP, VDOP
        # GPRMC - Time of Fix, A/V Status, Lat, Long, Speed (Knots), Track Angle (Deg), Date, Mag Variation
        # GPVTG - True Track Made Good, Magnetic Track Made Good, Speed (Knots), Speed (KM/H)

        # We basically can survive off of just GPRMC sentences, but it's useful to parse other sentences for potential weighting
        try:
            while True:
                self.gpsSer.flushInput()

                while self.gpsSer.inWaiting() > 0:
                    line = self.gpsSer.readline().decode() # String of decoded data
                    lineData = line.split(",")

                    # Parsing GPRMC Sentences
                    if lineData[0] == "$GPRMC":

                        # Parsing Latitude
                        latString = lineData[3]

                        if len(latString) > 0:
                            lat1 = float(latString[0:2])
                            lat2 = float(latString[2:]) / 60.0
                            self.lat = lat1 + lat2

                            if lineData[4] == "S":
                                self.lat *= -1

                        else:
                            continue

                        # Parsing Longitude
                        longString = lineData[5]

                        if len(longString) > 0:
                            long1 = float(longString[0:3])
                            long2 = float(longString[3:]) / 60.0
                            self.long = long1 + long2

                            if lineData[6] == "W":
                                self.long *= -1

                        else:
                            continue

                        # Parsing speed
                        if len(lineData[7]) > 0:
                            speedKnots = float(lineData[7])
                            self.speed = speedKnots * .51444444
                        else:
                            continue

                        # Parsing course-made-good
                        if len(lineData[8]) > 0:
                            self.cmg = float(lineData[8])
                        else:
                            continue

                    # Parsing GPGGA Sentences
                    elif lineData[0] == "$GPGGA":
                        # Parsing the number of satellites
                        if len(lineData[7]) > 0:
                            self.numSats = int(lineData[7])
                        else:
                            continue

                    # Parsing GPGSA Sentences
                    elif lineData[0] == "$GPGSA":
                        # Parsing the HDOP
                        if len(lineData[16]) > 0:
                            self.hdop = float(lineData[16])
                        else:
                            continue

                    # Parsing GPVTG Sentences
                    elif lineData[0] == "$GPVTG":
                        pass

                    # Probably garbage bytes. Flush the buffer and continue polling.
                    else:
                        self.gpsSer.flushInput()
                        continue

                    # Debug Prints in Order: GPRMC, GPGGA, GPGSA
                    print("Lat = " + str(self.lat) + " | Long = " + str(self.long) + " | Speed = " + str(self.speed) + " | CMG = " + str(self.cmg))
                    print("Num Sats = " + str(self.numSats))
                    print("HDOP = " + str(self.hdop))
                    print()

        except KeyboardInterrupt:
            print("GPS Polling Stopped!")
            pass

        # This happens when an incomplete transmission is made (it will eventually happen!)
        except Exception:
            # If there's an error, flush the buffer and continue trying to poll
            self.gpsSer.flushInput()
            startGPS()

    # For debugging the GPS stream
    def printRawData(self):
        try:
            while True:
                # Flush to clear garbage bytes
                self.gpsSer.flushInput()

                while self.gpsSer.inWaiting() > 0:
                    data = self.gpsSer.readline()

                    # Different print options
                    print(data.decode())
                    # print(data.decode().split(","))

        except KeyboardInterrupt:
            print("\nStream Interrupted!")

# Combined IMU class
class LSM9DS0:

    def __init__(self):
        # Sensor components
        self.xm = LSM9DS0_XM()
        self.g = LSM9DS0_G()

        # Timing for sampling
        self.prevTime = 0

        # Hard-Iron/Soft-Iron Values (Tune this with the calibrateHardSoftIronEffect() method!)
        # These are values that I tested myself, but you should calibrate right before flight.
        # If you do a manual calibration test, please UPDATE THESE VALUES!
        self.X_HI_OFFSET = -0.06
        self.Y_HI_OFFSET = -0.06
        self.Z_HI_OFFSET = 0.08

        self.X_SI_SCALE = 1
        self.Y_SI_SCALE = 0.95
        self.Z_SI_SCALE = 1.1

        # Gyro bias offsets (Tune this with the calibrateGyroOffsets() method!)
        # These are values that I tested myself, but you should calibrate right before flight.
        # If you do a manual calibration test, please UPDATE THESE VALUES!
        self.X_GB_OFFSET = 0.5
        self.Y_GB_OFFSET = -0.3
        self.Z_GB_OFFSET = -4.5

        # Accelerometer bias offsets (Tune this with the calibrateGyroOffsets() method!)
        # These are values that I tested myself, but you should calibrate right before flight.
        # If you do a manual calibration test, please UPDATE THESE VALUES!
        # [NOTE] These values seem to vary a lot more between tests
        self.X_AB_OFFSET = -0.4
        self.Y_AB_OFFSET = -0.016
        self.Z_AB_OFFSET = -0.23

        # ONLY FOR DEBUGGING - Locally calculated Euler angles
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

    # Activating the sensor
    def startLSM(self):
        if self.firstTime:
            self.prevTime = time.clock()
            self.ax = self.xm.getxAccel() - self.X_AB_OFFSET
            self.ay = self.xm.getyAccel() - self.Y_AB_OFFSET
            self.az = -(self.xm.getzAccel() - self.Z_AB_OFFSET)
            self.mx = (self.xm.getxMag() - self.X_HI_OFFSET) * self.X_SI_SCALE
            self.my = (self.xm.getyMag() - self.Y_HI_OFFSET) * self.Y_SI_SCALE
            self.mz = (self.xm.getzMag() - self.Z_HI_OFFSET) * self.Z_SI_SCALE
            self.wx = self.g.getxGyro() - self.X_GB_OFFSET
            self.wy = self.g.getyGyro() - self.Y_GB_OFFSET
            self.wz = self.g.getzGyro() - self.Z_GB_OFFSET

            self.firstTime = False

            self.startTime = self.prevTime
            self.lastPrintTime = self.prevTime

        try:
            while True:
                self.madgwickFilterUpdate()

                # Local Euler angle calculations for sensor debugging. Uncomment to use.

                
                # Calculating Euler angles locally
                self.yaw = atan2(2 * (self.SEq2 * self.SEq3 - self.SEq1 * self.SEq4), 2 * (self.SEq1 * self.SEq1 + self.SEq2 * self.SEq2) - 1)
                self.pitch = asin(2 * (self.SEq1 * self.SEq3 - self.SEq2 * self.SEq4))
                self.roll = atan2(2 * (self.SEq1 * self.SEq2 + self.SEq3 * self.SEq4), 1 - 2 * (self.SEq2 * self.SEq2 + self.SEq3 * self.SEq3))

                # Convert to degrees for readability
                self.yaw *= 180 / PI
                self.yaw += DECLINATION_ANGLE
                self.pitch *= 180 / PI
                self.roll = 180 - (self.roll * 180 / PI)
                if self.roll > 180:
                    self.roll -= 360

                # Debug print statements
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

        # Update Values - fixed for NED frame, hard-iron/soft-iron effect, and gyro bias
        self.ax = self.xm.getxAccel() - self.X_AB_OFFSET
        self.ay = self.xm.getyAccel() - self.Y_AB_OFFSET
        self.az = -(self.xm.getzAccel() - self.Z_AB_OFFSET)
        self.mx = (self.xm.getxMag() - self.X_HI_OFFSET) * self.X_SI_SCALE
        self.my = (self.xm.getyMag() - self.Y_HI_OFFSET) * self.Y_SI_SCALE
        self.mz = (self.xm.getzMag() - self.Z_HI_OFFSET) * self.Z_SI_SCALE
        self.wx = self.g.getxGyro() - self.X_GB_OFFSET
        self.wy = self.g.getyGyro() - self.Y_GB_OFFSET
        self.wz = self.g.getzGyro() - self.Z_GB_OFFSET

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

    # For calibration of accelerometer bias. Probably doesn't need to be run on startup, but is useful for the developer.
    def calibrateAccelOffsets(self):
        # Use this calibration protocol (pg 3): http://kionixfs.kionix.com/en/document/AN012%20Accelerometer%20Errors.pdf
        n = 0

        ax1 = 0
        ax3 = 0
        ax5 = 0
        ax6 = 0

        ay2 = 0
        ay4 = 0
        ay5 = 0
        ay6 = 0

        az1 = 0
        az2 = 0
        az3 = 0
        az4 = 0

        print("*** ACCELEROMETER CALIBRATION PROTOCOL STARTED ***")
        print("There are going to be six positions to orient the sensor in. It must be still.")
        print("It may help to have a corner so you are as close to perpendicular as possible.\n")

        # POSITION 1
        print("Position 1 is like such:")
        print("-------")
        print("|.    |")
        print("|     |")
        print("|     |")
        print("-------")
        print("Press ENTER when ready.")
        input()
        print("Gathering data. Please wait...\n")

        while n < ACCEL_CALIB_SAMPLES:
            n += 1
            ax1 += self.xm.getxAccel()
            az1 += self.xm.getzAccel()

        n = 0
        ax1 /= ACCEL_CALIB_SAMPLES
        az1 /= ACCEL_CALIB_SAMPLES

        # POSITION 2
        print("Position 2 is like such:")
        print("|----------|")
        print("|          |")
        print("|.         |")
        print("|----------|")
        print("Press ENTER when ready.")
        input()
        print("Gathering data. Please wait...\n")

        while n < ACCEL_CALIB_SAMPLES:
            n += 1
            ay2 += self.xm.getyAccel()
            az2 += self.xm.getzAccel()

        n = 0
        ay2 /= ACCEL_CALIB_SAMPLES
        az2 /= ACCEL_CALIB_SAMPLES

        # POSITION 3
        print("Position 3 is like such:")
        print("-------")
        print("|     |")
        print("|     |")
        print("|    .|")
        print("-------")
        print("Press ENTER when ready.")
        input()
        print("Gathering data. Please wait...\n")

        while n < ACCEL_CALIB_SAMPLES:
            n += 1
            ax3 += self.xm.getxAccel()
            az3 += self.xm.getzAccel()

        n = 0
        ax3 /= ACCEL_CALIB_SAMPLES
        az3 /= ACCEL_CALIB_SAMPLES

        # POSITION 4
        print("Position 4 is like such:")
        print("|----------|")
        print("|         .|")
        print("|          |")
        print("|----------|")
        print("Press ENTER when ready.")
        input()
        print("Gathering data. Please wait...\n")

        while n < ACCEL_CALIB_SAMPLES:
            n += 1
            ay4 += self.xm.getyAccel()
            az4 += self.xm.getzAccel()

        n = 0
        ay4 /= ACCEL_CALIB_SAMPLES
        az4 /= ACCEL_CALIB_SAMPLES

        # POSITION 5
        print("Position 5 is like such:")
        print("    TOP    ")
        print("-.---------")
        print("|         |")
        print("-----------")
        print("   BOTTOM  ")
        print("Press ENTER when ready.")
        input()
        print("Gathering data. Please wait...\n")

        while n < ACCEL_CALIB_SAMPLES:
            n += 1
            ax5 += self.xm.getxAccel()
            ay5 += self.xm.getyAccel()

        n = 0
        ax5 /= ACCEL_CALIB_SAMPLES
        ay5 /= ACCEL_CALIB_SAMPLES

        # POSITION 6
        print("Position 6 is like such:")
        print("   BOTTOM  ")
        print("-----------")
        print("|         |")
        print("-.---------")
        print("    TOP    ")
        print("Press ENTER when ready.")
        input()
        print("Gathering data. Please wait...\n")

        while n < ACCEL_CALIB_SAMPLES:
            n += 1
            ax6 += self.xm.getxAccel()
            ay6 += self.xm.getyAccel()

        n = 0
        ax6 /= ACCEL_CALIB_SAMPLES
        ay6 /= ACCEL_CALIB_SAMPLES

        # Calculating 0g biases
        self.X_AB_OFFSET = (ax1 + ax3 + ax5 + ax6) / 4
        self.Y_AB_OFFSET = (ay2 + ay4 + ay5 + ay6) / 4
        self.Z_AB_OFFSET = (az1 + az2 + az3 + az4) / 4

        print("Calibration complete!")
        print("We've already set these values for you in the system, but the offsets are printed for your convenience.\n")

        print("X Offset: " + str(self.X_AB_OFFSET))
        print("Y Offset: " + str(self.Y_AB_OFFSET))
        print("Z Offset: " + str(self.Z_AB_OFFSET))

    # For calibration of gyro bias. Probably doesn't need to be run on startup, but is useful for the developer.
    def calibrateGyroOffsets(self):
        n = 0

        sumx = 0
        sumy = 0
        sumz = 0

        print("*** GYROSCOPE CALIBRATION PROTOCOL STARTED ***")
        print("Please keep the device still for calibration. Press ENTER when ready.")
        input()
        print("Calibrating. Please wait...")

        while n < GYRO_CALIB_SAMPLES:
            n += 1
            sumx += self.g.getxGyro()
            sumy += self.g.getyGyro()
            sumz += self.g.getzGyro()

        self.X_GB_OFFSET = sumx / GYRO_CALIB_SAMPLES
        self.Y_GB_OFFSET = sumy / GYRO_CALIB_SAMPLES
        self.Z_GB_OFFSET = sumz / GYRO_CALIB_SAMPLES

        print("Calibration complete!")
        print("We've already set these values for you in the system, but the offsets are printed for your convenience.\n")

        print("X Offset: " + str(self.X_GB_OFFSET))
        print("Y Offset: " + str(self.Y_GB_OFFSET))
        print("Z Offset: " + str(self.Z_GB_OFFSET))

    # For calibration of hard-iron and soft-iron effects (magnetometer bias). Should probably run at startup every time.
    def calibrateHardSoftIronEffect(self):
        xmax = 0
        xmin = 0
        ymax = 0
        ymin = 0
        zmax = 0
        zmin = 0
        n = 0

        print("*** MAGNETOMETER CALIBRATION PROTOCOL STARTED ***")
        print("Please turn the device through the air in a figure 8 fashion until calibration finishes. Press ENTER when ready.")
        input()
        print("Calibrating. Continue turning...")

        while n < MAG_CALIB_SAMPLES:
            n += 1
            x = self.xm.getxMag()
            y = self.xm.getyMag()
            z = self.xm.getzMag()

            if x > xmax:
                xmax = x
            elif x < xmin:
                xmin = x

            if y > ymax:
                ymax = y
            elif y < ymin:
                ymin = y

            if z > zmax:
                zmax = z
            elif z < zmin:
                zmin = z

        xavg = (xmax + xmin) / 2
        yavg = (ymax + ymin) / 2
        zavg = (zmax + zmin) / 2

        self.X_HI_OFFSET = xavg
        self.Y_HI_OFFSET = yavg
        self.Z_HI_OFFSET = zavg

        allavg = (xmax - xmin + xmax - xmin + xmax - xmin) / 3

        self.X_SI_SCALE = abs(allavg / (xmax - xmin))
        self.Y_SI_SCALE = abs(allavg / (ymax - ymin))
        self.Z_SI_SCALE = abs(allavg / (zmax - zmin))

        print("Calibration complete!")
        print("We've already set these values for you in the system, but the offsets are printed for your convenience.\n")

        print("X Hard-Iron Offset: " + str(self.X_HI_OFFSET))
        print("Y Hard-Iron Offset: " + str(self.Y_HI_OFFSET))
        print("Z Hard-Iron Offset: " + str(self.Z_HI_OFFSET) + "\n")

        print("X Soft-Iron Scale: " + str(self.X_SI_SCALE))
        print("Y Soft-Iron Scale: " + str(self.Y_SI_SCALE))
        print("Z Soft-Iron Scale: " + str(self.Z_SI_SCALE))

    # Printing method - will print all raw sensor values at once
    def printRawData(self):
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
