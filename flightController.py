import Adafruit_Python_GPIO.Adafruit_GPIO.I2C as i2c

# Addresses for the XM and G when the SCL/SDA lines are pulled up
XM_ADDRESS = 0x1D
G_ADDRESS = 0x6B

LSM_XM = i2c.get_i2c_device(XM_ADDRESS)
LSM_G = i2c.get_i2c_device(G_ADDRESS)

# Class Definition for the Accelerometer/Magnetometer part of the LSM9DS0
class LSM9DS0_XM:

    """
    Accelerometer/Magnetometer Bit Registers
    For Reference and for Register Configurations/Restrictions: https://cdn-shop.adafruit.com/datasheets/LSM9DS0.pdf
    ***In the register names: L is for when the pins are connected LOW. H for HIGH.***
    """

    # Temperature Sensor Data
    OUT_TEMP_L_XM = 0x05 # Low
    OUT_TEMP_H_XM = 0x06 # High

    STATUS_REG_M = 0x07

    # Magnetic Data Values. 16 bit, two's complement, left-justified
    OUT_X_L_M = 0x08 # X
    OUT_X_H_M = 0x09
    OUT_Y_L_M = 0x0A # Y
    OUT_Y_H_M = 0x0B
    OUT_Z_L_M = 0x0C # Z
    OUT_Z_H_M = 0x0D

    WHO_AM_I_XM = 0x0F # Device Identification for Accel/Mag

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

# Class Definition for the Gyro part of the LSM9DS0
class LSM9DS0_G:

    """
    Gyro Bit Registers
    For Reference and for Register Configurations/Restrictions: https://cdn-shop.adafruit.com/datasheets/LSM9DS0.pdf
    """
    
    WHO_AM_I_G = 0x0F # Device Identification for Gyro
    
    CTRL_REG_1_G = 0x20 
    CTRL_REG_2_G = 0x21
    CTRL_REG_3_G = 0x22
    CTRL_REG_4_G = 0x23
    CTRL_REG_5_G = 0x24
    
    DATACAPTURE_G = 0x25
    
    STATUS_REG_G = 0x27

    # L is for when the pins are connected LOW, H for HIGH
    OUT_X_L_G = 0x28 # X-axis angular rate data
    OUT_X_H_G = 0x29
    OUT_Y_L_G = 0x2A # Y-axis angular rate data
    OUT_Y_H_G = 0x2B
    OUT_Z_L_G = 0x2C # Z-axis angular rate data
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

