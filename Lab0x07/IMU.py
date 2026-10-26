from micropython import const
from struct import calcsize, unpack_from, pack
from time import sleep_ms
from pyb import I2C

IMU_I2C_BUS = const(2)  # where PB10 = SCL and PB11 = SDA

class BNO055:
    DEV_ADDR = const(0x28) #may be 29 if this doesnt work

    #register format definitions. Each store register address and and byte format string
    class reg:
        EUL_HEADING_MSB = (const(0x1B), b"<B") #most significant byte
        EUL_HEADING_LSB = (const(0x1A), b"<B") #least significant byte
        EUL_HEADING = (const(0x1A), b"<h")
        EUL_DATA_ALL = (const(0x1A), b"<hhh")
        GYRO_Z = (const(0x18), b"<h") #signed 16 bit
        CALIB_STATUS = (const(0x35), b"B") #one byte
        OPR_MODE = (const(0x3D), b"<B") #one byte
        PAGE_ID = (const(0x07), b"B") #IMU has its registers grouped into pages
        UNIT_SEL = (const(0x3B), b"B") #chooses the unit of the sensor output

    def __init__(self, bus):
        self._i2c = I2C(bus, I2C.CONTROLLER, baudrate=100000)
        self._buf = bytearray(22)
        self.initialize()  # Add the initialization method discussed earlier

    def _read_reg(self, reg):
        # Determine number of bytes to read based on reg class definition
        length = calcsize(reg[1])

        # Create a memoryview object using just the number of bytes for the specific register
        buf = memoryview(self._buf)[:length]

        # Read from the I2C bus, where device address is at 0x28, then start read at reg[0] 
        #(reg address at 0x18), read enought bytes to match len of buf
        self._i2c.mem_read(buf, self.DEV_ADDR, reg[0])

        # Unpack the bytes into a tuple
        return unpack_from(reg[1], buf)

    def _write_reg(self, reg, value):
        data = pack(reg[1], value)
        #write the data to the device at the device address
        self._i2c.mem_write(data, self.DEV_ADDR, reg[0])

    def heading(self):
        raw_heading = self._read_reg(BNO055.reg.EUL_HEADING)[0]
        return raw_heading / 16

    def initialize(self):
        #wait until IMU started up
        sleep_ms(700)

        # Select page 0 of the registers
        self._write_reg(BNO055.reg.PAGE_ID, 0)

        # Enter configuration mode (should be default after power on)
        self._write_reg(BNO055.reg.OPR_MODE, 0x00)
        sleep_ms(20)

        # Select m/s^2 for acc., degrees for heading(yaw), and degrees/s for gyro
        self._write_reg(BNO055.reg.UNIT_SEL, 0x00)

        # Start IMU fusion mode: accelerometer + gyroscope
        self._write_reg(BNO055.reg.OPR_MODE, 0x08)
        sleep_ms(10)

    def read_calibration_profile(self):
        """gets the settings learned durring calibration"""
        #store current opperating mode before chaning
        previous_mode = self._read_reg(BNO055.reg.OPR_MODE)[0]

        #go into configuration mode
        self._write_reg(BNO055.reg.OPR_MODE, 0x00)
        sleep_ms(20)

        try:
            # Read 22 calibration bytes from page-0 at the addresses 0x55â€“0x6A
            profile = bytearray(22) 
            #writes the saved settinngs to go back to IMU
            self._i2c.mem_read(profile, self.DEV_ADDR, 0x55)
            return bytes(profile)
        finally:
            #restore register to the previous mode
            self._write_reg(BNO055.reg.OPR_MODE, previous_mode)
            sleep_ms(10)

    def write_calibration_profile(self, profile):
        """write the settings back to the IMU necessary when settings are lost durring sleep"""
        if len(profile) != 22:
            raise ValueError("The Calibration profile need to be 22 bytes")

        #store current opperating mode before changing
        previous_mode = self._read_reg(BNO055.reg.OPR_MODE)[0]

        #go into configuration mode
        self._write_reg(BNO055.reg.OPR_MODE, 0x00)
        sleep_ms(20)

        try:
            self._i2c.mem_write(profile, self.DEV_ADDR, 0x55)
        finally:
            self._write_reg(BNO055.reg.OPR_MODE, previous_mode)
            sleep_ms(10)

    #returns the head, roll, and pitch in degrees
    def euler(self):
        head, roll, pitch = self._read_reg(BNO055.reg.EUL_DATA_ALL)
        return (head / 16, roll / 16, pitch / 16)

    #get gyro Z (yaw rate) in dps
    def read_gyro_z(self):
        #may need to iniitialize with the gyroscope config. zero register
        #returns a tuple containing integer value of signed 16 bit value
        raw_gyro_z = self._read_reg(BNO055.reg.GYRO_Z)[0]
        #convert raw to degrees per sec based on sensor scale
        gyro_z_dps = raw_gyro_z/16
        return gyro_z_dps

    def calib_status(self):
        status = self._read_reg(BNO055.reg.CALIB_STATUS)[0]

        # Extract bits 7-6 for the level 0-3 calibaritno where a read of 3 is fully calibrated
        #otherwise not calibrated
        system = (status >> 6) & 0b11

        #Bits 5-4 for gyroscope calibration
        gyro = (status >> 4) & 0b11

        #Bits 3-2 for accelerometer (move to bit positions 0 and 1) where read of 3
        #is fully caliubrated otherwise not calibrated
        accel = (status >> 2) & 0b11

        #Bits 1-0 for magnometer
        mag = (status) & 0b11

        return system, gyro, accel, mag


    #operating-mode configuration, heading, the gyro component corresponding to chassis yaw rate, 
    #calibration status, and calibration-profile reading/writing
