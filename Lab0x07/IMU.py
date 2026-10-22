from micropython import const
from struct import calcsize, unpack_from, pack

#decide which power and ground pins to use
IMU_PINS = (cpu.Pin.B10, cpu.B11)

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

    def __init__(self, i2c):
        self._i2c = i2c
        self._buf = bytearray((0 for n in range(22)))

    def _read_reg(self, reg):
        # Determine number of bytes to read
        length = calcsize(reg[1])

        # Create a memoryview object of the right size
        buf = memoryview(self._buf)[:length]

        # Read from the I2C bus into the memoryview
        self._i2c.mem_read(buf, self.DEV_ADDR, reg[0])

        # Unpack the bytes into a tuple
        return unpack_from(reg[1], buf)

    def _write_reg(self, reg, value):
        data = pack(reg[1], value)
        #write the data to the device at the device address
        self._i2c.mem_write(data, self.DEV_ADDR, reg[0])

    #returns the head, roll, and pitch in degrees
    def euler(self):
        head, roll, pitch = self._read_reg(BNO055.reg.EUL_DATA_ALL)
        return (head / 16, roll / 16, pitch / 16)

    #get gyro Z (yaw rate) in dps
    def read_gyro_z(self):
        raw_gyro_z = self._read_reg(BNO055.reg.GYRO_Z)

        #convert raw to degrees per sec based on sensor scale: 
        #
        gyro_z_dps = raw_gyro_z
        return gyro_z_dps

    def calib_status(self):
        status = self._read_reg(BNO055.reg.CALIB_STATUS)

        # Extract bits 7-6 for the level 0-3 calibaritnon
        system = (status >> 6) & 0b11

        #Bits 5-4 for gyroscope calibration

        #Bits 3-2 for accelerometer

        #Bits 1-0 for magnometer


    #operating-mode configuration, heading, the gyro component corresponding to chassis yaw rate, 
    #calibration status, and calibration-profile reading/writing
