from pyb import I2C

# PB8 = SCL, PB9 = SDA on your board
i2c = I2C(1, I2C.CONTROLLER, baudrate=100_000)

devices = i2c.scan()
print("I2C addresses:", [hex(addr) for addr in devices])

# Read the BNO055 chip ID
address = 0x28
if address in devices:
    chip_id = i2c.mem_read(1, address, 0x00)[0]
    print("Chip ID:", hex(chip_id))
    print("BNO055 detected:", chip_id == 0xA0)