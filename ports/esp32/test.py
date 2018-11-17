# insert this into the prompt by
#
# C-e
#
# for paste  mode
import newjoy
import machine
import array
import ustruct

scl = machine.Pin(17)
sda = machine.Pin(16)
i2c = machine.I2C(freq=400_000, scl=scl, sda=sda)
buf = array.array('B', [0 for _ in range(128)])
newjoy.init(2, buf)
addresses = i2c.scan()
print(addresses)
newjoy.add_task(i2c, addresses[0], newjoy.TASK_BMP280, 0)
while True:
    newjoy.sync()
    print(ustruct.unpack("<I", buf)[0])


# import time
# import array
# import ustruct


# print(i2c.scan())

# buf = array.array('B', [0 for _ in range(128)])

# BMP280_ADDRESS_LO = 0x76
# BMP280_ID = 0xD0 # needs to return 0x58/b'X'
# BMP280_RESET = 0xE0 # only accepts 0xB6 as
# BMP280_CONTROL_MEASUREMENT = 0xF4
# BMP280_CONFIG = 0xF5
# BMP280_STATUS = 0xF3

# # Filter coefficients
# BMP280_FILTER_OFF = 0x00
# BMP280_FILTER_COEFF_2 = 0x01
# BMP280_FILTER_COEFF_4 = 0x02
# BMP280_FILTER_COEFF_8 = 0x03
# BMP280_FILTER_COEFF_16 = 0x04
# BMP280_PRESS_MSB = 0xF7
# BMP280_PRESS_LSB = 0xF8


# if i2c.readfrom_mem(BMP280_ADDRESS_LO, BMP280_ID, 1) == b'X':
#     print("found BMP280")
#     i2c.writeto_mem(BMP280_ADDRESS_LO, BMP280_RESET, b'\xb6')
#     time.sleep(.5)
#     # config register
#     # set to android device as described in chapter 3.3
#     t_sb = 0b000 # 0.5 ms standby
#     filter = 0
#     spi = 0x00 # no SPI interface
#     config_value = t_sb << 5 | filter << 2 | spi
#     i2c.writeto_mem(BMP280_ADDRESS_LO, BMP280_CONFIG, chr(config_value))
#     # ctrl_meas
#     # temperature
#     osrs_t = 0b000 # no temperature reading
#     osrs_p = 0b011 # 4 times oversampling
#     mode = 0b11 # normal mode
#     control_value= osrs_t << 5 | osrs_p << 2 | mode
#     print(bin(control_value))
#     i2c.writeto_mem(BMP280_ADDRESS_LO, BMP280_CONTROL_MEASUREMENT, chr(control_value))
#     while True:
#         time.sleep(.5)
#         status = i2c.readfrom_mem(BMP280_ADDRESS_LO, BMP280_STATUS, 1)
#         pressure = i2c.readfrom_mem(BMP280_ADDRESS_LO, BMP280_PRESS_MSB, 3)
#         pressure = ustruct.unpack(">H", pressure)[0]
#         print(status, pressure)
#         #
#         #
