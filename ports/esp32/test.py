import machine
scl = machine.Pin(0)
sda = machine.Pin(4)
i2c = machine.I2C(freq=400_000, scl=scl, sda=sda)
i2c.scan()
