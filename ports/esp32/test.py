import time
import machine
import newjoy
import array
import ustruct

scl = machine.Pin(0)
sda = machine.Pin(4)
i2c = machine.I2C(freq=400_000, scl=scl, sda=sda)
ids = i2c.scan()
if not 104 in ids:
    raise Exception("No MPU found")

buf = array.array('B', [0 for _ in range(128)])

newjoy.init(10, buf)
newjoy.add_task(i2c, newjoy.TASK_MPU6050, 0)

while True: #for _ in range(100):
    time.sleep(.2)
    print(ustruct.unpack_from("fff", buf, 0))
newjoy.deinit()
