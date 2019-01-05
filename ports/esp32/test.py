# insert this into the prompt by
#
# C-e
#
# for paste  mode
import newjoy
import machine
import time

OTTO = b'\xb4\xe6-\xbf\xda\xb5'
IRIS = b'0\xae\xa4\x8b\xd9\xe0'

newjoy.nrf24_setup()
time.sleep(5)
while True:
    newjoy.nrf24_run_spoke(IRIS[:5], OTTO[:5])
newjoy.nrf24_teardown()
