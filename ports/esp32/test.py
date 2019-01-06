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

newjoy.nrf24_setup(IRIS[:5], OTTO[:5])

newjoy.nrf24_start_listening()

while True:
    newjoy.nrf24_start_listening()
    while not newjoy.nrf24_any():
        #print("nothing received")
        pass
    print("we got a message!")
    newjoy.nrf24_stop_listening()

#newjoy.nrf24_teardown()
