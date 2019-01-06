# insert this into the prompt by
#
# C-e
#
# for paste  mode
import newjoy
import machine
import utime

OTTO = b'\xb4\xe6-\xbf\xda\xb5'
IRIS = b'0\xae\xa4\x8b\xd9\xe0'

START_LISTENING_TIMEOUT_US = 130
TX_SWITCH_DELAY_US = START_LISTENING_TIMEOUT_US + 500

newjoy.nrf24_setup(IRIS[:5], OTTO[:5])

newjoy.nrf24_start_listening()

while True:
    newjoy.nrf24_start_listening()
    while not newjoy.nrf24_any():
        pass
    # consume the messages away
    while newjoy.nrf24_any():
        print(newjoy.nrf24_recv())
    utime.sleep_us(TX_SWITCH_DELAY_US)
    newjoy.nrf24_stop_listening()
    print(newjoy.nrf24_send(b"PONG"))



#newjoy.nrf24_teardown()
