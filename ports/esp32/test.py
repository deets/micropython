# insert this into the prompt by
#
# C-e
#
# for paste  mode
import newjoy
import machine
import utime
import misc


#@misc.measure("wait_and_receive")
def wait_and_receive():
    #print("wait_and_receive")
    if newjoy.nrf24_any():
        while newjoy.nrf24_any():
            foo = newjoy.nrf24_recv()
            assert foo[:4] == b"PING"
        return True
    return False

OTTO = b'\xb4\xe6-\xbf\xda\xb5'
IRIS = b'0\xae\xa4\x8b\xd9\xe0'

START_LISTENING_TIMEOUT_US = 130
TX_SWITCH_DELAY_US = START_LISTENING_TIMEOUT_US + 500

newjoy.nrf24_teardown()
newjoy.nrf24_setup(IRIS[:5])

ERROR_TRANSLATION = {
    3: "MAX_RT",
    2: "RX_DR",
    1: "ok/TX_DS",
    -1: "timeout",
}

MESSAGE_LEN_4 = b"PONG"
MESSAGE_LEN_32 = b"This is a crafted to be 32 chars"
MESSAGE_LEN_512 = b"This is a crafted to be 512 chars long. The main trick is to just talk nonsense until the column counter in emacs shows 532 because the big T at the beginning of this line is at column 20. The way to find this out is to place the cursor on the character and look down into the mode-line of Emacs. The column is actually the second number shown in paretheses there. At least when you have my personal Emacs setup - I'm not sure if all emacs modelines show this kind of very useful information. But it's easy to do"
MESSAGE_LEN_513 = MESSAGE_LEN_512 + b"."
MESSAGE_LEN_180 = b"This is a message as long as the 6 * 2 sensors on a fully equipped board will send. That's 7 float * 6 mpus + 1 short * 6 bmps comping to a total of 180 bytes."

@misc.measure("answer")
def answer(i):
    # newjoy.nrf24_spoke_to_hub_send(MESSAGE_LEN_4)
    # newjoy.nrf24_spoke_to_hub_send(MESSAGE_LEN_32)
    newjoy.nrf24_spoke_to_hub_send(MESSAGE_LEN_180)
    # newjoy.nrf24_spoke_to_hub_send(MESSAGE_LEN_512)
    # newjoy.nrf24_spoke_to_hub_send(MESSAGE_LEN_513)
    # newjoy.nrf24_stop_listening()
    # # we expect the TX to go to RX mode, that
    # # takes a bit of time
    # utime.sleep_us(TX_SWITCH_DELAY_US)
    # send_res = newjoy.nrf24_send(b"PONG")
    # #print(ERROR_TRANSLATION[send_res])
    # newjoy.nrf24_start_listening()


newjoy.nrf24_open_rx_pipe(1, OTTO[:5])
newjoy.nrf24_start_listening()
for i in misc.cycle():
    if wait_and_receive():
        # we got a ping, answer with a PONG!
        answer(i)
    if not i % 1000:
        misc.print_measurements()
        print(newjoy.nrf24_error_info())


#newjoy.nrf24_teardown()
