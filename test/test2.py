import sys
import time
from random import randrange

sys.path.insert(0, "../src/")
from servo_adapter import ServoAdapter

# Serial port name. Adapt to your system.
port = "/dev/tty.usbmodem1101"

# Connect to adapter and enable PWM servo out 0.
adapter = ServoAdapter(port=port)
adapter.set_servo_state(0, True)

while True:
    # Random pulse width in the range 1000us to 2000us.
    pw_us = 1000 + randrange(1000 + 1)
    adapter.set_servo_pulse_width(0, pw_us)
    time.sleep(1.0)
