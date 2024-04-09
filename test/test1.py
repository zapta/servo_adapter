import sys
import time

sys.path.insert(0, '../src/')
from servo_adapter import ServoAdapter, AuxPinMode

# port = "COM18"
port = "/dev/tty.usbmodem1101"

print(f"Connecting to port {port}...", flush=True)
adapter =  ServoAdapter(port = port)
print(f"Connected.", flush=True)

adapter.set_aux_pin_mode(0, AuxPinMode.OUTPUT)


count_up = True
start_time = time.time()
us_per_sec = 100

adapter.set_servo_pulse_width(0, 500)
adapter.set_servo_state(0, True)  # Turn pwm on

while True:
  # time.sleep(0.002)

  #adapter.write_aux_pins(i % 256, 0b00000001)
  #   cs = 0  #i % 4
  #   mode = 0 # (i % 2) * 2
  #   #speed = 4000000
  #   #result = adapter.send(bytearray([0x11, 0x22, 0x33]), extra_bytes=2, cs=cs, mode=mode, speed=speed, read=True)

  time_now = time.time()
  if count_up:
    pw_us = 500 + (time_now - start_time) * us_per_sec
    if pw_us >= 2500:
      pw_us = 2500
      start_time = time_now
      count_up = False
  else:
    pw_us = 2500 - (time_now - start_time) * us_per_sec
    if pw_us <= 500:
      pw_us = 500
      start_time = time_now
      count_up = True

  pw_us = int(pw_us)
  adapter.set_servo_pulse_width(0, pw_us)
  # adapter.read_aux_pin(0)
  # adapter.set_servo_state(0, (pw_us & 0x1) == 0)
  print(f"{pw_us:4d}")
  


