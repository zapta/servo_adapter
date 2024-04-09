# A Simple USB to Servo Adapter that works.

The Servo Adapter allows python programs to connect to RC Servos using off the shelf low cost boards such the Raspberry Pico or SparkFun Pro Micro - RP2040. The Servo Adapter appears on the computer as a serial port (no device installation required) and acts as a USB to Servo bridge, with the ``servo_adapter`` Python package providing an easy to use API.


For example, the diagram below shows the wiring of a servo to one of the 8 servo outputs of the Servo adapter.

<br>
<img  src="https://raw.githubusercontent.com/zapta/servo_adapter/main/www/wiring_diagram.png"
      style="display: block;margin-left: auto;margin-right: auto;width: 80%;" />
<br>



## Highlights

* Support up to 8 independent servo channels.
* Hardware based 1us resolution servo PWM timing generation.
* Supports Windows/Mac/Linux.
* Uses low cost low cost off-the-shelf boards as adapters.
* Does not require driver installation (it appears on the computer as standard a serial port).
* Comes with an easy to use Python API.
* Easy to modify/extend and to adapt to new hardware.
* Permissive open source license. Comercial use OK, sharing and attribution not required. 
* Provides additional 8 general purpose auxilary input/output signals.

<br>

## Python API Example

Package installation

```bash
pip install servo-adapter --upgrade
```

In the example below, we use an Servo Adapter that control a servo that is connected to servo output 0.

```python
import time
from random import randrange

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
```

<br>

## Documentation

Full documentation is available at <https://servo-adapter.readthedocs.io/>
