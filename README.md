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
from spi_adapter import SpiAdapter

spi =  SpiAdapter(port = "COM18)

# Single shot, 2.046v FS, Input (A0, GND).
adc_cmd = bytes([0b11000101, 0b10001010, 0x00, 0x00])

while True:
  # Read previous value and start a the next conversion.
  response_bytes = spi.send(adc_cmd, mode=1)
  adc_value = int.from_bytes(response_bytes[0:2], byteorder='big', signed=True)
  print(f"ADC: {adc_value}", flush=True)
  time.sleep(0.5)
```

<br>

## Documentation

Full documentation is available at <https://servo-adapter.readthedocs.io/>
