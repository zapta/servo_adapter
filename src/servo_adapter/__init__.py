"""The ``servo_adapter`` package provides the API to access Servo Adapter boards. To access an Servo Adapter,
create an object of the  class ServoAdapter, and use the methods it provides.
"""

from typing import Optional, List, Tuple
from serial import Serial
from enum import Enum
import time


# NOTE: Numeric values match wire protocol.
class AuxPinMode(Enum):
    """Auxilary pin modes."""

    INPUT_PULLDOWN = 1
    INPUT_PULLUP = 2
    OUTPUT = 3


class ServoAdapter:
    """Connects to the Servo Adapter at the specified serial port and asserts that the
    adapter responses as expcted.

    :param port: The serial port of the Servo Adapter. Servo Adapters
        appear on the local computer as a standard serial port
    :type port: str
    """

    def __init__(self, port: str):
        self.__serial: Serial = Serial(port, timeout=1.0)
        if not self.test_connection_to_adapter():
            raise RuntimeError(f"servo driver not detected at port {port}")
        adapter_info = self.__read_adapter_info()
        if adapter_info is None:
            raise RuntimeError(f"Servo driver failed to read adapter info at {port}")
        print(f"Adapter info: {adapter_info.hex(" ")}", flush=True)
        if (
            adapter_info[0] != ord("S")
            or adapter_info[1] != ord("R")
            or adapter_info[2] != ord("V")
            or adapter_info[3] != 0x3
        ):
            raise RuntimeError(f"Unexpected Servo Adapter info at {port}")

    def __read_adapter_response(self, op_name: str, ok_resp_size: int) -> bytes:
        """A common method to read a response from the adapter.
        Returns None if error, otherwise OK response bytes"""
        assert isinstance(op_name, str)
        assert isinstance(ok_resp_size, int)
        assert 0 <= ok_resp_size
        # Read status flag.
        ok_resp = self.__serial.read(1)
        assert isinstance(ok_resp, bytes), type(ok_resp)
        if len(ok_resp) != 1:
            print(
                f"{op_name}: status flag read mismatch, expected {1}, got {len(ok_resp)}",
                flush=True,
            )
            return None
        status_flag = ok_resp[0]
        if status_flag not in (ord("E"), ord("K")):
            print(
                f"{op_name}: unexpected status flag in response: {ok_resp}", flush=True
            )
            return None

        # Handle the case of an error
        if status_flag == ord("E"):
            # Read the additional error info byte.
            ok_resp = self.__serial.read(1)
            assert isinstance(ok_resp, bytes), type(ok_resp)
            if len(ok_resp) != 1:
                print(
                    f"{op_name}: error info read mismatch, expected {1}, got {len(ok_resp)}",
                    flush=True,
                )
                return None
            print(f"{op_name}: failed with error code {ok_resp[0]}", flush=True)
            return None

        # Handle the OK case.
        #
        # Read the returned data count.
        ok_resp = self.__serial.read(ok_resp_size)
        assert isinstance(ok_resp, bytes), type(ok_resp)
        if len(ok_resp) != ok_resp_size:
            print(
                f"{op_name}: OK resp read count mismatch, expected {ok_resp_size}, got {len(ok_resp)}",
                flush=True,
            )
            return None
        return ok_resp

    def set_servo_pulse_width(self, pwm_pin: int, pulse_width_us: int) -> bool:
        """Sets a PWM pin pulse width. This doesn not change the pin state on or off.

        :param pwm_pin: The PWM pin index in the range [0, 7].
        :type values: int

        :param pulse_width_us: The Servo pulse width in us. Must be in the range [500, 2500]. Note
            that if the PWM pin is off, the pulses are not generated until it's set to on state.
        :type mask: int

        :returns: True if OK, False otherwise.
        :rtype: bool
        """
        assert isinstance(pwm_pin, int)
        assert 0 <= pwm_pin <= 7
        assert isinstance(pulse_width_us, int)
        assert 500 <= pulse_width_us <= 2500
        req = bytearray()
        req.append(ord("w"))
        req.append(pwm_pin)
        req.append(pulse_width_us >> 8)
        req.append(pulse_width_us & 0xFF)
        self.__serial.write(req)
        ok_resp = self.__read_adapter_response("Servo pulse", 0)
        if ok_resp is None:
            return False
        return True

    def set_servo_state(self, pwm_pin: int, state: bool) -> bool:
        """Sets a PWM pin on or off.

        :param pwm_pin: The PWM pin index in the range [0, 7].
        :type values: int

        :param state: Indicates if to turn the PWM state on (True) of off (False)
        :type mask: bool

        :returns: True if OK, False otherwise.
        :rtype: bool
        """
        assert isinstance(pwm_pin, int)
        assert 0 <= pwm_pin <= 7
        req = bytearray()
        req.append(ord("s"))
        req.append(1 << pwm_pin)
        req.append(1 << pwm_pin if state else 0x00)
        self.__serial.write(req)
        ok_resp = self.__read_adapter_response("Servo state", 0)
        if ok_resp is None:
            return False
        return True

    def set_aux_pin_mode(self, pin: int, pin_mode: AuxPinMode) -> bool:
        """Sets the mode of an auxilary pin.

        :param pin: The aux pin index, should be in [0, 7].
        :type pin: int

        :param pin_mode: The new pin mode.
        :type pin_mode: AuxPinMode

        :returns: True if OK, False otherwise.
        :rtype: bool
        """
        assert isinstance(pin, int)
        assert 0 <= pin <= 7
        assert isinstance(pin_mode, AuxPinMode)
        req = bytearray()
        req.append(ord("m"))
        req.append(pin)
        req.append(pin_mode.value)
        self.__serial.write(req)
        ok_resp = self.__read_adapter_response("Aux mode", 0)
        if ok_resp is None:
            return False
        return True

    def read_aux_pins(self) -> int | None:
        """Reads the auxilary pins.

        :returns: The pins value as a 8 bit in value or None if an error.
        :rtype: int | None
        """
        req = bytearray()
        req.append(ord("a"))
        self.__serial.write(req)
        ok_resp = self.__read_adapter_response("Aux read", 1)
        if ok_resp is None:
            return None
        return ok_resp[0]

    def write_aux_pins(self, values, mask=0b11111111) -> bool:
        """Writes the aux pins.

        :param values: An 8 bits integer with the bit values to write. In the range [0, 255].
        :type values: int

        :param mask: An 8 bits int with mask that indicates which auxilary pins should be written. If
            the corresponding bits is 1 than the pin is updated otherwise it's left as is.
        :type mask: int

        :returns: True if OK, False otherwise.
        :rtype: bool
        """
        assert isinstance(values, int)
        assert 0 <= values <= 255
        assert isinstance(mask, int)
        assert 0 <= mask <= 255
        req = bytearray()
        req.append(ord("b"))
        req.append(values)
        req.append(mask)
        self.__serial.write(req)
        ok_resp = self.__read_adapter_response("Aux write", 0)
        if ok_resp is None:
            return False
        return True

    def read_aux_pin(self, aux_pin_index: int) -> bool | None:
        """Read a single aux pin.

        :param aux_pin_index: An aux pin index in the range [0, 7]
        :type aux_pin_index: int

        :returns: The boolean value of the pin or None if error.
        :rtype: bool | None
        """
        assert isinstance(aux_pin_index, int)
        assert 0 <= aux_pin_index <= 7
        pins_values = self.read_aux_pins()
        if pins_values is None:
            return None
        return True if pins_values & (1 << aux_pin_index) else False

    def write_aux_pin(self, aux_pin_index: int, value: bool | int) -> bool:
        """Writes a single aux pin.

        :param aux_pin_index: An aux pin index in the range [0, 7]
        :type aux_pin_index: int

        :param value: The value to write.
        :type value: bool | int

        :returns: True if OK, False otherwise.
        :rtype: bool
        """
        assert isinstance(aux_pin_index, int)
        assert 0 <= aux_pin_index <= 7
        assert isinstance(value, (bool, int))
        pin_mask = 1 << aux_pin_index
        pin_value_mask = pin_mask if value else 0
        return self.write_aux_pins(pin_value_mask, pin_mask)

    def test_connection_to_adapter(self, max_tries: int = 3) -> bool:
        """Tests connection to the Servo Adapter.

        The method tests if the Servo adapter exists and is responding. It is provided
        for diagnostic purposes and is not needed in typical applications.

        :param max_tries: Max number of attempts. The default should be good for most case.
        :type max_tries: int

        :returns: True if connection is OK, false otherwise.
        :rtype: bool
        """
        assert max_tries > 0
        for i in range(max_tries):
            if i > 0:
                # Delay to let any pending command to timeout.
                time.sleep(0.3)
            ok: bool = True
            for b in [0x00, 0xFF, 0x5A, 0xA5]:
                if not self.__test_echo_cmd(b):
                    ok = False
                    break
            if ok:
                # We had one good pass on all patterns. We are good.
                return True
        # All tries failed.
        return False

    def __test_echo_cmd(self, b: int) -> bool:
        """Test if an echo command with given byte returns the same byte. Used
        to test the connection to the driver."""
        assert isinstance(b, int)
        assert 0 <= b <= 256
        req = bytearray()
        req.append(ord("e"))
        req.append(b)
        self.__serial.write(req)
        resp = self.__serial.read(1)
        assert isinstance(resp, bytes), type(resp)
        assert len(resp) == 1
        return resp[0] == b

    def __read_adapter_info(self) -> Optional[bytearray]:
        """Return adapter info or None if an error."""
        req = bytearray()
        req.append(ord("i"))
        n = self.__serial.write(req)
        if n != len(req):
            print(
                f"Servo adapter info: write mismatch, expected {len(req)}, got {n}",
                flush=True,
            )
            return None
        ok_resp = self.__read_adapter_response("Servo adapter info", ok_resp_size=7)
        return ok_resp
