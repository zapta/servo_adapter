// Firmware of the SPI Adapter implementation using a Raspberry Pico.

#include <Arduino.h>
#include <mbed.h>

#include "board.h"

// #pragma GCC push_options
// #pragma GCC optimize("Og")

using board::led;
using mbed::PwmOut;

// PwmOut* pwm = nullptr;

// constexpr uint kTest1Pin = 16;

// Based on
// https://forums.mbed.com/t/pwm-period-is-not-good-on-mbed-with-raspberry-pico/22151/5
class Servo {
 public:
  Servo(uint8_t gpio, uint16_t initial_pw_us)
      : _gpio(gpio),
        _slice(pwm_gpio_to_slice_num(gpio)),
        _chan(pwm_gpio_to_channel(gpio)) {
    // gpio_set_function(_gpio, GPIO_FUNC_PWM);

    // We start with pwm pin disabled.
    pwm_off(false);

    // Initialize the slice. Each slice is used by two gpio pins.
    // Raspberry Pico default system clock is 125Mhz.
    // We divide by 125 for 1us resolution.
    pwm_set_clkdiv(_slice, 125);
    // This determines the pwm period. We divide by 20,000 for 50 Hz
    // PWM cycle. This allowed value here is in the range [0, 65535].
    pwm_set_wrap(_slice, 20000 - 1);
    pwm_set_enabled(_slice, true);

    // Set the gpio pulse width. Note that the pin is still
    // not connected to the PWM generator, until the state is changed to on.
    set_pulse_width_us(initial_pw_us);

    // Connect the gpio to the PWM slice.
    // set_state(true);
  }

  // The standard servo pulse width is in the range 500us to 1500us.
  void set_pulse_width_us(uint16_t pw_us) {
    pwm_set_chan_level(_slice, _chan, pw_us - 1);
  }

  // Turn signal on and off.
  void set_state(bool enabled) {
    if (enabled == _is_on) {
      return;  // No change
    }
    if (enabled) {
      pwm_on(true);
    } else {
      pwm_off(true);
    }
  }

 private:
  const uint _gpio;
  const uint _slice;
  const uint _chan;
  bool _is_on = false;

  void pwm_on(bool sync) {
    if (sync) {
      sync_to_counter();
    }
    gpio_set_function(_gpio, GPIO_FUNC_PWM);
    _is_on = true;
  }

  void pwm_off(bool sync) {
    if (sync) {
      sync_to_counter();
    }
    // Disable. Set pin to output low.
    gpio_set_function(_gpio, GPIO_FUNC_SIO);
    gpio_set_dir(_gpio, true);
    gpio_put(_gpio, false);
    _is_on = false;
  }

  // Wait for a 'dead' portion of the pwm cycle so we can change the pwm pin
  // state without corrupting a pulse.
  void sync_to_counter() {
    // Dead zone specification [3000us, 15000us]. It's past the servo pulses and
    // provides 5ms to perform the switch until the next pwem pulses (assuming
    // period = 20ms)
    constexpr uint kDeadTimeMinUs = 3000;
    constexpr uint kDeadTimeMaxUs = 15000;
    for (;;) {
      const uint16_t counter = pwm_get_counter(_slice);
      if (counter >= kDeadTimeMinUs && counter <= kDeadTimeMaxUs) {
        return;
      }
    }
  }
};

static Servo servos[] = {
    Servo(8, 500),  Servo(9, 500),  Servo(10, 500), Servo(11, 500),
    Servo(12, 500), Servo(13, 500), Servo(14, 500), Servo(15, 500),
};
constexpr auto kNumServos = sizeof(servos) / sizeof(*servos);
static_assert(kNumServos == 8);

// Maps PWM pin index to gp pin index.
// static uint8_t pwm_pins[] = {
//     8,   // PWM 0 = GP8
//     9,   // PWM 1 = GP9
//     10,  // PWM 2 = GP10
//     11,  // PWM 3 = GP11
//     12,  // PWM 4 = GP12
//     13,  // PWM 5 = GP13
//     14,  // PWM 6 = GP14
//     15,  // PWM 7 = GP15
// };

// static constexpr uint8_t kNumPwmPins = sizeof(pwm_pins) / sizeof(*pwm_pins);
// static_assert(kNumPwmPins == 8);

// Maps aux pin index to gp pin index.
static uint8_t aux_pins[] = {
    0,  // Aux 0 = GP0
    1,  // Aux 0 = GP1
    2,  // Aux 0 = GP2
    3,  // Aux 0 = GP3
    4,  // Aux 0 = GP4
    5,  // Aux 0 = GP5
    6,  // Aux 0 = GP6
    7,  // Aux 0 = GP7
};

static constexpr uint8_t kNumAuxPins = sizeof(aux_pins) / sizeof(*aux_pins);
static_assert(kNumAuxPins == 8);

static constexpr uint8_t kApiVersion = 1;
static constexpr uint16_t kFirmwareVersion = 1;

// Max number of bytes per transaction.
static constexpr uint16_t kMaxTransactionBytes = 256;

// All command bytes must arrive within this time period.
static constexpr uint32_t kCommandTimeoutMillis = 250;

// Since LED updates may involved neopixel communication, we minimize
// it by filtering the 'no-change' updates.
static bool last_led_state;

// A temporary buffer for commands and SPI operations.
static uint8_t data_buffer[kMaxTransactionBytes];
// The number of valid bytes in data_buffer.
static uint16_t data_size = 0;

// Tracks the last spi mode we used. Used to implement a woraround for
// clock polarity change which requires changing the idle SPI clock
// level. See https://github.com/arduino/ArduinoCore-mbed/issues/828
// static SPIMode last_spi_mode = SPI_MODE1;

// static void track_spi_clock_polarity(SPIMode new_spi_mode) {
//   // No change.
//   if (new_spi_mode == last_spi_mode) {
//     return;
//   }

//   // Perform a dummy transaction to settle the clock level.
//   SPISettings spi_setting(4000000, MSBFIRST, new_spi_mode);
//   SPI.beginTransaction(spi_setting);
//   uint8_t dummy_byte = 0;
//   SPI.transfer(&dummy_byte, 0);
//   SPI.endTransaction();

//   // Update
//   last_spi_mode = new_spi_mode;
// }

// A simple timer.
// Cveate: overflows 50 days after last reset().
class Timer {
 public:
  Timer() { reset(millis()); }
  void reset(uint32_t millis_now) { _start_millis = millis_now; }
  uint32_t elapsed_millis(uint32_t millis_now) {
    return millis_now - _start_millis;
  }
  void add_start_millis(uint32_t time_millis) {
    // Wrap around ok.
    _start_millis += time_millis;
  }

 private:
  uint32_t _start_millis;
};

// Turn off all CS outputs.
// static inline void all_cs_off() {
//   static_assert(kNumCsPins == 4);
//   digitalWrite(cs_pins[0], HIGH);
//   digitalWrite(cs_pins[1], HIGH);
//   digitalWrite(cs_pins[2], HIGH);
//   digitalWrite(cs_pins[3], HIGH);
// }

// Turn on a specific CS output.
// static inline void cs_on(uint8_t cs_index) {
//   if (cs_index < kNumCsPins) {
//     digitalWrite(cs_pins[cs_index], LOW);
//   }
// }

// Time since the start of last cmd.
static Timer cmd_timer;

// Fill data_buffer with n bytes. Done in chunks. data_size tracks the
// num of bytes read so far.
static bool read_serial_bytes(uint16_t n) {
  // Handle the case where not enough chars.
  const uint16_t avail = Serial.available();
  const uint16_t required = n - data_size;
  const uint16_t requested = std::min(avail, required);

  if (requested) {
    size_t actual_read =
        Serial.readBytes((char*)(&data_buffer[data_size]), requested);
    data_size += actual_read;
  }

  return data_size >= n;
}

// Abstract base of all command handlers.
class CommandHandler {
 public:
  CommandHandler(const char* name) : _name(name) {}
  const char* cmd_name() const { return _name; }
  // Called each time the command starts to allow initialization.
  virtual void on_cmd_entered() {}
  // Returns true if command completed.
  virtual bool on_cmd_loop() = 0;
  // Call if the command is aborted due to timeout.
  virtual void on_cmd_aborted() {}

 private:
  const char* _name;
};

// ECHO command. Recieves a byte and echoes it back as a response. Used
// to test connectivity with the driver.
//
// Command:
// - byte 0:  'e'
// - byte 1:  Bhar to echo, 0x00 to 0xff
//
// Response:
// - byte 0:  Byte 1 from the command.
//
static class EchoCommandHandler : public CommandHandler {
 public:
  EchoCommandHandler() : CommandHandler("ECHO") {}
  virtual bool on_cmd_loop() override {
    static_assert(sizeof(data_buffer) >= 1);
    if (!read_serial_bytes(1)) {
      return false;
    }
    Serial.write(data_buffer[0]);
    return true;
  }
} echo_cmd_handler;

// INFO command. Provides information about this driver. Currently
// it's a skeleton for future values that will be returned.
//
// Command:
// - byte 0:  'i'
//
// Response:
// - byte 0:  'K' for OK.
// - byte 1:  'P'
// - byte 2:  'W'
// - byte 3:  'M'
// - byte 4:  Number of bytes to follow (3).
// - byte 5:  Version of wire format API.
// - byte 6:  MSB of firmware version.
// - byte 7:  LSB of firmware version.
static class InfoCommandHandler : public CommandHandler {
 public:
  InfoCommandHandler() : CommandHandler("INFO") {}
  virtual bool on_cmd_loop() override {
    Serial.write('K');  // 'K' for OK.
    Serial.write('P');
    Serial.write('W');
    Serial.write('M');
    Serial.write(0x03);                     // Number of bytes to follow.
    Serial.write(kApiVersion);              // API version.
    Serial.write(kFirmwareVersion >> 8);    // Firmware version MSB.
    Serial.write(kFirmwareVersion & 0x08);  // Firmware version LSB.
    return true;
  }
} info_cmd_handler;

// Pulse Width command. Sets the pulse width of a single servo channel.
//
// Command:
// - byte 0:    'w'
// - byte 1:    Pwm channel index [0, 7]
// - byte 2,3:  Pulse width in usecs. Big endian. Valid range is [500, 2500]
//
// Error response:
// - byte 0:    'E' for error.
// - byte 1:    Error code, per the list below, providing additional
//              information about the error.
//
// OK response
// - byte 0:    'K' for 'OK'.

// Error code:
//  1 : Servo channel index out of range.
//  2 : Pulse width too low.
//  3 : Pulse width too high.
//
static class PulseWidthCommandHandler : public CommandHandler {
 public:
  PulseWidthCommandHandler() : CommandHandler("PWM") {}

  virtual bool on_cmd_loop() override {
    // Read the command.
    static_assert(sizeof(data_buffer) >= 3);
    if (!read_serial_bytes(3)) {
      return false;
    }
    // Parse the command.
    const uint8_t servo_index = data_buffer[0];
    if (servo_index >= kNumServos) {
      Serial.write('E');
      Serial.write(0x01);  // error code
      return true;
    }
    const uint16_t pw_us = (((uint16_t)data_buffer[1]) << 8) + data_buffer[2];
    if (pw_us < 500 || pw_us > 2500) {
      Serial.write('E');
      Serial.write(pw_us < 500 ? 2 : 3);  // error code
      return true;
    }
    // Set the servo.
    servos[servo_index].set_pulse_width_us(pw_us);
    // All done ok.
    Serial.write('K');
    return true;
  }

} pulse_width_cmd_handler;

// PWM State command. Sets PWM channels on/off.
//
// Command:
// - byte 0:    's'
// - byte 1:    Channel mask. Only the PWM channels whose respective bit is set
// to 1 are affected.
// - byte 2::   Channel state. PWM channels shows mask bit is '1' are enabled if
// their bit here is '1'
//              or disabled if their bit here is '0'. Otherwise, their bit here
//              is ignored.
// Error response:
// - byte 0:    'E' for error.
//
// OK response
// - byte 0:    'K' for 'OK'.
//
static class PwmStateCommandHandler : public CommandHandler {
 public:
  PwmStateCommandHandler() : CommandHandler("STATE") {}

  virtual bool on_cmd_loop() override {
    // Read the command.
    static_assert(sizeof(data_buffer) >= 2);
    if (!read_serial_bytes(2)) {
      return false;
    }
    // Parse the command.
    const uint8_t chan_mask = data_buffer[0];
    const uint8_t chan_flags = data_buffer[1];

    // Apply the settings
    static_assert(kNumServos == 8);
    for (uint8_t i = 0; i < 8; i++) {
      if ((chan_mask >> i) & 0x01) {
        servos[i].set_state((chan_flags >> i) & 0x01);
      }
    }
    // All done ok.
    Serial.write('K');
    return true;
  }

} pwm_state_cmd_handler;

// SET AUXILARY PIN MODE command.
//
// Command:
// - byte 0:    'm'
// - byte 1:    pin index, 0 - 7
// - byte 2:    pin mode
//
// Error response:
// - byte 0:    'E' for error.
// - byte 1:    Error code, per the list below.
//
// OK response
// - byte 0:    'K' for 'OK'.

// Error codes:
//  1 : Pin index out of range.
//  2 : Mode value out of range.
static class AuxPinModeCommandHandler : public CommandHandler {
 public:
  AuxPinModeCommandHandler() : CommandHandler("AUX_MODE") {}

  virtual bool on_cmd_loop() override {
    // Read command header.
    // if (!_got_cmd_header) {
    static_assert(sizeof(data_buffer) >= 2);
    if (!read_serial_bytes(2)) {
      return false;
    }
    // Parse the command header
    const uint8_t aux_pin_index = data_buffer[0];
    const uint8_t aux_pin_mode = data_buffer[1];

    // Check aux pin index range.
    if (aux_pin_index >= kNumAuxPins) {
      Serial.write('E');
      Serial.write(0x01);
      return true;
    }

    // Map to underlying gpio pin.
    const uint8_t gpio_pin = aux_pins[aux_pin_index];

    // Dispatch by pin mode:
    switch (aux_pin_mode) {
      // Input pulldown
      case 1:
        pinMode(gpio_pin, INPUT_PULLDOWN);
        break;

      // Input pullup
      case 2:
        pinMode(gpio_pin, INPUT_PULLUP);
        break;

      // Output.
      case 3:
        pinMode(gpio_pin, OUTPUT);
        break;

      default:
        Serial.write('E');
        Serial.write(0x02);
        return true;
    }

    // All done Ok
    Serial.write('K');
    return true;
  }

} aux_mode_cmd_handler;

// READ AUXILARY PINS command.
//
// Command:
// - byte 0:    'a'
//
// Error response:
// - byte 0:    'E' for error.
// - byte 1:    Reserved. Always 0.
//
// OK response
// - byte 0:    'K' for 'OK'.
// - byte 1:    Auxilary pins values
static class AuxPinsReadCommandHandler : public CommandHandler {
 public:
  AuxPinsReadCommandHandler() : CommandHandler("AUX_READ") {}

  virtual bool on_cmd_loop() override {
    uint8_t result = 0;
    static_assert(kNumAuxPins == 8);
    for (int i = 7; i >= 0; i--) {
      const uint8_t gpio_pin = aux_pins[i];
      const PinStatus pin_status = digitalRead(gpio_pin);
      result = result << 1;
      if (pin_status) {
        result |= 0b00000001;
      }
    }

    // All done Ok
    Serial.write('K');
    Serial.write(result);
    return true;
  }

} aux_pins_read_cmd_handler;

// WRITE AUXILARY PINS command.
//
// Command:
// - byte 0:    'b'
// - byte 1:    New pins values
// - byte 2:    Write mask. Only pins with a corresponding '1' are written.
//
// Error response:
// - byte 0:    'E' for error.
// - byte 1:    Reserved. Always 0.
//
// OK response
// - byte 0:    'K' for 'OK'.
static class AuxPinsWriteCommandHandler : public CommandHandler {
 public:
  AuxPinsWriteCommandHandler() : CommandHandler("AUX_WRITE") {}

  virtual bool on_cmd_loop() override {
    static_assert(sizeof(data_buffer) >= 2);
    if (!read_serial_bytes(2)) {
      return false;
    }
    const uint8_t values = data_buffer[0];
    const uint8_t mask = data_buffer[1];
    static_assert(kNumAuxPins == 8);
    for (int i = 0; i < 8; i++) {
      if (mask & 1 << i) {
        const uint8_t gpio_pin = aux_pins[i];
        // TODO: We write also to input pins. What is the semantic?
        digitalWrite(gpio_pin, values & 1 << i);
      }
    }

    // All done Ok
    Serial.write('K');
    return true;
  }

} aux_pins_write_cmd_handler;

// Given a command char, return a Command pointer or null if invalid command
// char.
static CommandHandler* find_command_handler_by_char(const char cmd_char) {
  switch (cmd_char) {
    case 'e':
      return &echo_cmd_handler;
    case 'i':
      return &info_cmd_handler;
    case 'm':
      return &aux_mode_cmd_handler;
    case 'a':
      return &aux_pins_read_cmd_handler;
    case 'b':
      return &aux_pins_write_cmd_handler;
    case 'w':
      return &pulse_width_cmd_handler;
    case 's':
      return &pwm_state_cmd_handler;
    default:
      return nullptr;
  }
}

void setup() {
  // A short delay to let the USB/CDC settle down. Otherwise
  // it messes up with the debugger, in case it's used.
  delay(500);

  board::setup();
  board::led.update(false);
  last_led_state = false;

  // USB serial.
  Serial.begin(115200);

  // Init aux pins as inputs.
  for (uint8_t i = 0; i < kNumAuxPins; i++) {
    auto gp_pin = aux_pins[i];
    pinMode(gp_pin, INPUT_PULLUP);
  }

  // gpio_set_function(kTest1Pin, GPIO_FUNC_SIO);
  // gpio_set_dir(kTest1Pin, true);
  // gpio_put(kTest1Pin, false);

  servos[0].set_pulse_width_us(2000);
  servos[0].set_state(true);
}

// If in command, points to the command handler.
static CommandHandler* current_cmd = nullptr;

// static bool servo_up = true;
// static uint servo_pw = 500;
// static Timer servo_timer;

// static Timer on_off_timer;
// static bool on_off = true;

void loop() {
  Serial.flush();
  const uint32_t millis_now = millis();

  // if (servo_timer.elapsed_millis(millis_now) >= 3) {
  //   const uint step = 1;
  //   servo_timer.add_start_millis(3);
  //   if (servo_up) {
  //     servo_pw += step;
  //     if (servo_pw > 2500) {
  //       servo_pw = 2500;
  //       servo_up = false;
  //     }
  //   } else {
  //     servo_pw -= step;
  //     if (servo_pw < 500) {
  //       servo_pw = 500;
  //       servo_up = true;
  //     }
  //   }
  //   servos[0].set_pulse_width_us(servo_pw);
  // }

  // if (on_off_timer.elapsed_millis(millis_now) >= 500) {
  //   on_off_timer.reset(millis_now);
  //   on_off = !on_off;
  //   gpio_put(kTest1Pin, true);
  //   servos[0].set_state(on_off);
  //   gpio_put(kTest1Pin, false);
  // }

  const uint32_t millis_since_cmd_start = cmd_timer.elapsed_millis(millis_now);

  // Update LED state. Solid if active or short blinks if idle.
  {
    const bool is_active = current_cmd || millis_since_cmd_start < 200;
    const bool new_led_state =
        is_active || (millis_since_cmd_start & 0b11111111100) == 0;
    if (new_led_state != last_led_state) {
      led.update(new_led_state);
      last_led_state = new_led_state;
    }
  }

  // If a command is in progress, handle it.
  if (current_cmd) {
    // Handle command timeout.
    if (millis_since_cmd_start > kCommandTimeoutMillis) {
      current_cmd->on_cmd_aborted();
      current_cmd = nullptr;
      return;
    }
    // Invoke command loop.
    const bool cmd_completed = current_cmd->on_cmd_loop();
    if (cmd_completed) {
      current_cmd = nullptr;
    }
    return;
  }

  // Not in a command. Turn off all CS outputs. Just in case.
  // all_cs_off();

  // Try to read selection char of next command.
  static_assert(sizeof(data_buffer) >= 1);
  data_size = 0;
  if (!read_serial_bytes(1)) {
    return;
  }

  // Dispatch the next command by the selection char.
  current_cmd = find_command_handler_by_char(data_buffer[0]);
  if (current_cmd) {
    cmd_timer.reset(millis_now);
    data_size = 0;
    current_cmd->on_cmd_entered();
    // We call on_cmd_loop() on the next iteration, after updating the LED.
  } else {
    // Unknown command selector. We ignore it silently.
  }
}
