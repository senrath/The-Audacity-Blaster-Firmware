# The Audacity Firmware for Brushless Nerf Blasters
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

RP2040 based firmware for brushless Nerf blasters.

## Features
- Closed-loop flywheel control via Bi-directional Dshot 600 (both PID on RPM and naive throttle control available)
- Configurable select fire modes (semi, binary, burst, and full)
- Optional idle-rev mode
- Display screen and rotary encoder support
- Optional closed-loop solenoid control
- (Theoretical) Support for any ESCs that support Bi-directional Dshot 600
- Low voltage detection
- Supports both 3S and 4S batteries
- Custom PCB available [(files found here)](https://github.com/senrath/The-Audacity-Blaster-PCB)

## Required Hardware (PCBless)
See the custom PCB files for the wiring diagram.
- Microcontroller
    - The firmware assumes an RP2040-Zero's pinout, but any RP2040 board with enough GPIO pins will work with some adjustment
- Voltage Divider:
    - Resistors to form a voltage divider with approximately a 1 to 7 ratio. I used a 30k and a 4.99k resistor.
    - A ~10uF ceramic cap for the voltage divider.
- Solenoid Driver:
    - A mosfet capable of driving your solenoid of choice.
    - A pair of resistors for the pin driving the solenoid. I used a 100 and a 10k.
    - (Optional) Two diodes (zener + schottky) for a rapid decay circuit.
- Power for the RP2040:
    - A 5V buck converter.
    - Another ~10uF ceramic cap for the 5V output.
    - A diode for the line from the battery to the buck converter. I used a 50V 1A one.
    - An electrolytic cap for the line from the battery to the buck converter. The PCB is specced for a 100uF cap, but in testing I used a 47uF cap with no problems.
    - Some form of toggle switch to cut off the 5V line in case you want to connect to USB and battery at the same time.
- Power from the RP2040:
    - An electrolytic cap for the 3.3V line. The PCB is specced for a 100uF cap, but in testing I used a 47uF cap with no problems.
- (Optional) Closed Loop Solenoid Control:
    - Two pairs of IR Break Beam Sensors. I used [these ones from AdaFruit](https://www.adafruit.com/product/2167).

## Adjusting Pinout and Default Settings
The following are the default pinouts and settings in the firmware.
### Pinout
| Pin Name | What Connects Here | Default Value |
| -------- | ----------- | ------------- |
| ENC_DT | The DT pin on the rotary encoder. | 7 |
| ENC_CLK | The CLK pin on the rotary encoder. | 6 |
| ENC_SW | The SW pin on the rotary encoder. | 8 |
| VOLTAGE_PIN | The voltage divider, for low battery protection. This **must** be an ADC capable pin. | 29 |
| MOTOR1 | The first motor's esc signal wire. | 5 |
| MOTOR2 | The second motor's esc signal wire | 4 |
| MAINTRIGGER | The main trigger | 1 |
| REVTRIGGER | The rev trigger | 0 |
| OLED_SCL | The SCL pin on the OLED | 3 |
| OLED_SDA | The SDA pin on the OLED | 2 |
| OLED_RESET | The reset pin on the OLED | -1 (no pin) |
| IR_SENSOR1_PIN | The data pin on the first IR Break Beam sensor | 27 |
| IR_SENSOR2_PIN | The data pin on the second IR Break Beam sensor | 26 |
| IR_EMITTER1_PIN | The power pin on the first IR Emitter. Only used on v1.0 boards where there aren't enough 3.3V pins to power each sensor and emitter. | 15 |
| IR_EMITTER2_PIN | The power pin on the second IR Emitter. Only used on v1.0 boards where there aren't enough 3.3V pins to power each sensor and emitter. | 14 |
| SOLENOID_PIN | The mosfet that controls the solenoid | 9 |

### Settings that can only be adjusted from within the firmware
| Setting Name | Description and Possible Values | Default Value |
| ------------ | ----------- | ------------- |
| MOTOR_POLES  | How many motor poles the motors you're using have | 14 |
| MOTOR_KV     | The KV rating of the motors | 2450 |
| MOTOR1_REVERSE | If motor 1 should run in reverse or not | false |
| MOTOR2_REVERSE | Same, but motor 2 | false |
| USE_PID | Whether or not to use PID rpm control or basic throttle control | true |
| P | The proportional term of the PID controller | 10 |
| I | The integral term of the PID controller | 10 |
| D | The derivative term of the PID controller | 0 |
| ENC_LONG_CLICK | The minimum number of seconds to hold the encoder down for a long click | 2 |
| ENC_STEPS_PER_ROTATION | The number of physical clicks per rotation of the encoder | 20 |
| ENC_STEP_NORMALIZATION | How many digital steps are reported per physical click of the encoder | 2 |
| ENC_CLICK_DEBOUNCE | How many milliseconds to wait between encoder reads to avoid double clicks | 10 |
| TRIGGER_LONG_CLICK | The minimum number of seconds to hold the trigger down for a long click | 2 |
| SPINUP_SAFETY_TIME | How many milliseconds to give the motors to fully spin up, as a failsafe | 1000 |
| OLED_ADDRESS | The address of the display | 0x3C |
| OLED_WIDTH | The width of the display | 128 |
| OLED_HEIGHT | The height of the display | 64 |
| WIRE1 | Whether or not the display is on WIRE1 instead of WIRE | defined |
| USE_LOGO | Whether or not you display a logo on startup | defined |
| V1_0_BOARD | Whether or not you're using a v1.0 board (or other solution without enough direct 3.3v pins) | undefined |

### Settings that can also be adjusted via the settings menu
| Setting Name | Description and Possible Values | Default Value |
| ------------ | ----------- | ------------- |
| BAT_TYPE | What type of battery (BAT_3S or BAT_4S) | BAT_4S |
| SOLENOID_CONTROL | Whether to use open or closed-loop control for the solenoid (OPEN_LOOP or CLOSED_LOOP) | CLOSED_LOOP |
| SOLENOID_PULSE_3S_HIGH | When in open loop control and using a 3S battery, how long (in milliseconds) the solenoid stays extended when at full battery | 35 |
| SOLENOID_PULSE_3S_LOW | When in open loop control and using a 3S battery, how long (in milliseconds) the solenoid stays extended when at low battery | 40 |
| SOLENOID_PULSE_4S_HIGH | When in open loop control and using a 4S battery, how long (in milliseconds) the solenoid stays extended when at full battery | 30 |
| SOLENOID_PULSE_4S_LOW | When in open loop control and using a 4S battery, how long (in milliseconds) the solenoid stays extended when at low battery | 35 |
| SOLENOID_RETRACT | When in open loop control, how long (in milliseconds) the solenoid is given to fully retract | 30 |
| MOTOR_SPEED | Target motor speed (10-100, percentage) | 50 |
| IDLE_SPEED | Target motor speed while idling (0-100, percentage) | 15 |
| MOTOR_HOLD_TIME | How many milliseconds the motors will wait before spinning down after finishing firing | 200 |
| FULL_AUTO_ROF | Rate of fire in full auto mode (0-100, 0 is uncapped) | 0 |
| BURST_FIRE_ROF | Rate of fire in burst fire mode (0-100, 0 is uncapped) | 0 |
| BURST_SIZE | Number of darts per burst (2-100) | 3 |
| REV_TRIGGER_OPTION | What the rev trigger does (REV_DISABLED, REV_IDLE_TOGGLE, REV_IDLE_HOLD, REV_FULL_HOLD) | REV_FULL_HOLD |
| TRIGGER_DEBOUNCE | How many milliseconds does the blaster wait for the trigger signal to stabilize before registering a state change | 20 |

The solenoid pulse durations share one menu option each (high, low) with whatever battery type selected being the one that's available.

### Sanity check settings
These are used to clamp the possible values when adjusting from within the settings menu, as well as for sanity checking on initial load
| Setting Name | Description and Possible Values | Default Value |
| ------------ | ------------------------------- | ------------- |
| MIN_MOTOR_SPEED | The minimum speed allowed (percentage) | 25 |
| MAX_MOTOR_SPEED | The maximum speed allowed (percentage) | 100 |
| MIN_IDLE_SPEED | The minimum speed allowed for idle mode (percentage) | 0 |
| MAX_IDLE_SPEED | The maximum speed allowed for idle mode (percentage) | 100 |
| MIN_ROF | The minimum ROF allowed for burst and full auto | 0 | 
| MAX_ROF | The maximum ROF allowed for burst and full auto | 100 |
| MIN_BURST | The minimum number of darts per burst | 2 |
| MAX_BURST | The maximum number of darts per burst | 100 |
| MIN_PULSE | The minimum number of milliseconds that the solenoid can be set to extend or retract for (OPEN_LOOP only) | 5 |
| MAX_PULSE | The maximum number of milliseconds that the solenoid can be set to extend or retract for (OPEN_LOOP only) | 1000 |
| MIN_DEBOUNCE | The minimum number of milliseconds that the blaster can wait to debounce trigger inputs | 0 |
| MAX_DEBOUNCE | The maximum number of milliseconds that the blaster can wait to debounce trigger inputs | 500 |
| MIN_HOLD_TIME | The minimum number of milliseconds the blaster can be set to wait before letting the motors spin down | 0 |
| MAX_HOLD_TIME | The maximum number of milliseconds the blaster can be set to wait before letting the motors spin down | 1000 |

## Changing The Startup Logo
- If you want a different startup logo, you can by using [this website](https://javl.github.io/image2cpp/) with the following settings (assuming you're using a 128x64 SSD1306 screen)
    - Canvas size: 128x64
    - Background color: White (if you want the image to be drawn in dark pixels like the default) or Black (if you want the image to be drawn in light pixels)
    - Scaling: scale to fit, keeping proportions
    - Center image: your choice, I did both
    - Code output format: Arduino code, single bitmap
    - Identifier/Prefix: LOGO
    - Draw mode: Horizontal - 1 bit per pixel
- Copy the resulting output and replace the contents of logo.h with it.

## Flashing Your Board
There are two ways to use flash the firmware onto your board
### 1. PlatformIO
- Install [PlatformIO](https://platformio.org/platformio-ide).
- Clone the main branch of the repo.
- Open the cloned repo folder within the PlatformIO IDE.
- Connect your RP2040 board and click the Upload option from within PlatformIO.
- If you get an error about "Filename too long" then you need to follow [these instructions](https://arduino-pico.readthedocs.io/en/latest/platformio.html#important-steps-for-windows-users-before-installing) about enabling long paths.
### 2. Arduino IDE
- Install [Arduino IDE](https://www.arduino.cc/en/software/).
- Clone the arduino-ide branch of the repo.
- Open the firmware.ino file within Arduino IDE.
- Install the Raspberry Pi Pico/RP2040/RP2350 boards as per [these instructions](https://arduino-pico.readthedocs.io/en/latest/install.html#installing-via-arduino-boards-manager).
- Install the following libraries (Tools -> Manage Libraries):
    - Adafruit BusIO
    - Adafruit GFX Library
    - Adafruit SSD1306
    - NewEncoder by Alex Casal
    - Pico_Bidir_DShot by Bastian2001
    - PID by Brett Beauregard
    - OneButton by Matthias Hertel
- Select the "Raspberry Pi Pico" board.
- Connect your RP2040 board and click Upload from within Arduino IDE.

## Operating The Blaster
- While on the main screen, rotate the encoder to set the target power.
- Click the encoder to cycle between fire modes.
- Holding down the trigger for several seconds while in semi-auto will toggle the idle rev mode.
- Click and hold the encoder for several seconds to enter the menu.
- Errors can be cleared by clicking and holding the encoder for several seconds, assuming you are confident the source of the error has been fixed.

# Licensing
This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. If not, see <https://www.gnu.org/licenses/>.