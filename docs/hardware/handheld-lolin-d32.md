# Handheld transmitter — LOLIN D32 + SX1276

Sketch: `firmware/03_handheld_d32_lora_rc/`
FQBN: `esp32:esp32:lolin_d32`
Library: RadioLib

## Function

The unit is the stick. Its physical roll and pitch angle, derived from an
MPU6050 by a Mahony AHRS filter, become the roll and pitch stick positions. A
potentiometer is throttle, a toggle switch is arm. Four channels are packed as
11-bit values — the native CRSF encoding — into an 8-byte frame and transmitted
over LoRa at a fixed rate. Nothing comes back on this link.

## Pinout

### SX1276 — VSPI

| SX1276 | ESP32 GPIO | Note |
|--------|-----------|------|
| SCK | 18 | VSPI SCK |
| MISO | 19 | VSPI MISO |
| MOSI | 23 | VSPI MOSI |
| NSS | 5 | chip select |
| RST | 32 | |
| DIO0 | 26 | TX-done interrupt |
| VCC | 3V3 | |
| GND | GND | |

### MPU6050 — I²C

| MPU6050 | ESP32 GPIO |
|---------|-----------|
| SDA | 21 |
| SCL | 22 |
| VCC | 3V3 |
| GND | GND |
| AD0 | GND (address 0x68) |

Bus clock 400 kHz, set in `initMPU6050()`.

### Controls

| Signal | ESP32 GPIO | Wiring |
|--------|-----------|--------|
| Throttle pot | 36 (A0) | wiper to GPIO36, ends to 3V3 and GND; ADC1, 12-bit |
| Arm switch | 12 | switch to GND, `INPUT_PULLUP`; **LOW = armed** |

> **GPIO12 is a strapping pin (MTDI).** Held HIGH at boot it selects a 1.8 V
> flash supply and most modules will not start. `INPUT_PULLUP` leaves it HIGH
> whenever the switch is open. If the board fails to boot with the arm switch
> released, move the switch to GPIO 13, 14, 15 or 27 and change `PIN_ARM`.
>
> GPIO36 is input-only and has no internal pull-up. That is correct here — a
> potentiometer wiper drives it — but nothing else can share the pin.

## Radio configuration

| Parameter | Value |
|-----------|-------|
| Frequency | 869.5 MHz |
| Bandwidth | 500 kHz |
| Spreading factor | 7 |
| Coding rate | 4/5 |
| Sync word | 0x3C |
| Preamble | 8 symbols |
| TX power | +20 dBm (PA_BOOST) |
| Current limit | 140 mA |
| Frame rate | 5 Hz |

Time on air is 9.02 ms for the 8-byte frame at BW500, giving 4.5% duty at 5 Hz.
At BW250 it is 18.05 ms (9.0%) and at BW125 36.10 ms (18.0%) — the last exceeds
the 10% g3 allowance at this rate.

A +20 dBm PA_BOOST transmission needs the current limit raised from the 60 mA
RadioLib default; `setCurrentLimit(140)` does that. The 500 kHz errata fix
(registers 0x36 and 0x3A) is applied by `setFrequency()`, which must therefore
be called again after `begin()` — inside `begin()` the bandwidth is not yet 500
kHz when the frequency is set, so the fix is skipped.

## Behaviour

**Arming.** The switch alone does not arm. On switch-on the throttle pot must be
below 5% of full scale (`POT_ARM_MAX_RAW 205` of 4095), which sets a latch;
throttle can then be raised freely. Turning the switch off clears the latch, and
minimum throttle is required again on the next attempt. While disarmed, channel
3 is forced to 1000 µs.

**Tilt scaling.** ±90° of physical tilt maps to a stick deflection that commands
±20° of lean, assuming FC `ANGLE_MAX` is 30° (3000). A 2° deadband suppresses
hand tremor and a first-order low-pass with α = 0.02 at roughly 1 kHz sample
rate (τ ≈ 50 ms) smooths the rest. Setting `USE_FULL_STICK_RANGE` to 1 sends
full stick travel instead and delegates the limit to `ANGLE_MAX 2000` on the FC,
which is the safer arrangement: no code path can then exceed the limit.

**Diagnostics.** 2 Hz on USB serial at 115200: switch state, arm latch, filtered
roll/pitch, raw pot, resulting CRSF channel values, frames sent and skipped, and
the measured IMU update rate.
