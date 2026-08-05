# TiltLink

**Motion-controlled RC and dual-band telemetry link for ArduPilot.**

TiltLink replaces the conventional stick transmitter and the separate telemetry
radio with three small microcontroller nodes. A handheld unit is flown by
tilting it: an MPU6050 measures roll and pitch, a potentiometer sets throttle,
and a switch arms the aircraft. The resulting stick values are sent over a
sub-GHz LoRa link to a bridge on the aircraft, which converts them to CRSF and
feeds them into the flight controller exactly as a normal RC receiver would.

Telemetry runs on a second, independent link. The same on-board bridge carries
bidirectional MAVLink over 2.4 GHz nRF24 to a ground station, which exposes the
stream simultaneously as USB serial and as WiFi UDP, so MAVProxy or QGroundControl
can connect without any additional hardware.

## Why two bands

Control is a few bytes at a low, strictly periodic rate and needs range and 
robustness — LoRa at 869.5 MHz
delivers that within the duty-cycle budget of the European g3 sub-band.
Telemetry is a continuous 57600-baud byte stream and needs throughput and low
latency — 2.4 GHz nRF24 in Enhanced ShockBurst mode at
2 Mbps delivers roughly nine times the required capacity per direction, with no
duty-cycle limit under EN 300 328. Splitting them means neither link has to be
compromised for the other, and a telemetry stall can never delay a control frame.

## Nodes

| # | Node | MCU | Radio | Role |
|---|------|-----|-------|------|
| 01 | AIR | RP2040-Zero | SX1276 + nRF24L01+PA | LoRa→CRSF on core 0, MAVLink↔nRF24 on core 1 |
| 02 | GROUND | LOLIN D32 (ESP32) | nRF24L01+PA | MAVLink to USB serial and WiFi UDP |
| 03 | HANDHELD | LOLIN D32 (ESP32) | SX1276 | MPU6050 tilt, throttle pot, arm switch → LoRa |

```
   HANDHELD                    AIR (RP2040-Zero)                 FLIGHT CONTROLLER
  ESP32+SX1276  --869.5 MHz-->  SX1276 -> CRSF  ---UART--------> SERIAL1 (CRSF)
                    LoRa                                          DAKE FPV F405
                                nRF24  <-> MAVLink <--UART-----> SERIAL5 (MAVLink)
                                  ^
                                  | 2.4 GHz ESB, 2 Mbps
                                  v
   GROUND (ESP32+nRF24) --> USB serial 115200 + WiFi UDP 14550 --> MAVProxy / QGC
```

## Repository layout

```
firmware/   three Arduino sketches, one folder per node
docs/       hardware wiring, over-the-air protocol, flight controller setup
```

## Build

Arduino IDE or `arduino-cli`. Libraries: **RadioLib**, **RF24** (TMRh20).

| Node | FQBN |
|------|------|
| AIR | `rp2040:rp2040:rpipico` |
| GROUND | `esp32:esp32:lolin_d32` |
| HANDHELD | `esp32:esp32:lolin_d32` |

Before flashing the ground node, set `WIFI_SSID` / `WIFI_PASS` at the top of
`02_ground_d32_nrf24_mavlink.ino`. Every `NRF_*` define must be identical on the
air and ground nodes, and every `LORA_*` / `RF_*` define identical on the air
and handheld nodes.

## Safety

- Never power either radio without an antenna attached.
- Remove propellers for all bench testing.
- The handheld unit commands attitude directly; the FC `ANGLE_MAX` parameter is
  the only hard limit on commanded lean angle.
- 869.4–869.65 MHz is a 10% duty-cycle sub-band in the EU. At SF7/BW500 and
  5 Hz the frame duty is 4.5%; changing bandwidth or rate changes that number.
  Verify against local regulations before transmitting.
