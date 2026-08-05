# Hardware overview

Three independent nodes. Each has its own wiring sheet:

- [`handheld-lolin-d32.md`](handheld-lolin-d32.md) — the transmitter you hold
- [`air-rp2040-zero.md`](air-rp2040-zero.md) — the bridge on the aircraft
- [`ground-lolin-d32.md`](ground-lolin-d32.md) — the telemetry ground station

## System

| Link | Band | Modulation | Direction | Payload |
|------|------|-----------|-----------|---------|
| Control | 869.5 MHz | LoRa SF7 / BW500 / CR4/5 | handheld → air, one way | 8-byte frame at 5 Hz |
| Telemetry | 2476 MHz (ch 76) | nRF24 ESB, 2 Mbps | air ↔ ground, bidirectional | up to 31 B per transaction |
| Aircraft RC | wired | CRSF 416666 baud | air → FC | 16 channels at 50 Hz |
| Aircraft telemetry | wired | MAVLink 2 | air ↔ FC | 57600 baud |

## Common requirements

**nRF24 supply.** Both nRF24 modules need 100 µF electrolytic in parallel with
100 nF ceramic soldered directly across the module's VCC–GND pins. The PA
variants draw current in bursts that the on-board 3.3 V regulator of a dev board
cannot follow; without the local reservoir the link fails intermittently in ways
that look like RF problems.

**nRF24 SPI clock.** Clone nRF24 modules are unreliable above ~4 MHz on the
`arduino-pico` core. Both sketches pass `4000000` as the third `RF24`
constructor argument for this reason. Do not raise it without measuring.

**Antennas.** Every radio needs its antenna connected before power-up. The
SX1276 at +20 dBm PA_BOOST and the nRF24 PA modules will damage themselves into
an open load.

**Radio parameter symmetry.** The two ends of each link must agree on every
parameter. For nRF24 that means channel, data rate, PA level, CRC length,
address width, address, retry setup, and the auto-ack / dynamic-payload /
ack-payload triple. For LoRa: frequency, bandwidth, spreading factor, coding
rate, sync word, and preamble length.

## Bill of materials

| Qty | Part | Node |
|-----|------|------|
| 2 | LOLIN D32 (ESP32-WROOM) | handheld, ground |
| 1 | RP2040-Zero | air |
| 2 | SX1276 868 MHz module + antenna | handheld, air |
| 2 | nRF24L01+PA+LNA module + antenna | air, ground |
| 1 | MPU6050 breakout | handheld |
| 1 | 10 kΩ linear potentiometer | handheld |
| 1 | SPST toggle switch | handheld |
| 2 | 100 µF electrolytic + 100 nF ceramic | air, ground (nRF24 decoupling) |
| 1 | DAKE FPV F405 flight controller (or any ArduPilot board) | aircraft |
