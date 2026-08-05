# Air node — RP2040-Zero + SX1276 + nRF24L01+PA

Sketch: `firmware/01_air_pico_lora_rc_nrf24_mavlink/`
FQBN: `rp2040:rp2040:rpipico`
Libraries: RadioLib, RF24 (TMRh20)

## Function

The only node on the aircraft. Both RP2040 cores run continuously and
independently:

- **Core 0** receives the LoRa control frame, validates it, unpacks the four
  11-bit channels, and emits CRSF `RC_CHANNELS_PACKED` at 50 Hz plus
  `LINK_STATISTICS` at 5 Hz to the flight controller.
- **Core 1** is a transparent bidirectional MAVLink pipe between the flight
  controller UART and the nRF24 link.

The split matters: a stalled telemetry link cannot delay a control frame,
because they do not share a core, a SPI bus, or a UART.

## Pinout

### SX1276 — SPI0, core 0

| SX1276 | RP2040 GP | Note |
|--------|-----------|------|
| MISO | 0 | SPI0 RX |
| CS | 1 | |
| SCK | 2 | SPI0 SCK |
| MOSI | 3 | SPI0 TX |
| DIO0 | 14 | RX-done interrupt |
| RST | 15 | |
| VCC | 3V3 | |
| GND | GND | |

### nRF24L01+PA — SPI1, core 1

| nRF24 | RP2040 GP | Note |
|-------|-----------|------|
| CE | 7 | |
| MISO | 8 | SPI1 RX |
| CSN | 9 | |
| SCK | 10 | SPI1 SCK |
| MOSI | 11 | SPI1 TX |
| VCC | 3V3 | 100 µF ‖ 100 nF directly across VCC–GND |
| GND | GND | |

### UARTs to the flight controller

| Function | RP2040 GP | FC pad | FC port |
|----------|-----------|--------|---------|
| CRSF TX | 12 (UART0 TX) | R1 | SERIAL1 |
| CRSF RX | 13 (UART0 RX) | T1 | SERIAL1 (unused by this firmware) |
| MAVLink TX | 4 (UART1 TX = `Serial2`) | R5 | SERIAL5 |
| MAVLink RX | 5 (UART1 RX = `Serial2`) | T5 | SERIAL5 |

Cross-connected: every TX goes to the other end's RX. Common ground between the
bridge and the FC is required.

GP0/GP1 are the default UART0 pins on this core but are used here by the SX1276,
so the CRSF UART is explicitly moved to GP12/GP13 with `setTX()`/`setRX()`.

## Radio configuration

### LoRa (must match the handheld)

869.5 MHz, BW 500 kHz, SF7, CR 4/5, sync word 0x3C, preamble 8, +17 dBm, current
limit 140 mA, explicit header, CRC on. `setFrequency()` is called again after
`begin()` for the BW500 errata fix.

### nRF24 (must match the ground station)

| Parameter | Value | Reason |
|-----------|-------|--------|
| Channel | 76 (2476 MHz) | 2 Mbps occupies 2 MHz; keep clear of WiFi |
| Data rate | 2 Mbps | see the ACK-payload timing below |
| PA level | MAX | |
| CRC | 16-bit | |
| Address width | 5 bytes | |
| Address | 0xB0E1F0A7D2 | |
| Retries | ARD = 1 → 500 µs, ARC = 15 | datasheet minimum for ACK payload |
| Auto-ack | on | prerequisite for ACK payload |
| Dynamic payloads | on | prerequisite for ACK payload |
| ACK payload | on | this is the bidirectional mechanism |
| SPI clock | 4 MHz | clone modules fail at 10 MHz |

## Telemetry transport

The air node is permanently PTX and never switches mode. Each transaction is:
peek up to 31 bytes from the ring buffer → `write()` → hardware ACK → skip. The
uplink arrives inside that same ACK. There is no turnaround, no poll cycle, and
no waiting for a reply.

Frame format on the nRF24 link: `payload[0]` is the byte count (0–31),
`payload[1..n]` is raw MAVLink. Header 0 is an empty poll — sent every 1 ms when
the downlink buffer is empty, so the uplink always has a carrier to ride on. The
receiver validates that `payload[0] == len - 1` and discards mismatches.

Sizing at 2 Mbps: a packet is 8 preamble + 40 address + 9 PCF + 256 payload + 16
CRC = 329 bits = 164.5 µs, and the ACK with a 32-byte payload is the same. One
transaction is roughly 130 µs PLL + 165 µs TX + 165 µs ACK + ~130 µs SPI ≈
0.6 ms, giving about 51 kB/s per direction against a 5760 B/s load at 57600 baud
— a factor of nine in reserve.

The same scheme fails at 250 kbps, and that failure is what forced 2 Mbps: a
32-byte ACK payload takes 1316 µs of air time plus 130 µs PLL there, which an
ARD of 1500 µs does not cover, so the ACK is still in flight when the retry
fires and never arrives.

Data is removed from the ring buffer only after `write()` returns true, i.e.
after a hardware ACK. A lost ACK can therefore duplicate one packet; the MAVLink
CRC rejects it downstream.

## Failsafe

If no valid LoRa frame arrives for `AIR_STALE_MS` (750 ms), the node forces
throttle and the arm channel to 1000 µs and stops sending CRSF, which triggers
the flight controller's own RC failsafe.

An nRF24 initialisation failure sets a flag and leaves core 1 idling — it does
not halt the board, because the control path on core 0 must keep running.

## Diagnostics

USB serial, 115200, 2 Hz. `Serial.ignoreFlowControl(true)` is required on this
core, otherwise a host that does not raise DTR (QGroundControl among them) sees
nothing.

- `[LORA]` — init state, IRQ count, good frames, CRC failures, wrong-length
  frames, frames lost by sequence gap, link quality, RSSI, freshness, arm state,
  channel 1 and 3.
- `[NRF]` — init state, TX ok/fail, ACK packets received, bytes each way, bad
  headers, current auto-retry count (a usable range proxy), ring buffer
  occupancy, and drops.
