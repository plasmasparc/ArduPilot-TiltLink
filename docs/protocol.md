# Protocols

## 1. Control frame — handheld → air, LoRa

Fixed 8 bytes. Any received packet of a different length is counted and dropped.

| Byte | Bits | Content |
|------|------|---------|
| 0–5 | 0–43 | four 11-bit channels, LSB first, little-endian bit packing |
| 5 | 44–47 | flags nibble |
| 6 | — | sequence number, wraps at 256 |
| 7 | — | CRC8 DVB-S2 (poly 0xD5, init 0x00) over bytes 0–6 |

Channels, in CRSF units (172 = 988 µs, 992 = 1500 µs, 1811 = 2012 µs):

| Index | Channel | Source |
|-------|---------|--------|
| 0 | roll | filtered MPU6050 roll angle |
| 1 | pitch | filtered MPU6050 pitch angle |
| 2 | throttle | potentiometer, forced to 192 when disarmed |
| 3 | yaw | fixed at 992 (neutral) |

Flags:

| Bit | Name | Meaning |
|-----|------|---------|
| 0 | `AIR_FLAG_ARM` | armed |
| 1–3 | — | reserved, zero |

The 11-bit width is deliberate: it is CRSF's own channel encoding, so the air
node moves the values into the CRSF frame without any rescaling.

The sequence number is not used for retransmission — there is none. It only lets
the receiver count lost frames as a link-quality indicator.

## 2. CRSF — air node → flight controller

Standard CrossFire framing on a 416666-baud UART, sync byte 0xC8, CRC8 DVB-S2
over type and payload.

| Type | Name | Rate | Length |
|------|------|------|--------|
| 0x16 | `RC_CHANNELS_PACKED` | 50 Hz | 26 bytes |
| 0x14 | `LINK_STATISTICS` | 5 Hz | 14 bytes |

Channel assignment as seen by ArduPilot:

| CRSF channel | Value |
|--------------|-------|
| 1 | roll, from the LoRa frame |
| 2 | pitch, from the LoRa frame |
| 3 | throttle, from the LoRa frame |
| 4 | yaw, from the LoRa frame |
| 5 | 1000 µs, fixed |
| 6 | 2000 µs when armed, 1000 µs otherwise |
| 7–16 | 1500 µs, fixed |

`LINK_STATISTICS` carries the measured LoRa RSSI, SNR and computed link quality,
so the FC and the GCS display real link state rather than a placeholder.

Link quality is computed over a 2 s window as the ratio of good frames to
expected frames, where expected is derived from `AIR_STALE_MS`, and clamped to
100.

## 3. Telemetry framing — air ↔ ground, nRF24

The link is byte-transparent: it carries the FC's MAVLink stream without parsing
it. There is no framing layer of its own beyond a single length header, so
MAVLink message boundaries are not preserved across packets and do not need to
be — the receiving MAVLink parser resynchronises on its own.

| Byte | Content |
|------|---------|
| 0 | payload length *n*, 0–31 |
| 1…n | raw stream bytes |

Header 0 is an empty poll or an empty ACK: it carries no data but keeps the
transaction cadence going, which is what allows uplink to flow while the
downlink is idle. The air node emits one every 1 ms when its buffer is empty.

Both ends validate `payload[0] == len - 1` using the dynamic payload length
reported by the hardware, and count mismatches as `badHdr`. Corruption in flight
is already caught by the 16-bit hardware CRC; a bad header means a software or
framing fault, so the two counters are diagnostically distinct.

Each side buffers 4096 bytes and drops oldest on overflow. Bytes leave the
buffer only after the hardware confirms delivery.

## 4. Direction of responsibility

| Concern | Handled by |
|---------|-----------|
| Control frame integrity | CRC8 in the LoRa frame |
| Control frame loss | none — next frame in 200 ms; 750 ms silence triggers failsafe |
| Telemetry integrity | nRF24 hardware CRC16 |
| Telemetry loss | ESB automatic retransmission, up to 15 retries |
| Duplicate telemetry | MAVLink CRC downstream |
| Attitude limits | flight controller `ANGLE_MAX` |