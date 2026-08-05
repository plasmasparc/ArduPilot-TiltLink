# Ground station — LOLIN D32 + nRF24L01+PA

Sketch: `firmware/02_ground_d32_nrf24_mavlink/`
FQBN: `esp32:esp32:lolin_d32`
Libraries: RF24 (TMRh20) v1.4+, WiFi (ESP32 core)

## Function

The telemetry counterpart of the air node. It receives the MAVLink downlink over
nRF24 and republishes it on two transports at once — USB serial and WiFi UDP —
and accepts uplink from either, forwarding it in the ACK payload. It handles no
control traffic; the handheld transmitter talks to the aircraft directly.

## Pinout

### nRF24L01+PA — VSPI

| nRF24 | ESP32 GPIO |
|-------|-----------|
| SCK | 18 |
| MISO | 19 |
| MOSI | 23 |
| CSN | 5 |
| CE | 17 |
| VCC | 3V3 (100 µF ‖ 100 nF directly across VCC–GND) |
| GND | GND |

Chip select is left to the RF24 library, hence `SPI.begin(18, 19, 23, -1)`.

## Radio configuration

Identical to the air node in every field: channel 76, 2 Mbps, PA max, CRC 16,
address width 5, address 0xB0E1F0A7D2, `setRetries(1, 15)`, auto-ack on, dynamic
payloads on, ACK payload on, SPI 4 MHz. Any mismatch and the link is silent.

This side is permanently PRX and never leaves `startListening()`.

## ACK payload timing

The nRF24 hardware assembles the ACK the moment a packet arrives, so the payload
must already be in the TX FIFO **before** that. The queue is therefore topped up
continuously — whenever the ring buffer is non-empty and again immediately after
every reception — not only in response to traffic. The FIFO is three deep;
`writeAckPayload()` returns false when full, and the bytes stay in the ring
buffer (peek → write → skip) rather than being lost.

Uplink frame format matches the downlink: `ackbuf[0]` is the byte count,
`ackbuf[1..n]` is raw MAVLink, and the air node validates the length header.

## WiFi and the 2.4 GHz conflict

The ESP32 WiFi radio and the nRF24 share the band, on the same board, centimetres
apart. The WiFi power amplifier can desensitise the nRF24 receiver at close
range. `USE_WIFI` exists as a measurement tool as much as a feature: set it to 0
and reflash. If the link improves abruptly, WiFi is the cause and the answer is
physical separation, shielding, or moving the nRF24 channel further from the
WiFi channel in use.

`WiFi.setSleep(false)` disables modem sleep, which otherwise adds tens of
milliseconds of jitter to UDP delivery.

## Transports

| Transport | Setting |
|-----------|---------|
| USB serial | 115200 baud, 2048-byte RX buffer |
| Telemetry UDP | port 14550, aggregated to 512 bytes or 20 ms, whichever comes first |
| Diagnostics UDP | port 14551, 2 Hz text |

The UDP peer is learned from the first inbound datagram and held for 10 s after
the last one; with no known peer, output is broadcast to port 14550. That means
a GCS only has to send one packet to register itself.

The UDP aggregation window exists because one datagram per MAVLink byte would
swamp both the ESP32 IP stack and the WiFi air time. 20 ms is short enough not
to matter for telemetry.

WiFi reconnection is retried every 30 s without blocking the radio loop.

## Connecting a GCS

```bash
# WiFi, preferred
mavproxy.py --master=udp:0.0.0.0:14550 --console

# USB fallback
mavproxy.py --master=/dev/cu.usbserial-XXXX,115200 --console

# live diagnostics
nc -ulk 14551
```

If another process already holds 14550, change `UDP_PORT` to 14555 and reflash.
QGroundControl connects to the same UDP port.

## Diagnostics fields

`link` (UP if a packet arrived within 3 s), `age` of the last reception, packets
and bytes received, ACK payloads loaded and their byte count, bad headers, ring
buffer occupancy, dropped bytes, UDP datagrams sent, and UDP bytes received.

`drop` rising means the uplink is being generated faster than the ACK queue can
carry it. `badHdr` rising means framing corruption, not RF loss — the nRF24 CRC
would have caught the latter.

## Configuration before flashing

`WIFI_SSID` and `WIFI_PASS` are compile-time constants at the top of the sketch
and must be set for your network.
