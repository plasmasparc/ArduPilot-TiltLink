# Flight controller setup (ArduPilot)

Reference target: DAKE FPV F405 (STM32F405), ArduCopter 4.8-dev. Any ArduPilot
board with two spare UARTs works; only the port numbers change.

## Serial ports

| Port | Protocol | Baud | Use |
|------|----------|------|-----|
| SERIAL1 | 23 (RCIN) | 115 | CRSF from the air node |
| SERIAL5 | 2 (MAVLink 2) | 57 | telemetry to the air node |

```
SERIAL1_PROTOCOL 23
SERIAL1_BAUD     115
SERIAL1_OPTIONS  0
SERIAL5_PROTOCOL 2
SERIAL5_BAUD     57
SERIAL5_OPTIONS  0
```

On F405 the `SERIALx_OPTIONS` inversion (bits 0–1) and TX/RX swap (bit 3) are not
implemented — they exist only on F7 and H7. `OPTIONS 0` is mandatory; wire the
cross-connection correctly in hardware instead.

CRSF at 416666 baud needs a DMA-capable UART for RC input. Check the board's
`SERIAL_ORDER` and pick a port that has DMA on both directions before assuming
SERIAL1 is the right one on your hardware.

## Telemetry stream rates

From ArduPilot 4.7 onwards the stream-rate parameters are named `MAVx_` rather
than `SRx_`, and *x* indexes the MAVLink-protocol ports in order, not the
`SERIALx` number. With the configuration above, SERIAL0 is `MAV1_` and SERIAL5
is `MAV2_`.

Set bit 2 (value 4) in `MAV2_OPTIONS`, otherwise the GCS overrides the rates you
configure as soon as it connects.

The link carries roughly 5760 B/s at 57600 baud. Set stream rates so the FC does
not try to push more than the UART can carry — the radio has ninefold headroom
above the UART, so the UART is the binding constraint, not the RF link.
