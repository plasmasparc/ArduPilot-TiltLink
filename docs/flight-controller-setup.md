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

