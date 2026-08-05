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

## RC channel functions

| Parameter | Value | Effect |
|-----------|-------|--------|
| `RC6_OPTION` | 153 | channel 6 is the arm/disarm switch |
| `ANGLE_MAX` | 2000–3000 | maximum commanded lean angle, in centidegrees |

`ANGLE_MAX` is the hard ceiling on what the tilt controller can command. With
`USE_FULL_STICK_RANGE 0` in the handheld sketch, the scaling assumes 3000; with
it set to 1, set `ANGLE_MAX 2000` and the FC enforces the ±20° limit itself.
The second arrangement is preferable — no firmware bug in the handheld can
exceed a limit the FC applies.

## Failsafe

The air node stops emitting CRSF after 750 ms of LoRa silence. Configure the FC's
own RC failsafe (`FS_THR_ENABLE`, `RC_FS_TIMEOUT`) for what should happen then;
this project does not substitute for it.

Because the handheld link is one-way, there is no "transmitter says it lost the
aircraft" path. Loss of control and loss of telemetry are independent events on
independent bands, and the failsafe must be configured for either happening
alone.

## Bench checklist

1. Propellers off.
2. Both radios have antennas.
3. Air node USB serial shows `[LORA] init=1` and rising `good`, with `crcBad`
   near zero.
4. Air node shows `[NRF] init=1`, `txOk` rising, `txFail` near zero.
5. GCS connects and `link=UP` appears in the ground station diagnostics.
6. Tilt the handheld and confirm the FC's RC input page moves channels 1 and 2.
7. Arm switch: confirm channel 6 goes to 2000 µs and that arming is refused
   above minimum throttle.
8. Only then fit propellers.

