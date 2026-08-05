// 03_handheld_d32_lora_rc.ino  —  HANDHELD transmitter, LOLIN D32 + SX1276
// Control and arming only; no telemetry on this link.
//
// MPU6050 tilt + potentiometer + arm switch -> 8-byte LoRa frame -> air node
// (01_air_...) -> CRSF -> FC SERIAL1. Channels are packed 11-bit, the native
// CRSF format, so the receiver needs no conversion.
//
// At SF7/BW500 the 8-byte frame takes 9.02 ms on air, so 5 Hz is 4.5% duty in
// the 869.4-869.65 MHz g3 sub-band (10% limit). BW250 doubles it, BW125
// quadruples it. Full table and pinout: docs/hardware/handheld-lolin-d32.md
//
// Library: RadioLib        FQBN: esp32:esp32:lolin_d32
// Never transmit without an antenna.

#include <RadioLib.h>
#include <SPI.h>
#include "v1_MPU6050.h"

// ---- user configuration ----------------------------------------------------
#define TX_RATE_HZ      5         // duty = ToA * TX_RATE_HZ / 1000

#define RF_FREQ        869.5      // MHz, centre of the g3 sub-band
#define RF_BW          500.0      // kHz (500 / 250 / 125), must match the receiver
#define RF_SF          7
#define RF_CR          5
#define RF_SYNCWORD    0x3C
#define RF_PREAMBLE    8
#define RF_POWER       20         // dBm PA_BOOST (max)
#define RF_CURRENT_MA  140        // the 60 mA default is not enough for +20 dBm

// GPIO12 is a strapping pin (MTDI): held HIGH at boot it sets the flash supply
// to 1.8 V and the board will not start. INPUT_PULLUP leaves it HIGH with the
// switch open. If it fails to boot, move to GPIO 13, 14, 15 or 27.
#define PIN_ARM         12        // LOW = ARMED
#define PIN_POT         A0        // GPIO36

// Tilt -> stick. 0 = scale here, +/-90 deg tilt -> +/-20 deg with ANGLE_MAX
// 3000. 1 = full stick travel, limit enforced by the FC via ANGLE_MAX 2000.
#define USE_FULL_STICK_RANGE  0

#define INPUT_RANGE_DEG    90.0f
#define TILT_LIMIT_DEG     20.0f
#define FC_ANGLE_MAX_DEG   30.0f

#define INVERT_ROLL     0
#define INVERT_PITCH    0

#define TILT_LPF_ALPHA     0.02f   // alpha = dt/(tau+dt); ~1 kHz, tau ~50 ms
#define TILT_DEADBAND_DEG  2.0f

// Arm only at minimum throttle. ArduPilot would reject a high-throttle arm
// anyway; this also stops the switch from generating rejected frames.
#define ARM_REQUIRES_LOW_THROTTLE  1
#define POT_ARM_MAX_RAW            205    // 4095 * 5%
// ----------------------------------------------------------------------------

// SX1276 pinout, LOLIN D32 + VSPI
#define LORA_SCK   18
#define LORA_MISO  19
#define LORA_MOSI  23
#define LORA_CS     5
#define LORA_RST   32
#define LORA_DIO0  26

// over-the-air frame, 8 bytes (see the air node)
#define AIR_LEN         8
#define AIR_FLAG_ARM    0x01

// CRSF units: 172 = 988 us, 992 = 1500 us, 1811 = 2012 us
#define CRSF_MID      992
#define CRSF_US1000   192
#define CRSF_SPAN     800          // 992 -> 1792 = 1500 -> 2000 us

#define TX_PERIOD_MS   (1000UL / TX_RATE_HZ)
#define TX_WATCHDOG_MS  60UL       // ToA max 36 ms (BW125), so 60 ms is ample

SX1276 radio = new Module(LORA_CS, LORA_DIO0, LORA_RST);

// ---- shared state, core 1 -> core 0 ----------------------------------------
portMUX_TYPE ctlMux = portMUX_INITIALIZER_UNLOCKED;
volatile float    shRollF = 0, shPitchF = 0;
volatile int      shPot = 0;
volatile bool     shArmSw = false;
volatile uint32_t shImuHz = 0;

// ---- radio state -----------------------------------------------------------
volatile bool txDone = false;
bool     transmitting = false;
uint32_t txStartMs = 0, lastTxMs = 0, lastLogMs = 0;
uint8_t  air[AIR_LEN];
uint8_t  seq = 0;
uint32_t nSent = 0, nSkipped = 0;
bool     armLatch = false;

void IRAM_ATTR setTxFlag(void) { txDone = true; }

static uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a) {
  crc ^= a;
  for (uint8_t i = 0; i < 8; i++)
    crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
  return crc;
}
static uint8_t crc8_buf(const uint8_t *d, size_t n) {
  uint8_t c = 0;
  for (size_t i = 0; i < n; i++) c = crc8_dvb_s2(c, d[i]);
  return c;
}

// ---- tilt -> stick, in CRSF units ------------------------------------------
uint16_t tiltToCrsf(float deg, bool invert) {
  if (fabsf(deg) < TILT_DEADBAND_DEG) deg = 0.0f;
  float f = deg / INPUT_RANGE_DEG;
  if (f >  1.0f) f =  1.0f;
  if (f < -1.0f) f = -1.0f;
  if (invert) f = -f;
#if USE_FULL_STICK_RANGE
  float span = (float)CRSF_SPAN;
#else
  float span = (float)CRSF_SPAN * (TILT_LIMIT_DEG / FC_ANGLE_MAX_DEG);
#endif
  int v = CRSF_MID + (int)lroundf(f * span);
  if (v < 172)  v = 172;
  if (v > 1811) v = 1811;
  return (uint16_t)v;
}

uint16_t potToCrsfThrottle(int raw) {
  int v = CRSF_US1000 + (int)((long)raw * (long)(1792 - CRSF_US1000) / 4095L);
  if (v < 172)  v = 172;
  if (v > 1811) v = 1811;
  return (uint16_t)v;
}

// ---- 11-bit packing --------------------------------------------------------
static void packAir(uint8_t *p, const uint16_t *c4, uint8_t flags) {
  memset(p, 0, 6);
  uint32_t acc = 0; uint8_t bits = 0, o = 0;
  for (uint8_t i = 0; i < 4; i++) {
    acc |= ((uint32_t)(c4[i] & 0x7FF)) << bits;
    bits += 11;
    while (bits >= 8) { p[o++] = (uint8_t)(acc & 0xFF); acc >>= 8; bits -= 8; }
  }
  acc |= ((uint32_t)(flags & 0x0F)) << bits;
  p[o] = (uint8_t)(acc & 0xFF);
}

// ============================================================================
//  CORE 1 — IMU + potentiometer + switch
// ============================================================================
void imuTask(void *arg) {
  initMPU6050();
  static float fR = 0, fP = 0;
  uint32_t cnt = 0, t0 = millis();

  for (;;) {
    updateMPU6050();
    fR += TILT_LPF_ALPHA * (roll  - fR);
    fP += TILT_LPF_ALPHA * (pitch - fP);

    int  pot = analogRead(PIN_POT);
    bool sw  = (digitalRead(PIN_ARM) == LOW);

    portENTER_CRITICAL(&ctlMux);
    shRollF = fR; shPitchF = fP; shPot = pot; shArmSw = sw;
    portEXIT_CRITICAL(&ctlMux);

    if (++cnt >= 200) {
      uint32_t dt = millis() - t0;
      portENTER_CRITICAL(&ctlMux);
      shImuHz = dt ? (200UL * 1000UL / dt) : 0;
      portEXIT_CRITICAL(&ctlMux);
      cnt = 0; t0 = millis();
    }
    vTaskDelay(1);
  }
}

// ============================================================================
//  CORE 0 — radio
// ============================================================================
void setup() {
  pinMode(PIN_ARM, INPUT_PULLUP);
  analogReadResolution(12);
  Serial.begin(115200);

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);

  int st = radio.begin(RF_FREQ, RF_BW, RF_SF, RF_CR,
                       RF_SYNCWORD, RF_POWER, RF_PREAMBLE);
  Serial.print(F("LoRa begin = ")); Serial.println(st);
  if (st != RADIOLIB_ERR_NONE) { while (true) { delay(1000); } }

  // RadioLib applies the SX1276 BW500 errata fix (Errata 2.1: reg 0x36 = 0x02,
  // reg 0x3a = 0x64) inside setFrequency(). begin() sets frequency before
  // bandwidth, so the condition is still false there and the registers are
  // never written. This second call, with BW already at 500 kHz, applies it.
  radio.setFrequency(RF_FREQ);

  radio.setCurrentLimit(RF_CURRENT_MA);
  radio.explicitHeader();
  radio.setCRC(true);
  radio.setDio0Action(setTxFlag, RISING);
  radio.standby();

  xTaskCreatePinnedToCore(imuTask, "imu", 8192, NULL, 2, NULL, 1);
}

void loop() {
  uint32_t now = millis();

  if (txDone) {
    txDone = false;
    if (transmitting) { radio.finishTransmit(); transmitting = false; }
  }
  if (transmitting && (now - txStartMs) > TX_WATCHDOG_MS) {
    radio.finishTransmit(); transmitting = false;
  }

  if (!transmitting && (now - lastTxMs) >= TX_PERIOD_MS) {
    float r, p; int pot; bool sw;
    portENTER_CRITICAL(&ctlMux);
    r = shRollF; p = shPitchF; pot = shPot; sw = shArmSw;
    portEXIT_CRITICAL(&ctlMux);

    // Arm latch: throttle must be at minimum when the switch goes on. Raising
    // the throttle later keeps the armed state; turning the switch off clears
    // the latch and minimum throttle is required again.
#if ARM_REQUIRES_LOW_THROTTLE
    if (!sw)                                       armLatch = false;
    else if (!armLatch && pot <= POT_ARM_MAX_RAW)  armLatch = true;
#else
    armLatch = sw;
#endif
    bool armed = sw && armLatch;

    uint16_t c4[4];
    c4[0] = tiltToCrsf(r, INVERT_ROLL);                     // ch1 roll
    c4[1] = tiltToCrsf(p, INVERT_PITCH);                    // ch2 pitch
    c4[2] = armed ? potToCrsfThrottle(pot) : CRSF_US1000;   // ch3 throttle
    c4[3] = CRSF_MID;                                       // ch4 yaw, neutral

    packAir(air, c4, armed ? AIR_FLAG_ARM : 0x00);
    air[6] = seq++;
    air[7] = crc8_buf(air, AIR_LEN - 1);

    if (radio.startTransmit(air, AIR_LEN) == RADIOLIB_ERR_NONE) {
      transmitting = true; txStartMs = now; lastTxMs = now; nSent++;
    } else nSkipped++;
  }

  // Diagnostics at 2 Hz on the serial port
  if (now - lastLogMs >= 500) {
    lastLogMs = now;
    float r, p; int pot; bool sw; uint32_t hz;
    portENTER_CRITICAL(&ctlMux);
    r = shRollF; p = shPitchF; pot = shPot; sw = shArmSw; hz = shImuHz;
    portEXIT_CRITICAL(&ctlMux);
    Serial.print(F("sw=")); Serial.print(sw);
    Serial.print(F(" latch=")); Serial.print(armLatch);
    Serial.print(F(" roll=")); Serial.print(r, 1);
    Serial.print(F(" pitch=")); Serial.print(p, 1);
    Serial.print(F(" pot=")); Serial.print(pot);
    Serial.print(F(" ch1=")); Serial.print(tiltToCrsf(r, INVERT_ROLL));
    Serial.print(F(" ch2=")); Serial.print(tiltToCrsf(p, INVERT_PITCH));
    Serial.print(F(" ch3=")); Serial.print((sw && armLatch) ? potToCrsfThrottle(pot) : CRSF_US1000);
    Serial.print(F(" sent=")); Serial.print(nSent);
    Serial.print(F(" skip=")); Serial.print(nSkipped);
    Serial.print(F(" imuHz=")); Serial.println(hz);
  }
}
