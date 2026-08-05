// 01_air_pico_lora_rc_nrf24_mavlink.ino  —  AIR node, RP2040-Zero
//
//   CORE 0 : SX1276 LoRa 869.5 MHz (SPI0) -> CRSF -> FC SERIAL1
//   CORE 1 : nRF24L01+ 2.4 GHz (SPI1) <-> MAVLink <-> FC SERIAL5 (Serial2)
//
// Radio layer: 2 Mbps + ACK payload (Enhanced ShockBurst). Downlink and uplink
// travel in the same transaction, so there is no mode switch and no turnaround
// delay. Transaction ~0.6 ms -> ~51 kB/s per direction vs. 5760 B/s load.
// ACK payload requires auto-ack + dynamic payloads, and ARD >= 500 us.
//
// Pinout and sizing: docs/hardware/air-rp2040-zero.md
// Libraries: RadioLib, RF24 (TMRh20)      FQBN: rp2040:rp2040:rpipico
// Never transmit without an antenna.

#include <RadioLib.h>
#include <SPI.h>
#include <RF24.h>

// ---- user configuration ----------------------------------------------------
// LoRa control link (must match the handheld transmitter)
#define LORA_FREQ        869.5
#define LORA_BW          500.0
#define LORA_SF          7
#define LORA_CR          5
#define LORA_SYNCWORD    0x3C
#define LORA_PREAMBLE    8
#define LORA_POWER       17
#define LORA_CURRENT_MA  140
#define AIR_STALE_MS     750UL

// nRF24 MAVLink link (every field must match the ground station)
#define NRF_CHANNEL      76        // 2476 MHz; 2 Mbps occupies 2 MHz
#define NRF_DATARATE     RF24_2MBPS
#define NRF_PA_LEVEL     RF24_PA_MAX
#define NRF_ARD          1         // (1+1)*250 = 500 us, datasheet minimum for ACK payload
#define NRF_ARC          15
#define NRF_SPI_HZ       4000000   // clone chips fail at 10 MHz
#define NRF_ADDR_DOWN    0xB0E1F0A7D2ULL
#define MAV_BAUD         57600

#define DIAG_ENABLE      1
// ----------------------------------------------------------------------------

// SX1276 (SPI0), core 0
#define LORA_MISO   0
#define LORA_CS     1
#define LORA_SCK    2
#define LORA_MOSI   3
#define LORA_DIO0   14
#define LORA_RST    15

// nRF24 (SPI1), core 1
#define NRF_CE      7
#define NRF_CSN     9
#define NRF_MISO    8
#define NRF_SCK    10
#define NRF_MOSI   11

// CRSF output (UART0), core 0
#define CRSF_UART     Serial1
#define CRSF_TX_PIN   12
#define CRSF_RX_PIN   13
#define CRSF_BAUD     416666

// MAVLink UART (UART1 = Serial2), core 1
#define MAV_UART     Serial2
#define MAV_TX_PIN   4
#define MAV_RX_PIN   5

// over-the-air control frame
#define AIR_LEN         8
#define AIR_FLAG_ARM    0x01

// CRSF protocol
#define CRSF_SYNC_FC              0xC8
#define CRSF_TYPE_RC_CHANNELS     0x16
#define CRSF_TYPE_LINK_STATISTICS 0x14
#define CRSF_US1000   192
#define CRSF_US1500   992
#define CRSF_US2000  1792
#define CRSF_RATE_HZ  50
#define LINKSTAT_MS   200UL

// nRF24 stream
#define NRF_MAX       32
#define DATA_MAX      31           // payload[0] is the length header
#define RB_SIZE       4096
#define RB_MASK       (RB_SIZE - 1)
#define IDLE_POLL_US  1000UL       // poll rate on an empty stream, so the uplink
                                   // ACK payload still gets carried

SX1276 lora = new Module(LORA_CS, LORA_DIO0, LORA_RST);
RF24   nrf(NRF_CE, NRF_CSN, NRF_SPI_HZ);

// ---- CRC8 DVB-S2 -----------------------------------------------------------
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

// ---- diagnostic counters ---------------------------------------------------
volatile uint32_t dAirGood = 0, dAirCrc = 0, dAirLen = 0, dAirLost = 0, dAirIrq = 0;
volatile int32_t  dRssi = -127;
volatile uint8_t  dLq = 0;
volatile bool     dLoraOk = false, dNrfOk = false;
volatile uint32_t dTxOk = 0, dTxFail = 0, dAckPkt = 0;
volatile uint32_t dTxBytes = 0, dRxBytes = 0, dBadHdr = 0;
volatile uint32_t dRbCount = 0, dRbDrop = 0, dArc = 0;

// ############################################################################
//  CORE 0 — LoRa control reception -> CRSF
// ############################################################################
volatile bool loraRxFlag = false;
uint8_t  airbuf[AIR_LEN];
uint16_t ch[16];

uint32_t lastAirMs = 0, lastCrsfMs = 0, lastStatMs = 0, lqWinStart = 0;
bool     armFlag = false;
uint8_t  lastSeq = 0; bool seqInit = false;
uint32_t lqWinGood = 0;
float    loraRssi = -120, loraSnr = 0;

void loraFlag(void) { loraRxFlag = true; dAirIrq++; }

static void unpackAir(const uint8_t *p, uint16_t *out4, uint8_t *flags) {
  uint32_t acc = 0; uint8_t bits = 0, idx = 0;
  for (uint8_t i = 0; i < 4; i++) {
    while (bits < 11) { acc |= ((uint32_t)p[idx++]) << bits; bits += 8; }
    out4[i] = (uint16_t)(acc & 0x7FF);
    acc >>= 11; bits -= 11;
  }
  *flags = (uint8_t)(acc & 0x0F);
}

void crsfSendChannels() {
  uint8_t f[26];
  f[0] = CRSF_SYNC_FC; f[1] = 24; f[2] = CRSF_TYPE_RC_CHANNELS;
  uint8_t *p = &f[3];
  memset(p, 0, 22);
  uint32_t acc = 0; uint8_t bits = 0, o = 0;
  for (uint8_t i = 0; i < 16; i++) {
    acc |= ((uint32_t)(ch[i] & 0x7FF)) << bits;
    bits += 11;
    while (bits >= 8) { p[o++] = (uint8_t)(acc & 0xFF); acc >>= 8; bits -= 8; }
  }
  if (bits) p[o++] = (uint8_t)(acc & 0xFF);
  f[25] = crc8_buf(&f[2], 23);
  CRSF_UART.write(f, 26);
}

void crsfSendLinkStats() {
  uint8_t f[14];
  f[0] = CRSF_SYNC_FC; f[1] = 12; f[2] = CRSF_TYPE_LINK_STATISTICS;
  int r = (int)(-loraRssi);
  if (r < 0) r = 0; if (r > 255) r = 255;
  f[3] = (uint8_t)r; f[4] = 0; f[5] = dLq; f[6] = (uint8_t)(int8_t)loraSnr;
  f[7] = 0; f[8] = 0; f[9] = 2; f[10] = 0; f[11] = 0; f[12] = 0;
  f[13] = crc8_buf(&f[2], 11);
  CRSF_UART.write(f, 14);
}

void setup() {
  for (uint8_t i = 0; i < 16; i++) ch[i] = CRSF_US1500;
  ch[2] = CRSF_US1000;
  ch[4] = CRSF_US1000;
  ch[5] = CRSF_US1000;

#if DIAG_ENABLE
  Serial.begin(115200);
  Serial.ignoreFlowControl(true);
#endif

  CRSF_UART.setTX(CRSF_TX_PIN);
  CRSF_UART.setRX(CRSF_RX_PIN);
  CRSF_UART.begin(CRSF_BAUD);

  SPI.setSCK(LORA_SCK);
  SPI.setTX(LORA_MOSI);
  SPI.setRX(LORA_MISO);
  SPI.begin();

  int st = lora.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR,
                      LORA_SYNCWORD, LORA_POWER, LORA_PREAMBLE);
  dLoraOk = (st == RADIOLIB_ERR_NONE);

  if (dLoraOk) {
    lora.setFrequency(LORA_FREQ);      // BW500 errata: must be re-applied after begin()
    lora.setCurrentLimit(LORA_CURRENT_MA);
    lora.explicitHeader();
    lora.setCRC(true);
    lora.setDio0Action(loraFlag, RISING);
    lora.startReceive();
  }
  lqWinStart = millis();
}

void loop() {
  uint32_t now = millis();

  if (loraRxFlag) {
    loraRxFlag = false;
    int len = lora.getPacketLength();
    if (len == AIR_LEN) {
      int rst = lora.readData(airbuf, AIR_LEN);
      if (rst == RADIOLIB_ERR_NONE) {
        if (crc8_buf(airbuf, AIR_LEN - 1) != airbuf[AIR_LEN - 1]) {
          dAirCrc++;
        } else {
          uint8_t sq = airbuf[6];
          if (seqInit) { uint8_t d = (uint8_t)(sq - lastSeq); if (d > 1) dAirLost += (d - 1); }
          lastSeq = sq; seqInit = true;

          uint16_t c4[4]; uint8_t flags;
          unpackAir(airbuf, c4, &flags);
          ch[0] = c4[0]; ch[1] = c4[1]; ch[2] = c4[2]; ch[3] = c4[3];
          armFlag = (flags & AIR_FLAG_ARM) != 0;

          lastAirMs = now;
          loraRssi = lora.getRSSI();
          loraSnr  = lora.getSNR();
          dRssi = (int32_t)loraRssi;
          dAirGood++; lqWinGood++;
        }
      } else {
        dAirCrc++;
      }
    } else if (len > 0) {
      lora.readData(airbuf, (len > AIR_LEN) ? AIR_LEN : len);
      dAirLen++;
    }
    lora.startReceive();
  }

  if (now - lqWinStart >= 2000UL) {
    uint32_t expect = (2000UL * 3UL) / AIR_STALE_MS;
    uint32_t v = (lqWinGood * 100UL) / (expect ? expect : 1);
    dLq = (v > 100) ? 100 : (uint8_t)v;
    lqWinGood = 0; lqWinStart = now;
  }

  bool fresh = (lastAirMs != 0) && (now - lastAirMs < AIR_STALE_MS);
  if (fresh) {
    ch[4] = CRSF_US1000;
    ch[5] = armFlag ? CRSF_US2000 : CRSF_US1000;
    for (uint8_t i = 6; i < 16; i++) ch[i] = CRSF_US1500;
    if (now - lastCrsfMs >= (1000UL / CRSF_RATE_HZ)) { crsfSendChannels(); lastCrsfMs = now; }
    if (now - lastStatMs >= LINKSTAT_MS)             { crsfSendLinkStats(); lastStatMs = now; }
  } else {
    ch[2] = CRSF_US1000;
    ch[5] = CRSF_US1000;
    armFlag = false;
  }

#if DIAG_ENABLE
  static uint32_t lastDiag = 0;
  if (now - lastDiag >= 500) {
    lastDiag = now;
    Serial.printf("[LORA] init=%d irq=%lu good=%lu crcBad=%lu badLen=%lu lost=%lu lq=%u rssi=%ld fresh=%d arm=%d ch1=%u ch3=%u\n",
      (int)dLoraOk, dAirIrq, dAirGood, dAirCrc, dAirLen, dAirLost,
      (unsigned)dLq, (long)dRssi, (int)fresh, (int)armFlag, ch[0], ch[2]);
    Serial.printf("[NRF]  init=%d txOk=%lu txFail=%lu ackPkt=%lu txB=%lu rxB=%lu badHdr=%lu arc=%lu q=%lu drop=%lu\n",
      (int)dNrfOk, dTxOk, dTxFail, dAckPkt, dTxBytes, dRxBytes,
      dBadHdr, dArc, dRbCount, dRbDrop);
  }
#endif
}

// ############################################################################
//  CORE 1 — MAVLink (Serial2) <-> nRF24, PTX role
//  One transaction: peek 31 B -> write() -> hardware ACK -> skip.
//  The radio stays in PTX permanently; the uplink rides in the ACK payload.
// ############################################################################
static uint8_t  rb[RB_SIZE];
static uint16_t rbHead = 0, rbTail = 0, rbCount = 0;

static inline void rbPush(uint8_t b) {
  if (rbCount == RB_SIZE) { rbTail = (rbTail + 1) & RB_MASK; rbCount--; dRbDrop++; }
  rb[rbHead] = b; rbHead = (rbHead + 1) & RB_MASK; rbCount++;
}
static inline size_t rbPeek(uint8_t *out, size_t maxLen) {
  size_t n = (rbCount < maxLen) ? rbCount : maxLen;
  uint16_t t = rbTail;
  for (size_t i = 0; i < n; i++) { out[i] = rb[t]; t = (t + 1) & RB_MASK; }
  return n;
}
static inline void rbSkip(size_t n) {
  rbTail = (rbTail + (uint16_t)n) & RB_MASK;
  rbCount -= (uint16_t)n;
}

uint8_t  txbuf[NRF_MAX], ackbuf[NRF_MAX];
uint32_t lastTxUs = 0;

void setup1() {
  MAV_UART.setTX(MAV_TX_PIN);
  MAV_UART.setRX(MAV_RX_PIN);
  MAV_UART.begin(MAV_BAUD);

  SPI1.setSCK(NRF_SCK);
  SPI1.setTX(NRF_MOSI);
  SPI1.setRX(NRF_MISO);
  SPI1.begin();

  // A failure here must not halt the board: LoRa/CRSF has to keep running.
  if (nrf.begin(&SPI1) && nrf.isChipConnected()) {
    nrf.setPALevel(NRF_PA_LEVEL);
    nrf.setDataRate(NRF_DATARATE);
    nrf.setChannel(NRF_CHANNEL);
    nrf.setCRCLength(RF24_CRC_16);
    nrf.setAddressWidth(5);
    nrf.setAutoAck(true);                 // required for ACK payload
    nrf.enableDynamicPayloads();          // required for ACK payload
    nrf.enableAckPayload();               // this is what makes the link bidirectional
    nrf.setRetries(NRF_ARD, NRF_ARC);
    nrf.openWritingPipe(NRF_ADDR_DOWN);
    nrf.stopListening();                  // PTX at all times
    nrf.flush_tx();
    nrf.flush_rx();
    dNrfOk = true;
  } else {
    dNrfOk = false;
  }
}

void loop1() {
  if (!dNrfOk) { delay(100); return; }

  // 1. FC UART -> ring buffer
  while (MAV_UART.available()) rbPush((uint8_t)MAV_UART.read());
  dRbCount = rbCount;

  // 2. Start a transaction: immediately if there is data, otherwise poll every
  //    IDLE_POLL_US so the uplink still has a carrier to ride on.
  uint32_t nowUs = micros();
  if (rbCount == 0 && (nowUs - lastTxUs) < IDLE_POLL_US) return;
  lastTxUs = nowUs;

  uint8_t n = (uint8_t)rbPeek(&txbuf[1], DATA_MAX);
  txbuf[0] = n;                            // header 0 = empty poll

  bool ok = nrf.write(txbuf, (uint8_t)(n + 1));
  if (ok) {
    rbSkip(n);                             // discard only after a hardware ACK
    dTxOk++; dTxBytes += n;
  } else {
    dTxFail++;                             // data stays in the buffer
    nrf.flush_tx();
  }
  dArc = nrf.getARC();                     // retry count, used as a range proxy

  // 3. Uplink received in the ACK -> out to the FC
  while (nrf.available()) {
    uint8_t len = nrf.getDynamicPayloadSize();
    if (len < 1 || len > NRF_MAX) { nrf.flush_rx(); break; }
    nrf.read(ackbuf, len);
    dAckPkt++;
    uint8_t h = ackbuf[0];
    if (h == 0) {
      // empty ACK: no uplink data
    } else if (h == (uint8_t)(len - 1)) {  // header validation
      MAV_UART.write(&ackbuf[1], h);
      dRxBytes += h;
    } else {
      dBadHdr++;
    }
  }
}
