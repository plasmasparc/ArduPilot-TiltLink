// 02_ground_d32_nrf24_mavlink.ino  —  GROUND MAVLink station, LOLIN D32 + nRF24L01+
//
// nRF24 (2.4 GHz) <-> { USB serial , WiFi UDP }, both at the same time.
// Peer: 01_air_pico_lora_rc_nrf24_mavlink
//
// This side is the PRX and never changes mode. The uplink travels back inside
// the ACK payload of the same transaction that carries the telemetry down, so
// there is no turnaround delay. The ACK payload must already be in the TX FIFO
// BEFORE the packet arrives, hence the continuously topped-up queue.
//
// GCS:  mavproxy.py --master=udp:0.0.0.0:14550 --console
// Diagnostics: nc -ulk 14551.   Pinout: docs/hardware/ground-lolin-d32.md
// Libraries: RF24 (TMRh20) v1.4+, WiFi (core)   FQBN: esp32:esp32:lolin_d32
// Never transmit without an antenna.

#include <SPI.h>
#include <RF24.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// ---- user configuration ----------------------------------------------------
#define WIFI_SSID   "Change me - ssid"
#define WIFI_PASS   "Change me - pass"
#define UDP_PORT    14550

// nRF24: every field must match the air node
#define NRF_CHANNEL   76
#define NRF_DATARATE  RF24_2MBPS
#define NRF_PA_LEVEL  RF24_PA_MAX
#define NRF_ARD       1                 // (1+1)*250 = 500 us
#define NRF_ARC       15
#define NRF_SPI_HZ    4000000
#define NRF_ADDR_DOWN 0xB0E1F0A7D2ULL

// WiFi and the nRF24 share the 2.4 GHz band on the same board; the WiFi PA can
// desensitise the nRF24 receiver at close range. To test, set this to 0: if the
// link improves abruptly, WiFi is the cause.
#define USE_WIFI       1
// ----------------------------------------------------------------------------

// nRF24 pinout (VSPI)
#define NRF_SCK    18
#define NRF_MISO   19
#define NRF_MOSI   23
#define NRF_CSN     5
#define NRF_CE     17

#define GCS_BAUD    115200
#define NRF_MAX     32
#define DATA_MAX    31                  // payload[0] is the length header
#define RB_SIZE     4096
#define RB_MASK     (RB_SIZE - 1)

// UDP aggregation
#define UDP_AGG_SIZE    512
#define UDP_AGG_US      20000UL
#define UDP_IN_MAX      512
#define WIFI_RETRY_MS   30000UL
#define PEER_TIMEOUT_MS 10000UL

RF24     nrf(NRF_CE, NRF_CSN, NRF_SPI_HZ);
WiFiUDP  udp;

// ---- ring buffer (GCS{USB,UDP} -> RF), drop-oldest --------------------------
static uint8_t  rb[RB_SIZE];
static uint16_t rbHead = 0, rbTail = 0, rbCount = 0;
static uint32_t rbDropped = 0;

static inline void rbPush(uint8_t b) {
  if (rbCount == RB_SIZE) { rbTail = (rbTail + 1) & RB_MASK; rbCount--; rbDropped++; }
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

// ---- UDP state -------------------------------------------------------------
uint8_t  udpAgg[UDP_AGG_SIZE]; size_t udpAggLen = 0; uint32_t udpAggFirstUs = 0;
uint8_t  udpIn[UDP_IN_MAX];
bool      peerKnown = false; IPAddress peerIp; uint16_t peerPort = 0;
uint32_t  lastPeerMs = 0;
bool      wifiUp = false; uint32_t lastWifiTryMs = 0;

// ---- statistics ------------------------------------------------------------
uint8_t  rxbuf[NRF_MAX], ackbuf[NRF_MAX];
uint32_t nRxPkt = 0, rxBytes = 0, nAckLoaded = 0, ackBytes = 0;
uint32_t nBadHdr = 0, udpOutDg = 0, udpInBytes = 0, lastRxMs = 0, lastLogMs = 0;

// ============================================================================
//  UDP
// ============================================================================
void udpFlush() {
  if (udpAggLen == 0) return;
  if (!wifiUp) { udpAggLen = 0; return; }
  IPAddress dst; uint16_t dport;
  if (peerKnown && (millis() - lastPeerMs) < PEER_TIMEOUT_MS) { dst = peerIp; dport = peerPort; }
  else { peerKnown = false; dst = WiFi.broadcastIP(); dport = UDP_PORT; }
  if (udp.beginPacket(dst, dport)) {
    udp.write(udpAgg, udpAggLen);
    if (udp.endPacket()) udpOutDg++;
  }
  udpAggLen = 0;
}
static inline void udpQueue(const uint8_t *data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    if (udpAggLen == 0) udpAggFirstUs = micros();
    udpAgg[udpAggLen++] = data[i];
    if (udpAggLen >= UDP_AGG_SIZE) udpFlush();
  }
}
void wifiService() {
#if !USE_WIFI
  return;
#else
  if (WiFi.status() == WL_CONNECTED) {
    if (!wifiUp) { wifiUp = true; udp.begin(UDP_PORT); }
    return;
  }
  if (wifiUp) { wifiUp = false; peerKnown = false; }
  if (millis() - lastWifiTryMs >= WIFI_RETRY_MS) {
    lastWifiTryMs = millis(); WiFi.disconnect(); WiFi.begin(WIFI_SSID, WIFI_PASS);
  }
#endif
}

// Topping up the ACK payload queue = uplink towards the air node. Keep filling
// while the 3-deep FIFO accepts; false means full, data stays in the buffer.
void topUpAckPayload() {
  while (rbCount > 0) {
    uint8_t n = (uint8_t)rbPeek(&ackbuf[1], DATA_MAX);
    ackbuf[0] = n;                                     // length header
    if (!nrf.writeAckPayload(1, ackbuf, (uint8_t)(n + 1))) break;
    rbSkip(n);                                         // only after a successful load
    nAckLoaded++; ackBytes += n;
  }
}

void setup() {
  Serial.setRxBufferSize(2048);
  Serial.begin(GCS_BAUD);

#if USE_WIFI
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);              // no modem sleep: lower UDP jitter
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  lastWifiTryMs = millis();
#else
  WiFi.mode(WIFI_OFF);
  btStop();
#endif

  SPI.begin(NRF_SCK, NRF_MISO, NRF_MOSI, -1);   // CS is handled by RF24

  if (!nrf.begin(&SPI) || !nrf.isChipConnected()) {
    while (true) { Serial.println(F("NRF INIT FAIL")); delay(1000); }
  }
  nrf.setPALevel(NRF_PA_LEVEL);
  nrf.setDataRate(NRF_DATARATE);
  nrf.setChannel(NRF_CHANNEL);
  nrf.setCRCLength(RF24_CRC_16);
  nrf.setAddressWidth(5);
  nrf.setAutoAck(true);                // required for ACK payload
  nrf.enableDynamicPayloads();         // required for ACK payload
  nrf.enableAckPayload();              // this is what makes the link bidirectional
  nrf.setRetries(NRF_ARD, NRF_ARC);
  nrf.openReadingPipe(1, NRF_ADDR_DOWN);
  nrf.startListening();                // PRX at all times, no mode switching
  nrf.flush_tx();
  nrf.flush_rx();
}

void loop() {
  uint32_t nowMs = millis();

  wifiService();

  // 1a. USB -> ring buffer (uplink)
  while (Serial.available()) rbPush((uint8_t)Serial.read());

  // 1b. UDP -> ring buffer, learning the peer address
  if (wifiUp) {
    int pkt;
    while ((pkt = udp.parsePacket()) > 0) {
      peerIp = udp.remoteIP(); peerPort = udp.remotePort();
      peerKnown = true; lastPeerMs = nowMs;
      int n = udp.read(udpIn, (pkt > UDP_IN_MAX) ? UDP_IN_MAX : pkt);
      for (int i = 0; i < n; i++) rbPush(udpIn[i]);
      if (n > 0) udpInBytes += n;
    }
  }

  // 2. Keep the ACK queue loaded: the payload must be ready before the packet
  if (rbCount > 0) topUpAckPayload();

  // 3. Downlink reception -> USB + UDP
  uint8_t pipe;
  while (nrf.available(&pipe)) {
    uint8_t len = nrf.getDynamicPayloadSize();
    if (len < 1 || len > NRF_MAX) { nrf.flush_rx(); break; }
    nrf.read(rxbuf, len);
    nRxPkt++; lastRxMs = nowMs;

    uint8_t h = rxbuf[0];
    if (h == 0) {
      // empty poll from the air node: no telemetry, uplink query only
    } else if (h == (uint8_t)(len - 1)) {     // header validation
      Serial.write(&rxbuf[1], h);             // RF -> USB
      udpQueue(&rxbuf[1], h);                 // RF -> UDP (aggregated)
      rxBytes += h;
    } else {
      nBadHdr++;
    }
    topUpAckPayload();                        // refill immediately
  }

  // 4. Flush the UDP aggregate on timeout
  if (udpAggLen > 0 && (micros() - udpAggFirstUs) >= UDP_AGG_US) udpFlush();

  // 5. Diagnostics at 2 Hz on UDP_PORT + 1
#if USE_WIFI
  if (wifiUp && (nowMs - lastLogMs) >= 500) {
    lastLogMs = nowMs;
    char line[220];
    int n = snprintf(line, sizeof(line),
      "link=%s age=%lu rx=%lu p / %lu B  ackLoaded=%lu / %lu B  "
      "badHdr=%lu q=%u drop=%lu udpDg=%lu udpIn=%lu\n",
      (lastRxMs && (nowMs - lastRxMs) < 3000) ? "UP" : "DOWN",
      (unsigned long)(lastRxMs ? (nowMs - lastRxMs) : 999999),
      (unsigned long)nRxPkt, (unsigned long)rxBytes,
      (unsigned long)nAckLoaded, (unsigned long)ackBytes,
      (unsigned long)nBadHdr, (unsigned)rbCount, (unsigned long)rbDropped,
      (unsigned long)udpOutDg, (unsigned long)udpInBytes);
    IPAddress dst = peerKnown ? peerIp : WiFi.broadcastIP();
    if (udp.beginPacket(dst, UDP_PORT + 1)) {
      udp.write((const uint8_t*)line, n);
      udp.endPacket();
    }
  }
#endif
}