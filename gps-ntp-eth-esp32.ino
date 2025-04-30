/*
 * GPS NTP Server for ESP32 with Fallback to internal clock
 * Uses hardware UART to read GPS, parse time, and serve NTP
 * When no GPS fix, serves estimated time since last fix (microCRTs) and sets LI=3
 */

#include <Wire.h>
#include <time.h>
#include <string.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Ticker.h>
#include <SPI.h>
#include "minmea.h"
#include "DateTime.h"
#include "arduino_secrets.h"  // contains ssid & pass if STA mode

// ======== Configuration ========
#define DEBUG

// Wi-Fi/Ethernet mode: choose one
#define MODE_STA
// #define MODE_AP
// #define MODE_ETH

// AP mode settings
#ifdef MODE_AP
static const char AP_SSID[] = "NTP";
static const char AP_PASS[] = "NTP";
#define AP_CHANNEL 9
#define AP_HIDDEN_SSID false
#define AP_MAX_CONN 3
#endif

// Ethernet settings
#ifdef MODE_ETH
#include <ETH.h>
#define ETH_ADDR 1
#define ETH_POWER_PIN 16
#define ETH_MDC_PIN 23
#define ETH_MDIO_PIN 18
#define ETH_TYPE ETH_PHY_LAN8720
#endif

// GPS / UART
#define GPS_BAUDRATE 9600
#define GPS_RX_PIN 18
#define GPS_TX_PIN 16
#define GPS_PPS_PIN 17
#define TIMING_OFFSET_US 1u

// NTP
#define NTP_PORT 123
#define NTP_PACKET_SIZE 48

// ======== Globals ========
WiFiUDP Udp;
Ticker displayTimer;
String ipAddr;
uint8_t packetBuffer[NTP_PACKET_SIZE];

// Status flags
union SystemStatus {
  uint8_t all;
  struct {
    uint8_t noSatellite : 1;
    uint8_t noPPS : 1;
    uint8_t newIP : 1;
    uint8_t reserved : 5;
  } bits;
} sysStatus;

// Fallback indicator
bool usingFallback = false;

// GPS parsing
String gpsBuffer;
char gpsSentence[MINMEA_MAX_LENGTH];
struct minmea_sentence_rmc rmcFrame;
struct minmea_sentence_gga ggaFrame;

// Timekeeping
DateTime referenceTime;
unsigned long lastPpsMicros = 0;
unsigned long microOffset = TIMING_OFFSET_US;

// Display update flag
volatile bool displayUpdateFlag = false;

// NTP timestamps
DateTime rxTime, txTime, gpsTime;
byte clientOrigTs[8];

// Forward declarations
void initNetworking();
void initGPS();
void initDisplayTimer();
void handleGPS();
void handleDisplayUpdate();
void handleNTP();
void parseGpsSentence();
DateTime getCurrentTime();
uint64_t toNtp64(const DateTime& dt);
void sendNtpReply(const IPAddress& remoteIP, uint16_t port);
void updateIp();
void setDisplayFlag();

// ======== Setup ========
void setup() {
  Serial.begin(115200);
  randomSeed(analogRead(A0));
  initNetworking();
  initGPS();
  Udp.begin(NTP_PORT);
  initDisplayTimer();
#ifdef DEBUG
  Serial.println("Setup complete.");
  Serial.print("IP: ");
  Serial.println(ipAddr);
#endif
}

// ======== Main Loop ========
void loop() {
  handleGPS();
  if (sysStatus.bits.newIP)
    updateIp();
  handleDisplayUpdate();
  handleNTP();
}

// ======== Initialization ========

void initNetworking() {
#ifdef MODE_STA
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  ipAddr = WiFi.localIP().toString();
#endif

#ifdef MODE_AP
  WiFi.softAP(AP_SSID, AP_PASS, AP_CHANNEL, AP_HIDDEN_SSID, AP_MAX_CONN);
  ipAddr = WiFi.softAPIP().toString();
#endif

#ifdef MODE_ETH
  pinMode(ETH_POWER_PIN, OUTPUT);
  digitalWrite(ETH_POWER_PIN, HIGH);
  ETH.begin(ETH_ADDR, ETH_POWER_PIN, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_TYPE);
  ipAddr = ETH.localIP().toString();
#endif
}

void initGPS() {
  gpsBuffer.reserve(128);
  Serial2.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
#ifdef GPS_PPS_PIN
  pinMode(GPS_PPS_PIN, INPUT_PULLUP);
  attachInterrupt(
    digitalPinToInterrupt(GPS_PPS_PIN), []() {
      lastPpsMicros = micros();
      referenceTime = DateTime(gpsTime.ntptime() + 1, microOffset);
    },
    FALLING);
#endif
}

void initDisplayTimer() {
  displayTimer.attach_ms(900, setDisplayFlag);
}

// ======== Handlers ========

void handleGPS() {
  if (!Serial2.available()) return;
  gpsBuffer = Serial2.readStringUntil('\n');
  gpsBuffer.toCharArray(gpsSentence, gpsBuffer.length() + 1);
  parseGpsSentence();
  gpsBuffer.clear();
}

void handleDisplayUpdate() {
  if (!displayUpdateFlag) return;
  // TODO: update TFT or Serial output
  displayUpdateFlag = false;
}

void handleNTP() {
  int packetSize = Udp.parsePacket();
  if (packetSize == 0) return;

  // Decide fallback or normal
  if (sysStatus.bits.noSatellite || sysStatus.bits.noPPS) {
    usingFallback = true;
#ifdef DEBUG
    Serial.println("Fallback mode: serving internal clock estimate");
#endif
  } else {
    usingFallback = false;
  }

  // Read client request
  Udp.read(packetBuffer, packetSize);
  memcpy(clientOrigTs, packetBuffer + 40, 8);

  IPAddress remoteIP = Udp.remoteIP();
  uint16_t port = Udp.remotePort();

  rxTime = getCurrentTime();
  sendNtpReply(remoteIP, port);
}

// ======== GPS Parsing ========

void parseGpsSentence() {
  switch (minmea_sentence_id(gpsSentence, true)) {
    case MINMEA_SENTENCE_RMC:
      if (minmea_parse_rmc(&rmcFrame, gpsSentence)) {
        sysStatus.bits.noSatellite = false;
        if (rmcFrame.date.year < 0) {
          sysStatus.bits.noSatellite = true;
        } else {
          gpsTime = DateTime(rmcFrame.date.year,
                             rmcFrame.date.month,
                             rmcFrame.date.day,
                             rmcFrame.time.hours,
                             rmcFrame.time.minutes,
                             rmcFrame.time.seconds,
                             microOffset);
        }
#ifndef GPS_PPS_PIN
        lastPpsMicros = micros();
        referenceTime = DateTime(gpsTime.ntptime(), microOffset);
#endif
      }
      break;

    case MINMEA_SENTENCE_GGA:
      if (minmea_parse_gga(&ggaFrame, gpsSentence)) {
        // satellites tracked (could set debug counts)
      }
      break;

    default:
      break;
  }
}

// ======== Time Utilities ========

DateTime getCurrentTime() {
  unsigned long nowMicros = micros();
  unsigned long deltaUs = (nowMicros - lastPpsMicros) + microOffset;
  // Only declare PPS lost if really long gap (e.g., >5s)
  sysStatus.bits.noPPS = (deltaUs > 5000000);
  return DateTime(referenceTime.ntptime(), deltaUs);
}

uint64_t toNtp64(const DateTime& dt) {
  uint64_t hi = (uint64_t)dt.ntptime() << 32;
  uint64_t lo = (uint64_t)(dt.microsfraction() * 4294.967296);
  return hi | lo;
}

void sendNtpReply(const IPAddress& remoteIP, uint16_t port) {
  // NTP header configuration
  // LI (2 bits): 11 = unsynchronized (fallback), 00 = synchronized (GPS)
  // Version (3 bits): NTP v4 (100)
  // Mode (3 bits): Server mode (100)
  packetBuffer[0] = usingFallback ? 0xDC : 0x1C;
  packetBuffer[1] = 1;     // Stratum: 1 = primary server (when using GPS)
  packetBuffer[2] = 2;     // Poll interval
  packetBuffer[3] = 0xF6;  // Precision (~15 us)

  // Root delay and dispersion (unused in basic implementation)
  memset(packetBuffer + 4, 0, 8);

  // Reference Identifier: ASCII "GPS" for GPS source
  packetBuffer[12] = 'G';
  packetBuffer[13] = 'P';
  packetBuffer[14] = 'S';
  packetBuffer[15] = 0;

  /* === Critical NTP Timestamps === */

  // Reference Timestamp (Offset 16-23)
  // Time when server's clock was last set/corrected
  // Either GPS time (PPS sync) or fallback internal clock
  uint64_t refTs = toNtp64(getCurrentTime());
  for (int i = 0; i < 8; ++i)
    packetBuffer[16 + i] = (refTs >> (56 - 8 * i)) & 0xFF;

  // Originate Timestamp (Offset 24-31)
  // Client's request transmission time (copied from client packet)
  memcpy(packetBuffer + 24, clientOrigTs, 8);

  // Receive Timestamp (Offset 32-39)
  // Server's time when request was received
  uint64_t rxTs = toNtp64(rxTime);
  for (int i = 0; i < 8; ++i)
    packetBuffer[32 + i] = (rxTs >> (56 - 8 * i)) & 0xFF;

  // Transmit Timestamp (Offset 40-47)
  // Server's time when response is sent
  txTime = getCurrentTime();
  uint64_t txTs = toNtp64(txTime);
  for (int i = 0; i < 8; ++i)
    packetBuffer[40 + i] = (txTs >> (56 - 8 * i)) & 0xFF;

  // Send completed NTP packet
  Udp.beginPacket(remoteIP, port);
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}


// ======== Helpers ========

void updateIp() {
#ifdef MODE_STA
  ipAddr = WiFi.localIP().toString();
#elif defined(MODE_AP)
  ipAddr = WiFi.softAPIP().toString();
#elif defined(MODE_ETH)
  ipAddr = ETH.localIP().toString();
#endif
  sysStatus.bits.newIP = 0;
#ifdef DEBUG
  Serial.print("New IP: ");
  Serial.println(ipAddr);
#endif
}

void setDisplayFlag() {
  displayUpdateFlag = true;
}
