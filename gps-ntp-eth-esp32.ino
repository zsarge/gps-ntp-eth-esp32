/*
 * GPS NTP Server for ESP32
 * Uses hardware UART to read GPS, parse time, and serve NTP
 * Only responds when valid GPS fix is available
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
#include "arduino_secrets.h"
#include "GpsInflux.h"

// ======== Configuration ========
#define DEBUG

// Network mode
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

// GPS Configuration
#define GPS_BAUDRATE 9600
#define GPS_RX_PIN 18
#define GPS_TX_PIN 16
#define GPS_PPS_PIN 17
#define TIMING_OFFSET_US 1u

// NTP Server
#define NTP_PORT 123
#define NTP_PACKET_SIZE 48

// ======== Globals ========
WiFiUDP Udp;
Ticker displayTimer;
String ipAddr;
uint8_t packetBuffer[NTP_PACKET_SIZE];

// System Status
union SystemStatus {
  uint8_t all;
  struct {
    uint8_t newIP : 1;
    uint8_t reserved : 7;
  } bits;
} sysStatus;

bool displayUpdateFlag;

// GPS Data
String gpsBuffer;
char gpsSentence[MINMEA_MAX_LENGTH];
struct minmea_sentence_rmc rmcFrame;
struct minmea_sentence_gga ggaFrame;

// Timekeeping
DateTime referenceTime;
unsigned long lastPpsMicros = 0;
unsigned long microOffset = TIMING_OFFSET_US;
DateTime gpsTime;  // Valid only when GPS fix exists

// NTP Transaction
DateTime rxTime, txTime;
byte clientOrigTs[8];

// Forward Declarations
void initNetworking();
void initGPS();
void initDisplayTimer();
void handleGPS(String gpsData);
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
  String data = Serial2.readStringUntil('\n');
  handleGPS(data);
  gpsInfluxFeed(data);
  gpsInfluxUpdate();

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
      if (gpsTime.isValid()) {
        referenceTime = DateTime(gpsTime.ntptime() + 1, microOffset);
      }
    },
    FALLING);
#endif
}

void initDisplayTimer() {
  displayTimer.attach_ms(900, setDisplayFlag);
}

// ======== Handlers ========
void handleGPS(String gpsData) {
  gpsBuffer = gpsData;
  gpsBuffer.toCharArray(gpsSentence, gpsBuffer.length() + 1);
  parseGpsSentence();
  gpsBuffer.clear();
}

void handleDisplayUpdate() {
  if (!displayUpdateFlag) return;
  displayUpdateFlag = false;
}

void handleNTP() {
  int packetSize = Udp.parsePacket();
  if (packetSize == 0 || !gpsTime.isValid()) {
#ifdef DEBUG
    if (packetSize > 0) Serial.println("Ignoring request - no GPS fix");
#endif
    return;
  }

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
        if (rmcFrame.valid && rmcFrame.date.year >= 2023) {
          gpsTime = DateTime(rmcFrame.date.year,
                             rmcFrame.date.month,
                             rmcFrame.date.day,
                             rmcFrame.time.hours,
                             rmcFrame.time.minutes,
                             rmcFrame.time.seconds,
                             microOffset);
        }
      }
      break;

    case MINMEA_SENTENCE_GGA:
      minmea_parse_gga(&ggaFrame, gpsSentence);
      break;

    default:
      break;
  }
}

// ======== Time Utilities ========
DateTime getCurrentTime() {
  unsigned long nowMicros = micros();
  unsigned long deltaUs = (nowMicros - lastPpsMicros) + microOffset;
  return DateTime(gpsTime.ntptime(), deltaUs);
}

uint64_t toNtp64(const DateTime& dt) {
  uint64_t hi = (uint64_t)dt.ntptime() << 32;
  uint64_t lo = (uint64_t)(dt.microsfraction() * 4294.967296);
  return hi | lo;
}

void sendNtpReply(const IPAddress& remoteIP, uint16_t port) {
  // NTP Header
  packetBuffer[0] = 0x1C;  // LI=00, Version=4, Mode=4
  packetBuffer[1] = 1;     // Stratum 1
  packetBuffer[2] = 2;     // Poll interval
  packetBuffer[3] = 0xF6;  // Precision

  memset(packetBuffer + 4, 0, 8);  // Root delay/dispersion

  // Reference ID
  packetBuffer[12] = 'G';
  packetBuffer[13] = 'P';
  packetBuffer[14] = 'S';
  packetBuffer[15] = 0;

  // Reference Timestamp (GPS time)
  uint64_t refTs = toNtp64(gpsTime);
  for (int i = 0; i < 8; ++i)
    packetBuffer[16 + i] = (refTs >> (56 - 8 * i)) & 0xFF;

  // Originate Timestamp
  memcpy(packetBuffer + 24, clientOrigTs, 8);

  // Receive Timestamp
  uint64_t rxTs = toNtp64(rxTime);
  for (int i = 0; i < 8; ++i)
    packetBuffer[32 + i] = (rxTs >> (56 - 8 * i)) & 0xFF;

  // Transmit Timestamp
  txTime = getCurrentTime();
  uint64_t txTs = toNtp64(txTime);
  for (int i = 0; i < 8; ++i)
    packetBuffer[40 + i] = (txTs >> (56 - 8 * i)) & 0xFF;

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