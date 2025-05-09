#include "GpsInflux.h"
#include <WiFi.h>

const unsigned long POST_INTERVAL = 15000;
const int GPS_RX_PIN = 18;
const int GPS_TX_PIN = 16;
const uint32_t GPS_BAUD = 9600;

static TinyGPSPlus gps;
static TinyGPSCustom satsInView(gps, "GPGSV", 3);
static unsigned long lastPost = 0;

String writeCache;

void gpsInfluxFeed(String s) {
  for (int i = 0; i < s.length(); i++) {
    gps.encode(s.charAt(i));
  }
  writeCache += (s + "\n");
  // Serial.print("fed string: ");
  // Serial.println(s);
}

void gpsInfluxUpdate() {
  // Only post at intervals and if GPS data updated
  if (millis() - lastPost < POST_INTERVAL) return;
  // if (!gps.location.isUpdated()) return;

  lastPost = millis();
  int sats = gps.satellites.value();
  int visible = atoi(satsInView.value());
  bool fixOk = gps.location.isValid() && gps.date.isValid() && gps.time.isValid();

  String line = String("gps_metrics")
                + " satellites=" + sats
                + ",visible=" + visible
                + ",raw=\"" + writeCache
                + "\",fix=" + (fixOk ? "1" : "0");

  String url = String(INFLUX_URL)
               + "/api/v2/write?org=" + INFLUX_ORG
               + "&bucket=" + INFLUX_BUCKET
               + "&precision=s";

  HTTPClient http;
  http.begin(url);
  http.addHeader("Authorization", String("Token ") + INFLUX_TOKEN);
  http.addHeader("Content-Type", "text/plain; charset=utf-8");

  int code = http.POST(line);
  Serial.print("[Influx] ");
  Serial.println(line);
  if (code > 0) {
    Serial.print("HTTP status: ");
    Serial.println(code);
  } else {
    Serial.print("Post failed: ");
    Serial.println(http.errorToString(code));
  }
  writeCache = "";
  http.end();
}
