// ===== File: GpsInflux.h =====
#ifndef GPS_INFLUX_H
#define GPS_INFLUX_H

#include <TinyGPSPlus.h>
#include <HTTPClient.h>

// InfluxDB settings (from arduino_secrets.h)
#include "influxdb_secrets.h"

// Post interval in milliseconds
extern const unsigned long POST_INTERVAL;

// GPS pins & baud rate
extern const int GPS_RX_PIN;
extern const int GPS_TX_PIN;
extern const uint32_t GPS_BAUD;

void gpsInfluxFeed(String c);


// Call regularly (e.g., in loop) to handle timed POSTs
void gpsInfluxUpdate();

#endif // GPS_INFLUX_H

