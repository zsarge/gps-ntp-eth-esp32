/*
 * This piece of code uses the hardware UART to connect to the GPS, and fetches the time.
 * ntp seconds: Seconds since 01/01/1900!
 */

#include <Wire.h>
#include <time.h>
#include <string.h>

/// ESP-related stuff. You can get these via the board manager
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <Ticker.h>
#include "GpsInflux.h"

// You can get these libraries from the arduino library manager
// #include <TFT_eSPI.h>  // Graphics and font library for ILI9341 driver chip
#include <SPI.h>

// Local stuff.
#include "minmea.h"
#include "DateTime.h"  // this helps with the NTP time stamp calculations.

/*
 * Global settings
 */
// Debug mode
#define DEBUG

// Wi-Fi stuff.
//#define OPERATE_AS_AP
//OR
#define OPERATE_AS_STA
//OR
// #define OPERATE_AS_ETH

#ifdef OPERATE_AS_STA
// Change this info with your network's name and password, if you want to use the client mode.
// const char ssid[] = "SSID";     //SSID of your network
// const char pass[] = "PASSWORD";  //password of your WPA Network
#include "arduino_secrets.h"
#endif

#ifdef OPERATE_AS_AP
const char ssid[] = "NTP";  //SSID of your network
const char pass[] = "NTP";  //password of your WPA Network
#define CHANNEL 9           // Wifi channel. Between 1 and 13, to your taste. 2.4 GHz.
#define HIDE_SSID false     // Don't hide SSID.
#define MAX_CONNECTION 3    // How many clients we should handle simultaneously. between 0 and 8.
// the IP will be 192.168.4.1. It can be cinfigured further, but I don't think it matters.
#endif

#ifdef OPERATE_AS_ETH
#include <ETH.h>
#define ETH_ADDR 1
#define ETH_POWER_PIN 16              // -1             // ??? Do not use it, it can cause conflict during the software reset.
#define ETH_POWER_PIN_ALTERNATIVE 16  // 17    // ???
#define ETH_MDC_PIN 23
#define ETH_MDIO_PIN 18
#define ETH_TYPE ETH_PHY_LAN8720
//#define ETH_CLK_MODE    ETH_CLOCK_GPIO17_OUT // ETH_CLOCK_GPIO0_IN // ???
#endif

#define NTP_PORT 123
#define NTP_PACKET_SIZE 48

String ip = "";  // we use DHCP

/*
 * Hardware pins
 */
// GPS
#define GPS_BAUDRATE 9600
// #define GPS_PPS_PIN 17  // If your GPS doesn't have a PPS output, just comment out this line.
#define GPS_RX_PIN 18  //16
#define GPS_TX_PIN 16
//#define GPS_EN_PIN 12 // If your GPS doesn't have a EN input

/* IMPORTANT:
 * ALWAYS VERIFY TIMING ACCURACY BEFORE USING THIS DEVICE!!!
 *  
 * Since this device doesn't take leap seconds into account, the 'correct' time will be calculated with a simple offset.
 * This offset affects the microsecond counter so fine control can be achieved.
 * NOTE that this only ever can be a positive number!
 * 
 */
#define TIMING_OFFSET_US 0001

// #define DEBUG_PRINTLN Serial.printf("Got to: " __FILE__ ": %d\n", __LINE__)
#define DEBUG_PRINTLN

/*
 * Library-provided high-level objects
 */
// TFT_eSPI tft = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h
Ticker display_timer;  // This controls how often the screen is updated.

WiFiUDP Udp;  // UDP handler.

/*
 * Global variables
 */
struct event_s {
  uint8_t notSatellite : 1;
  uint8_t notPPS : 1;
  uint8_t newIP : 1;
};
union sysStatus_u {
  uint8_t all;
  struct event_s event;
} sysStatus;
uint8_t sysStatusCache = 255;
String chackIPCache;

bool can_respond_to_packets = false;  // Set to true when the data is parsed, set to false just after sending an NTP packet.
char there_is_new_data = 0;           // This is for the UART
String uart_string;                   // A GPS string should not be any longer than this.
unsigned int uart_string_length = 0;  // This tells how long an NMEA sentence is, in bytes.
bool update_the_display = false;      // This is controlled from a timer.

struct minmea_sentence_rmc rmc_frame;  // $GPRMC frame, after minmea parsed it.
struct minmea_sentence_gga gga_frame;  // $GPRMC frame, after minmea parsed it.
char gps_sentence[MINMEA_MAX_LENGTH];  // Character array, initialised as per the minmea lib.

// these are for keeping time.
unsigned long microsecond_counter = 0;  // CPU microseconds
int satellites_tracked = 0;             //
DateTime reference_time;                // This is being updated by the G
DateTime uart_time;                     // this is the DateTime structure decoded by the GPS.
DateTime receive_time;                  // This is set on an incoming NTP request
DateTime transmit_time;                 // This is set when transmitting the NTP packet.
byte origTimeTs[9];                     // The remote host's local time stamp.

byte packetBuffer[NTP_PACKET_SIZE];

void setup() {
  // For randomisation, we need this: the ADC is not connected, so it reads noise,
  randomSeed(analogRead(A0));
  DEBUG_PRINTLN;
// Wifi.
#ifdef OPERATE_AS_AP
  // Stand-alone access point.
  WiFi.disconnect(true);  // This re-initialises the wifi.
  WiFi.softAP(ssid, CHANNEL, HIDE_SSID, MAX_CONNECTION);
#endif
  DEBUG_PRINTLN;

#ifdef OPERATE_AS_STA
  // If client, use these.
  WiFi.disconnect(true);  // This re-initialises the wifi.
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
#endif
  DEBUG_PRINTLN;

#ifdef OPERATE_AS_ETH
  pinMode(ETH_POWER_PIN_ALTERNATIVE, OUTPUT);
  digitalWrite(ETH_POWER_PIN_ALTERNATIVE, HIGH);
#endif

#ifdef GPS_EN_PIN
  pinMode(GPS_EN_PIN, OUTPUT);
  digitalWrite(GPS_EN_PIN, 0);  // Disable the GPS receiver
#endif

  // DEBUG: Serial port
  Serial.begin(115200);  // GPS is connected to the uart's RX pin.

  // GPS: Serial port and enable pin
  Serial2.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

#ifdef GPS_EN_PIN
  digitalWrite(GPS_EN_PIN, 1);  // This turns on the GPS receiver, if hooked up.
#endif

// GPS: PPS-pin handling, if the PPS-pin is specified.
#ifdef GPS_PPS_PIN
  pinMode(GPS_PPS_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(GPS_PPS_PIN), pps_interrupt, FALLING);  // Interrupt is triggered on falling edge.
#endif
  uart_string.reserve(200);

  // Timer.
  display_timer.attach_ms(900, set_display_to_update);  // This starts a display frame update, every 500 milliseconds.
  DEBUG_PRINTLN;

// Network.
#ifdef OPERATE_AS_ETH
  //WiFi.onEvent(EthEvent); //attach ETH PHY event handler
  ETH.begin(ETH_ADDR, ETH_POWER_PIN, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_TYPE, ETH_CLK_MODE);  // Enable ETH
  ip = ETH.localIP().toString();
#endif

  Udp.begin(NTP_PORT);  // start udp server

  // // TFT screen
  // tft.init();
  // tft.setRotation(3);
  // tft.fillScreen(TFT_BLACK);

// Debug message via uart.
#ifdef DEBUG
  Serial.println("Hardware initialised, now entering loop.");

#ifdef OPERATE_AS_ETH
  Serial.print("ETH IP:");
  Serial.println(ETH.localIP());
#endif

#ifdef OPERATE_AS_AP
  Serial.print("AP IP:");
  Serial.println(WiFi.softAPIP());
#endif
  DEBUG_PRINTLN;

#ifdef OPERATE_AS_STA
  Serial.print("STA IP:");
  Serial.println(WiFi.localIP());
#endif
#endif
}

void loop() {
  DEBUG_PRINTLN;

  /*
   * Do we have a GPS sentence?
   */
  // noInterrupts();  // disable interrupts while waiting for a sentence.
  if (Serial2.available()) {
    uart_string = Serial2.readStringUntil('\n');  // Read string until carriage return.
    there_is_new_data = 1;
    gpsInfluxFeed(uart_string);

    //   // 2) Post to Influx if interval elapsed
    gpsInfluxUpdate();
#ifdef DEBUG
    //Serial.println("Echo the GPS sentence");
    //Serial.println(uart_string); // Echo the GPS sentence.
#endif
  }
  // interrupts();  // Re-enable interrupts after the string is received.
  DEBUG_PRINTLN;


  /*
   * Parse the GPS sentence.
   */
  if (there_is_new_data) {
    uart_string_length = uart_string.length();
    uart_string.toCharArray(gps_sentence, uart_string_length);  // Copy the string to a character array so minmea can process it.
    there_is_new_data = 0;
    uart_string = "";               // Clear this string.
    Serial2.flush();                // If there was something remaining in the buffer, it's gone now.
    parse_rmc();                    // Update the date and time using gps_sentence
    can_respond_to_packets = true;  // Now we can respond to NTP requests.
  }
  DEBUG_PRINTLN;

  /*
   * Can we update the display?
   * (Ticker calls timer function, sets semaphore, and then execute this statement once.)
   */
  if (update_the_display) {
    drive_display();             // Update the contents of the screen.
    update_the_display = false;  // Make sure this is being executed once per call.
  }

  /*
   * Have we got a time sync request??
   */
  DEBUG_PRINTLN;

  // process NTP requests
  IPAddress remoteIP;  // this will store the remote hosts's IP address
  int remotePort;      // the port it was sent from
  DEBUG_PRINTLN;
  int packetSize = Udp.parsePacket();  // this is what's failing
  DEBUG_PRINTLN;
#ifdef DEBUG
  if (packetSize > 0) {
    Serial.print("\n[NET] UDP Packet received - ");
  }
#endif

  if (packetSize && !sysStatus.event.notPPS && !sysStatus.event.notSatellite && can_respond_to_packets)  // we've got a packet, and there is GPS fix, and we have received new GPS data since the last NTP reply
  {
    DEBUG_PRINTLN;

    /*
       * Process NTP request.
       */

    // Disable interrupts for this time.
    //noInterrupts();
    receive_time = get_time_now();  // Log the time the packet came in

    //store sender ip and port for later use
    remoteIP = Udp.remoteIP();
    remotePort = Udp.remotePort();

#ifdef DEBUG
    // Some very useful debug stuff. But it takes time!
    Serial.print("*********************************\r\nReceived UDP packet with ");
    Serial.print(packetSize);
    Serial.print(" bytes size - ");
    Serial.print("SourceIP ");
    for (uint8_t i = 0; i < 4; i++) {
      Serial.print(remoteIP[i], DEC);
      if (i < 3) {
        Serial.print(".");
      }
    }
    Serial.print(", Port ");
    Serial.println(remotePort);
    Serial.print("query: ");
    Serial.print(receive_time.toStringTime());
    Serial.print(",");
    Serial.print(receive_time.microsfraction());
    Serial.println();
    DEBUG_PRINTLN;

#endif

    // We've received a packet, read the data from it
    // read the packet into the buffer
    Udp.read(packetBuffer, NTP_PACKET_SIZE);

    // get client transmit time (becomes originTime in reply packet)
    for (int i = 0; i < 8; i++)
      origTimeTs[i] = packetBuffer[40 + i];

    sendNTPpacket(remoteIP, remotePort);

#ifdef DEBUG
    //send NTP reply
    Serial.print("reply: ");
    Serial.print(transmit_time.toStringTime());
    Serial.print(",");
    Serial.println(transmit_time.microsfraction());
    Serial.println("NTP reply sent.\r\n*********************************");
    DEBUG_PRINTLN;
#endif
    DEBUG_PRINTLN;

    //Re-enable the interrupts
    //interrupts();

    //can_respond_to_packets == false; // This is set so no NTP responses will be sent until we have new GPS data.
  }
  DEBUG_PRINTLN;
}

void parse_rmc() {
  DEBUG_PRINTLN;

  // This function parses the $GPRMC NMEA sentence. This function works with global variables, and updates the reference_time accordingly.
  switch (minmea_sentence_id(gps_sentence, true)) {
    // In this function, the boolean value enables GPS sentence checksum verification.
    case MINMEA_SENTENCE_RMC:
      if (minmea_parse_rmc(&rmc_frame, gps_sentence)) {
        sysStatus.event.notSatellite = false;

        // Checking for zero data with GPS
        if (rmc_frame.date.year == -1 || rmc_frame.date.month == -1 || rmc_frame.date.day == -1 || rmc_frame.time.hours == -1 || rmc_frame.time.minutes == -1 || rmc_frame.time.seconds == -1)
          sysStatus.event.notSatellite = true;
        else
          uart_time = DateTime((uint16_t)rmc_frame.date.year, (uint16_t)rmc_frame.date.month, (uint16_t)rmc_frame.date.day, (uint16_t)rmc_frame.time.hours, (uint16_t)rmc_frame.time.minutes, (uint16_t)rmc_frame.time.seconds, TIMING_OFFSET_US);  // (uint16_t)61, (unsigned long)rmc_frame.time.microseconds);

        // For some reason, my GPS returns dummy data when it can't find fix. Not sure why this happens.
        // well, this will be a problem in 2036. This was written in 2019, so there is time to fix it :)
        if (reference_time.unixtime() == (uint32_t)2085978497)
          sysStatus.event.notSatellite = true;

// Update the microsecond counter too, if no PPS pin is assigned.
#ifndef GPS_PPS_PIN
        pps_interrupt();  // If the system had a GPS_PPS_PIN, this function would be executed in an interrupt.
#endif
        //Serial.println("RMC sentence received.");
      } else {
        // If the parsing failed
        //Serial.println("Bad RMC sentence!");
      }
      break;

    case MINMEA_SENTENCE_GGA:
      if (minmea_parse_gga(&gga_frame, gps_sentence))
        satellites_tracked = gga_frame.satellites_tracked;
      //Serial.println("GGA sentence received.");
      break;
    case MINMEA_SENTENCE_GSA:
      //Serial.println("GSA sentence received.");
      break;

    case MINMEA_SENTENCE_VTG:
      //Serial.println("VTG sentence received.");
      break;

    case MINMEA_SENTENCE_ZDA:
      //Serial.println("ZDA sentence received.");
      break;

    default:
      //Do nothing.
      break;
  }
  DEBUG_PRINTLN;
}

DateTime get_time_now(void) {
  DEBUG_PRINTLN;

  /*
   * this function derives the time from the reference time, and gets the number of microseconds since the last GPS update.
   * It derives time from reference_time, which is set by the GPS, and the PPS interrupt, if any.
   */
  DateTime stuff_to_return;                                    // This is what we are going to return.
  unsigned long current_microsecond_counter = micros();        // Get the number of microseconds
  uint32_t current_reference_time = reference_time.ntptime();  // This gets the time as per NTP
  // This calculates the time difference since the last $GPRMC NMEA sentence in microseconds, and takes a manual offset into account
  unsigned long microsecond_difference = (current_microsecond_counter - microsecond_counter) + TIMING_OFFSET_US;

  // debug stuff
  /*Serial.print("current_microsecond_counter: ");
    Serial.println(current_microsecond_counter);
    Serial.print("microsecond_counter: ");
    Serial.println(microsecond_counter);
    Serial.print("microsecond_difference: ");
    Serial.println(microsecond_difference);*/

  // Did the PPS impulses stop happening? Make this crash!
  if ((microsecond_difference > 5000000) || (microsecond_difference > -5000000)) {
    sysStatus.event.notPPS = true;
#ifdef DEBUG
    Serial.println("No GPS data or PPS pulse was received in the past 5 seconds.");
#endif
  } else
    sysStatus.event.notPPS = false;

  // Did we have a variable overflow?
  if (microsecond_counter > current_microsecond_counter) {
    microsecond_difference = 0;  // This is going to introduce a an error up to a second at variable overflow.
    //microsecond_difference = (current_microsecond_counter - ((unsigned long)-1 - microsecond_counter)) + TIMING_OFFSET_US;
  }
  DEBUG_PRINTLN;

  // Did we experience time longer than a second? A Lot longer too? Compensate.
  while ((microsecond_difference >= 10000000)) {
    // If we get a larger than 1 second here, we have a problem.
    microsecond_difference = microsecond_difference - 1000000;  // Remove the extra second offset
    //current_reference_time += 1; //...and add it to the time.
  }

  // Now let's assemble the new DateTime object so we can return it.
  DEBUG_PRINTLN;

  stuff_to_return = DateTime(current_reference_time, microsecond_difference);
  return stuff_to_return;
}

uint64_t DateTimeToNtp64(DateTime time_to_send) {
  DEBUG_PRINTLN;

  /*
     * This function generates the time information required for the NTP packet.
     */
  uint64_t time_stamp;  // 64-bit time stamp.

  time_stamp = (((uint64_t)time_to_send.ntptime()) << 32);                // Shove it to the top 32 bits.
  time_stamp |= (uint64_t)(time_to_send.microsfraction() * 4294.967296);  // Add the lower 32-bit nibble, which is the precise information

  return (time_stamp);
}

// send NTP reply to the given address
void sendNTPpacket(IPAddress remoteIP, int remotePort) {
  DEBUG_PRINTLN;

  /*
   * This function assembles an NTP packet, and sends it back to the host requesting it.
   */

  // LI: 0, Version: 4, Mode: 4 (server)
  //packetBuffer[0] = 0b00100100;
  // Not a leap second (LI=0), NTP version: 3, Mode: 4 (server)
  packetBuffer[0] = 0b00011100;

  // Stratum, or type of clock. Since we have the clock derived from a GPS, we are stratum 1, no matter how inaccurate are we.
  packetBuffer[1] = 0b00000001;

  // Polling Interval: the log2 value of the maximum interval between souccessive messages
  packetBuffer[2] = 2;  // Was 4.

  // Peer Clock Precision
  // log2(sec)
  // 0xF6 <--> -10 <--> 0.0009765625 s
  // 0xF7 <--> -9 <--> 0.001953125 s
  // 0xF8 <--> -8 <--> 0.00390625 s
  // 0xF9 <--> -7 <--> 0.0078125 s
  // 0xFA <--> -6 <--> 0.0156250 s
  // 0xFB <--> -5 <--> 0.0312500 s
  // 0xFC <--> -4 <--> 0.0625 s
  // 0xFD <--> -3 <--> 0.125 s

#ifndef GPS_PPS_PIN
  // report a worse precision if a GPS without PPS output was used.
  packetBuffer[3] = 0xFC;
#else
  packetBuffer[3] = 0xF6;  // the actual clock precision is better, but let's be conservative, this is just a microcontroller!
#endif
  DEBUG_PRINTLN;

  // 8 bytes for Root Delay & Root Dispersion
  // Root delay is 0, becuase we got our clock from a GPS.
  packetBuffer[4] = 0;
  packetBuffer[5] = 0;
  packetBuffer[6] = 0;
  packetBuffer[7] = 0;
  DEBUG_PRINTLN;

  // Root dispersion. Refers to clock frequency tolerance. Well, I guess this is a bit optimistic :)
  packetBuffer[8] = 0;
  packetBuffer[9] = 0;
  packetBuffer[10] = 0;
  packetBuffer[11] = 0x50;

  // Time source is GPS. The external reference source code is GPS.
  packetBuffer[12] = 71;  // G
  packetBuffer[13] = 80;  // P
  packetBuffer[14] = 83;  // S
  packetBuffer[15] = 0;

  // Reference Time.
  uint64_t refT = DateTimeToNtp64(get_time_now());  // This one fetches the current time.

  packetBuffer[16] = (int)((refT >> 56) & 0xFF);
  packetBuffer[17] = (int)((refT >> 48) & 0xFF);
  packetBuffer[18] = (int)((refT >> 40) & 0xFF);
  packetBuffer[19] = (int)((refT >> 32) & 0xFF);
  packetBuffer[20] = (int)((refT >> 24) & 0xFF);
  packetBuffer[21] = (int)((refT >> 16) & 0xFF);
  packetBuffer[22] = (int)((refT >> 8) & 0xFF);
  packetBuffer[23] = (int)(refT & 0xFF);

  // Origin Time
  //copy old transmit time to origtime

  for (int i = 24; i < 32; i++)
    packetBuffer[i] = origTimeTs[i - 24];

  // write Receive Time to bytes 32-39
  refT = DateTimeToNtp64(receive_time);
  DEBUG_PRINTLN;

  packetBuffer[32] = (int)((refT >> 56) & 0xFF);
  packetBuffer[33] = (int)((refT >> 48) & 0xFF);
  packetBuffer[34] = (int)((refT >> 40) & 0xFF);
  packetBuffer[35] = (int)((refT >> 32) & 0xFF);
  packetBuffer[36] = (int)((refT >> 24) & 0xFF);
  packetBuffer[37] = (int)((refT >> 16) & 0xFF);
  packetBuffer[38] = (int)((refT >> 8) & 0xFF);
  packetBuffer[39] = (int)(refT & 0xFF);

  // get current time + write  as Transmit Time to bytes 40-47
  transmit_time = get_time_now();
  refT = DateTimeToNtp64(transmit_time);

  packetBuffer[40] = (int)((refT >> 56) & 0xFF);
  packetBuffer[41] = (int)((refT >> 48) & 0xFF);
  packetBuffer[42] = (int)((refT >> 40) & 0xFF);
  packetBuffer[43] = (int)((refT >> 32) & 0xFF);
  packetBuffer[44] = (int)((refT >> 24) & 0xFF);
  packetBuffer[45] = (int)((refT >> 16) & 0xFF);
  packetBuffer[46] = (int)((refT >> 8) & 0xFF);
  packetBuffer[47] = (int)(refT & 0xFF);

  // send reply:
  Udp.beginPacket(remoteIP, remotePort);
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
  DEBUG_PRINTLN;
}

// This is the interrupt function.
void pps_interrupt() {
  // Log the microseconds counter, so we know what the time was when the interrupt happened.
  microsecond_counter = micros();
  // This function sets the reference time, every second.
  uint32_t reference_time_to_process_in_ntp = uart_time.ntptime();  // Save the current time as NTP time.

#ifdef GPS_PPS_PIN
  reference_time_to_process_in_ntp++;  // Increase the number of seconds, when using PPS interrupt.
#endif
  reference_time = DateTime(reference_time_to_process_in_ntp, (unsigned long)TIMING_OFFSET_US);  // This updates the global reference time.

  // Print the current time, so we can check the fraction stuff.
  // reference_time.print();
}

void set_display_to_update(void) {
  // This function adjusts a semaphore, which controls the display update. The Adafruit library doesn't seem to like being used from an interrupt function.
  update_the_display = true;  // Set this global variable. The display is updated from the loop() function.
}

void drive_display() {
  chackIP();

  // Refresh screen after changing error flags
  if (sysStatusCache != sysStatus.all) {
    sysStatusCache = sysStatus.all;
    //     tft.fillScreen(TFT_BLACK);

    //     tft.setTextColor(TFT_WHITE, TFT_BLACK);
    //     tft.setCursor(5, 16 * 0, 2);  // 16
    //     tft.println("GPS NTP Server");

    //     tft.setTextColor(TFT_GREEN, TFT_BLACK);
    //     tft.setCursor(5, 16 * 1, 2);
    // #ifdef OPERATE_AS_ETH
    //     tft.print("ETH: ");
    // #endif
    // #ifdef OPERATE_AS_AP
    //     tft.print("AP: ");
    // #endif
    // #ifdef OPERATE_AS_STA
    //     tft.print("STA: ");
    // #endif
    //     tft.println(ip);
    sysStatus.event.newIP = 0;
  }

  //satellites tracked
  // tft.setTextColor(TFT_ORANGE, TFT_BLACK);
  // tft.setCursor(135, 0, 2);
  // char charDate[4];
  // sprintf(charDate, "S%02d", satellites_tracked);
  // tft.print(charDate);

  //
  // DateTime time_tmp = get_time_now();
  // if (!(sysStatus.event.notSatellite || sysStatus.event.notPPS)) {
  //   tft.setTextColor(TFT_MAGENTA, TFT_BLACK);
  //   tft.setCursor(28, 5 + 16 * 2, 4);  // 26
  //   tft.println(time_tmp.toStringTime());

  //   tft.setTextColor(0xFC9F, TFT_BLACK);
  //   tft.setCursor(38, 5 + 16 * 2 + 21, 2);
  //   tft.println(time_tmp.toStringDate());  //reference_time
  // } else {
  //   if (sysStatus.event.notSatellite) {
  //     tft.setTextColor(TFT_RED, TFT_BLACK);
  //     tft.setCursor(38, 16 * 2, 2);
  //     tft.println("GPS ERROR");
  //   }
  //   if (sysStatus.event.notPPS) {
  //     tft.setTextColor(TFT_RED, TFT_BLACK);
  //     tft.setCursor(38, 16 * 3, 2);
  //     tft.println("PPS ERROR");
  //   }
  // }
}

//String IP;
int countNotIP = 0;
void chackIP() {
#ifdef OPERATE_AS_ETH
  ip = ETH.localIP().toString();
  //There is a bug. disconnected once a day
  if (ip == "0.0.0.0") {
    countNotIP++;
    if (countNotIP >= 5) {
      //Serial.print("init eth");
      countNotIP = 0;
      ETH.begin(ETH_ADDR, ETH_POWER_PIN, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_TYPE, ETH_CLK_MODE);  // Enable ETH
    }
  }
#endif
#ifdef OPERATE_AS_AP
  ip = WiFi.softAPIP().toString();
#endif
#ifdef OPERATE_AS_STA
  ip = WiFi.localIP().toString();
#endif
  if (chackIPCache != ip) {
    chackIPCache = ip;
    sysStatus.event.newIP = 1;
  }
}



// /*
//  * GPS NTP Server for ESP32 with Fallback to internal clock
//  * Uses hardware UART to read GPS, parse time, and serve NTP
//  * When no GPS fix, serves estimated time since last fix (microCRTs) and sets LI=3
//  */

// #include <Wire.h>
// #include <time.h>
// #include <string.h>
// #include <WiFi.h>
// #include <WiFiUdp.h>
// #include <Ticker.h>
// #include <SPI.h>
// #include "minmea.h"
// #include "DateTime.h"
// #include "arduino_secrets.h"  // contains ssid & pass if STA mode
// #include "GpsInflux.h"

// // ======== Configuration ========
// #define DEBUG

// // Wi-Fi/Ethernet mode: choose one
// #define MODE_STA
// // #define MODE_AP
// // #define MODE_ETH

// // AP mode settings
// #ifdef MODE_AP
// static const char AP_SSID[] = "NTP";
// static const char AP_PASS[] = "NTP";
// #define AP_CHANNEL 9
// #define AP_HIDDEN_SSID false
// #define AP_MAX_CONN 3
// #endif

// // Ethernet settings
// #ifdef MODE_ETH
// #include <ETH.h>
// #define ETH_ADDR 1
// #define ETH_POWER_PIN 16
// #define ETH_MDC_PIN 23
// #define ETH_MDIO_PIN 18
// #define ETH_TYPE ETH_PHY_LAN8720
// #endif

// // GPS / UART
// #define GPS_BAUDRATE 9600
// #define GPS_RX_PIN 18
// #define GPS_TX_PIN 16
// #define GPS_PPS_PIN 17
// #define TIMING_OFFSET_US 1u

// // NTP
// #define NTP_PORT 123
// #define NTP_PACKET_SIZE 48

// // ======== Globals ========
// WiFiUDP Udp;
// Ticker displayTimer;
// String ipAddr;
// uint8_t packetBuffer[NTP_PACKET_SIZE];

// // Status flags
// union SystemStatus {
//   uint8_t all;
//   struct {
//     uint8_t noSatellite : 1;
//     uint8_t noPPS : 1;
//     uint8_t newIP : 1;
//     uint8_t reserved : 5;
//   } bits;
// } sysStatus;

// // Fallback indicator
// bool usingFallback = false;

// // GPS parsing
// String gpsBuffer;
// char gpsSentence[MINMEA_MAX_LENGTH];
// struct minmea_sentence_rmc rmcFrame;
// struct minmea_sentence_gga ggaFrame;

// // Timekeeping
// DateTime referenceTime;
// unsigned long lastPpsMicros = 0;
// unsigned long microOffset = TIMING_OFFSET_US;

// // Display update flag
// volatile bool displayUpdateFlag = false;

// // NTP timestamps
// DateTime rxTime, txTime, gpsTime;
// byte clientOrigTs[8];

// // Forward declarations
// void initNetworking();
// void initGPS();
// void initDisplayTimer();
// void handleGPS();
// void handleDisplayUpdate();
// void handleNTP();
// void parseGpsSentence();
// DateTime getCurrentTime();
// uint64_t toNtp64(const DateTime& dt);
// void sendNtpReply(const IPAddress& remoteIP, uint16_t port);
// void updateIp();
// void setDisplayFlag();

// // ======== Setup ========
// void setup() {
//   Serial.begin(115200);
//   randomSeed(analogRead(A0));
//   initNetworking();
//   initGPS();
//   Udp.begin(NTP_PORT);
//   initDisplayTimer();
// #ifdef DEBUG
//   Serial.println("Setup complete.");
//   Serial.print("IP: ");
//   Serial.println(ipAddr);
// #endif
// }

// // ======== Main Loop ========
// void loop() {
//   String data = Serial2.readStringUntil('\n');
//   handleGPS(data);
//   gpsInfluxFeed(data);

//   // 2) Post to Influx if interval elapsed
//   gpsInfluxUpdate();
//   if (sysStatus.bits.newIP)
//     updateIp();
//   handleDisplayUpdate();
//   handleNTP();
// }

// // ======== Initialization ========

// void initNetworking() {
// #ifdef MODE_STA
//   WiFi.mode(WIFI_STA);
//   WiFi.begin(ssid, pass);
//   while (WiFi.status() != WL_CONNECTED) {
//     delay(500);
//   }
//   ipAddr = WiFi.localIP().toString();
// #endif

// #ifdef MODE_AP
//   WiFi.softAP(AP_SSID, AP_PASS, AP_CHANNEL, AP_HIDDEN_SSID, AP_MAX_CONN);
//   ipAddr = WiFi.softAPIP().toString();
// #endif

// #ifdef MODE_ETH
//   pinMode(ETH_POWER_PIN, OUTPUT);
//   digitalWrite(ETH_POWER_PIN, HIGH);
//   ETH.begin(ETH_ADDR, ETH_POWER_PIN, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_TYPE);
//   ipAddr = ETH.localIP().toString();
// #endif
// }

// void initGPS() {
//   gpsBuffer.reserve(128);
//   Serial2.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
// #ifdef GPS_PPS_PIN
//   pinMode(GPS_PPS_PIN, INPUT_PULLUP);
//   attachInterrupt(
//     digitalPinToInterrupt(GPS_PPS_PIN), []() {
//       lastPpsMicros = micros();
//       referenceTime = DateTime(gpsTime.ntptime() + 1, microOffset);
//     },
//     FALLING);
// #endif
// }

// void initDisplayTimer() {
//   displayTimer.attach_ms(900, setDisplayFlag);
// }

// // ======== Handlers ========

// void handleGPS(String gpsData) {
//   gpsBuffer = gpsData;
//   gpsBuffer.toCharArray(gpsSentence, gpsBuffer.length() + 1);
//   parseGpsSentence();
//   gpsBuffer.clear();
// }

// void handleDisplayUpdate() {
//   if (!displayUpdateFlag) return;
//   // TODO: update TFT or Serial output
//   displayUpdateFlag = false;
// }

// void handleNTP() {
//   int packetSize = Udp.parsePacket();
//   if (packetSize == 0) return;

//   // Decide fallback or normal
//   if (sysStatus.bits.noSatellite || sysStatus.bits.noPPS) {
//     usingFallback = true;
// #ifdef DEBUG
//     Serial.println("Fallback mode: serving internal clock estimate");
// #endif
//   } else {
//     usingFallback = false;
//   }

//   // Read client request
//   Udp.read(packetBuffer, packetSize);
//   memcpy(clientOrigTs, packetBuffer + 40, 8);

//   IPAddress remoteIP = Udp.remoteIP();
//   uint16_t port = Udp.remotePort();

//   rxTime = getCurrentTime();
//   sendNtpReply(remoteIP, port);
// }

// // ======== GPS Parsing ========

// void parseGpsSentence() {
//   switch (minmea_sentence_id(gpsSentence, true)) {
//     case MINMEA_SENTENCE_RMC:
//       if (minmea_parse_rmc(&rmcFrame, gpsSentence)) {
//         sysStatus.bits.noSatellite = false;
//         if (rmcFrame.date.year < 0) {
//           sysStatus.bits.noSatellite = true;
//         } else {
//           gpsTime = DateTime(rmcFrame.date.year,
//                              rmcFrame.date.month,
//                              rmcFrame.date.day,
//                              rmcFrame.time.hours,
//                              rmcFrame.time.minutes,
//                              rmcFrame.time.seconds,
//                              microOffset);
//         }
// #ifndef GPS_PPS_PIN
//         lastPpsMicros = micros();
//         referenceTime = DateTime(gpsTime.ntptime(), microOffset);
// #endif
//       }
//       break;

//     case MINMEA_SENTENCE_GGA:
//       if (minmea_parse_gga(&ggaFrame, gpsSentence)) {
//         // satellites tracked (could set debug counts)
//       }
//       break;

//     default:
//       break;
//   }
// }

// // ======== Time Utilities ========

// DateTime getCurrentTime() {
//   unsigned long nowMicros = micros();
//   unsigned long deltaUs = (nowMicros - lastPpsMicros) + microOffset;
//   // Only declare PPS lost if really long gap (e.g., >5s)
//   sysStatus.bits.noPPS = (deltaUs > 5000000);
//   return DateTime(referenceTime.ntptime(), deltaUs);
// }

// uint64_t toNtp64(const DateTime& dt) {
//   uint64_t hi = (uint64_t)dt.ntptime() << 32;
//   uint64_t lo = (uint64_t)(dt.microsfraction() * 4294.967296);
//   return hi | lo;
// }

// void sendNtpReply(const IPAddress& remoteIP, uint16_t port) {
//   // NTP header configuration
//   // LI (2 bits): 11 = unsynchronized (fallback), 00 = synchronized (GPS)
//   // Version (3 bits): NTP v4 (100)
//   // Mode (3 bits): Server mode (100)
//   packetBuffer[0] = usingFallback ? 0xDC : 0x1C;
//   packetBuffer[1] = 1;     // Stratum: 1 = primary server (when using GPS)
//   packetBuffer[2] = 2;     // Poll interval
//   packetBuffer[3] = 0xF6;  // Precision (~15 us)

//   // Root delay and dispersion (unused in basic implementation)
//   memset(packetBuffer + 4, 0, 8);

//   // Reference Identifier: ASCII "GPS" for GPS source
//   packetBuffer[12] = 'G';
//   packetBuffer[13] = 'P';
//   packetBuffer[14] = 'S';
//   packetBuffer[15] = 0;

//   /* === Critical NTP Timestamps === */

//   // Reference Timestamp (Offset 16-23)
//   // Time when server's clock was last set/corrected
//   // Either GPS time (PPS sync) or fallback internal clock
//   uint64_t refTs = toNtp64(getCurrentTime());
//   for (int i = 0; i < 8; ++i)
//     packetBuffer[16 + i] = (refTs >> (56 - 8 * i)) & 0xFF;

//   // Originate Timestamp (Offset 24-31)
//   // Client's request transmission time (copied from client packet)
//   memcpy(packetBuffer + 24, clientOrigTs, 8);

//   // Receive Timestamp (Offset 32-39)
//   // Server's time when request was received
//   uint64_t rxTs = toNtp64(rxTime);
//   for (int i = 0; i < 8; ++i)
//     packetBuffer[32 + i] = (rxTs >> (56 - 8 * i)) & 0xFF;

//   // Transmit Timestamp (Offset 40-47)
//   // Server's time when response is sent
//   txTime = getCurrentTime();
//   uint64_t txTs = toNtp64(txTime);
//   for (int i = 0; i < 8; ++i)
//     packetBuffer[40 + i] = (txTs >> (56 - 8 * i)) & 0xFF;

//   // Send completed NTP packet
//   Udp.beginPacket(remoteIP, port);
//   Udp.write(packetBuffer, NTP_PACKET_SIZE);
//   Udp.endPacket();
// }


// // ======== Helpers ========

// void updateIp() {
// #ifdef MODE_STA
//   ipAddr = WiFi.localIP().toString();
// #elif defined(MODE_AP)
//   ipAddr = WiFi.softAPIP().toString();
// #elif defined(MODE_ETH)
//   ipAddr = ETH.localIP().toString();
// #endif
//   sysStatus.bits.newIP = 0;
// #ifdef DEBUG
//   Serial.print("New IP: ");
//   Serial.println(ipAddr);
// #endif
// }

// void setDisplayFlag() {
//   displayUpdateFlag = true;
// }
