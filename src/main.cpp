// #include <SPI.h>
// #include <SD.h>
// #include <DFRobot_BMP388_SPI.h>
// #include <wiring_private.h>
// #include <NMEAGPS.h>
// #include <GPSport.h>
// #include "min.h"
// #include "bma2x2_support.hpp"
// #include "bmg160_support.hpp"
//
// #define LED_PIN SCL
// #define BMP_CS 12
// #define BMP2_CS 26
// #define SD_CS 5
// #define ACCEL_CS 13
// #define GYRO_CS 3
// #define ADXL_CS 0
//
// #define SRAD_TX 10
// #define SRAD_RX 11
// #define SGPS_TX 4
// #define SGPS_RX 1
//
// #define MODE_RECEIVING 1
// #define MODE_TRANSMITTING 2
//
// Uart SerialS6C(&sercom1, SRAD_RX, SRAD_TX, SERCOM_RX_PAD_0, UART_TX_PAD_2);
// Uart SerialGPS(&sercom2, SGPS_RX, SGPS_TX, SERCOM_RX_PAD_2, UART_TX_PAD_0);
//
// void SERCOM1_Handler()
// {
//   SerialS6C.IrqHandler();
// }
//
// void SERCOM2_Handler()
// {
//   SerialGPS.IrqHandler();
// }
//
// static void printL( Print & outs, int32_t degE7 )
// {
//   // Extract and print negative sign
//   if (degE7 < 0) {
//     degE7 = -degE7;
//     outs.print( '-' );
//   }
//
//   // Whole degrees
//   int32_t deg = degE7 / 10000000L;
//   outs.print( deg );
//   outs.print( '.' );
//
//   // Get fractional degrees
//   degE7 -= deg*10000000L;
//
//   // Print leading zeroes, if needed
//   int32_t factor = 1000000L;
//   while ((degE7 < factor) && (factor > 1L)){
//     outs.print( '0' );
//     factor /= 10L;
//   }
//
//   // Print fractional degrees
//   outs.print( degE7 );
// }
//
// uint16_t min_tx_space(uint8_t port) {
// 	uint16_t n = 1;
// 	if (port == 0) n = SerialS6C.availableForWrite();
// 	return n;
// }
//
// void min_tx_byte(uint8_t port, uint8_t byte) {
// 	if (port == 0) SerialS6C.write(&byte, 1U);
// }
//
// uint32_t min_time_ms() {
//   return millis();
// }
//
// void min_tx_start(uint8_t port) {}
//
// void min_tx_finished(uint8_t port) { SerialS6C.flush(); }
//
// struct min_context min_ctx_s6c;
//
// struct bma2x2_t bma;
//
// void setup() {
//   // initialize ports
//   PORT->Group[0].DIR.reg = PORT_PA27;
//   PORT->Group[0].OUTSET.reg = (1UL << (27 % 32));
//   //COMMEMNT OUT B4 LAUNCH
//   PORT->Group[0].DIR.reg |= PORT_PA28;
//   PORT->Group[0].OUTSET.reg = (1UL << (28 % 32));
//
//   //initialize pins
//   pinMode(LED_PIN, OUTPUT);
//   pinMode(SD_CS, OUTPUT);
//   pinMode(ACCEL_CS, OUTPUT);
//   pinMode(BMP_CS, OUTPUT);
//   pinMode(ADXL_CS, OUTPUT);
//   pinMode(GYRO_CS, OUTPUT);
//   pinMode(BMP2_CS,OUTPUT);
//
//   digitalWrite(BMP2_CS,HIGH);
//   digitalWrite(SD_CS, HIGH);
//   digitalWrite(BMP_CS, HIGH);
//   digitalWrite(ACCEL_CS, HIGH);
//   digitalWrite(GYRO_CS, HIGH);
//   digitalWrite(ADXL_CS, HIGH);
//
//   // initialize USB connection
//   SerialUSB.begin(9600);
//
//   // initialize SD card
//   SD.begin(SD_CS);
//   char fname[32];
//   randomSeed(micros());
//   sprintf(fname, "%07d.csv", random(1000000));
//   File logFile = SD.open(fname, FILE_WRITE | O_CREAT);
//
//   if (logFile) {
//     logFile.println("pressure,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,"
//                     "temperature,latitude,longitude,altitude,time");
//   }
//
//   // initialize GPS
//   NMEAGPS gps;
//   gps_fix fix;
//   SerialGPS.begin(9600);
//   pinPeripheral(SGPS_RX, PIO_SERCOM_ALT);
//   pinPeripheral(SGPS_TX, PIO_SERCOM_ALT);
//
//  // initialize pressure sensor
//   DFRobot_BMP388_SPI bmp(BMP_CS);
//   bmp.begin();
//   // DFRobot_BMP388_SPI bmp(BMP2_CS); //This stupid library dosen't store the CS pins seperately...
//
//   // initialize accelerometer and gyroscope
//   bma2x2_init_accel();
//   bmg160_init_gyro();
//
//   // wait for sensors to boot
//   delay(2000);
//
//   struct bma2x2_accel_data_temp accel_data;
//   struct bmg160_data_t gyro_data;
//   while (true) {
//     SerialUSB.println("Logging...");
//     if (logFile) {
//       SerialUSB.println("Found file...");
//       logFile.print(bmp.readPressure());
//       logFile.print(",");
//
//       accel_data = bma2x2_read_accel();
//       logFile.print(accel_data.x);
//       logFile.print(",");
//       logFile.print(accel_data.y);
//       logFile.print(",");
//       logFile.print(accel_data.z);
//       logFile.print(",");
//
//       gyro_data = bmg160_read_gyro();
//       logFile.print(gyro_data.datax);
//       logFile.print(",");
//       logFile.print(gyro_data.datay);
//       logFile.print(",");
//       logFile.print(gyro_data.dataz);
//       logFile.print(",");
//
//       logFile.print(bmp.readTemperature());
//       logFile.print(",");
//
//       logFile.print(fix.latitudeL());
//       logFile.print(",");
//       logFile.print(fix.longitudeL());
//       logFile.print(",");
//       logFile.print(fix.altitude());
//       logFile.print(",");
//       logFile.print(fix.dateTime_ms());
//       logFile.println(",");
//
//       logFile.flush();
//
//       delay(50);
//       digitalWrite(LED_PIN, HIGH);
//       delay(50);
//       digitalWrite(LED_PIN, LOW);
//     }
//   }
// }
//
// void loop() {}

void displayInfo();
void batteryCheck();

#include <Arduino.h>
#include <wiring_private.h>
#include "SSIradio.h"
#include <TinyGPS++.h>
#include "ODriveArduino.h"
#include <string.h>

#define LED 21

#define SRAD_TX 36
#define SRAD_RX 35

#define SGPS_TX 4
#define SGPS_RX 1

#define ODRIVE_TX 30
#define ODRIVE_RX 31

#define CUTDOWN1 38
#define CUTDOWN2 2

#define logFile SerialUSB

char buf[64];
bool battery_warned = false;
unsigned long battery_time;
const float kMinVoltage = 13.0;
const int kNumWarnings = 10;
const unsigned int kODriveTimeout = 1000;
const char kStartQuery[] = "spinmebaby";
const char kStopQuery[] = "stop";
const char kSpeedQuery[] = "speed";
const char kReadQuery[] = "read";
const char kBatteryQuery[] = "battery";
const char kStartCommand[] = "w axis0.controller.vel_ramp_enable 1\n"
                             "w axis0.requested_state 5\n";
const char kStopCommand[] = "w axis0.requested_state 1\n";
const char kSpeedCommand[] = "w axis0.controller.vel_ramp_target %d\n";
const char kReadCommand[] = "r axis0.sensorless_estimator.vel_estimate\n";
const char kBatteryCommand[] = "r vbus_voltage\n";
const char kBatteryError[] = "BATTERY LOW\n";

Uart SerialS6C(&sercom1, SRAD_RX, SRAD_TX, SERCOM_RX_PAD_2, UART_TX_PAD_0);
void SERCOM1_Handler()
{
  SerialS6C.IrqHandler();
}

SSIradio S6C;

void receiveMsg(char* msg) {
  SerialUSB.println(msg);
}

TinyGPSPlus gps;
Uart SerialGPS(&sercom0, SGPS_RX, SGPS_TX, SERCOM_RX_PAD_2, UART_TX_PAD_0);
//#define Serial1 SerialGPS // oh god no

void SERCOM0_Handler(void) {
  SerialGPS.IrqHandler();
}

// Uart SerialODrive(&sercom5, ODRIVE_RX, ODRIVE_TX, SERCOM_RX_PAD_3, UART_TX_PAD_2);
// void SERCOM5_Handler()
// {
//   SerialODrive.IrqHandler();
// }
#define SerialODrive Serial



void setup(){
  SerialUSB.begin(115200);
  pinMode(LED, OUTPUT);

  pinMode(CUTDOWN1, OUTPUT);
  pinMode(CUTDOWN2, OUTPUT);
  digitalWrite(CUTDOWN1, HIGH); // OFF
  digitalWrite(CUTDOWN2, HIGH); // OFF

  S6C.set_callback(receiveMsg);
  S6C.begin(9600, &SerialS6C);
  digitalWrite(LED, HIGH);
  while (!S6C);
  delay(125);
  digitalWrite(LED, LOW);

  pinPeripheral(SGPS_RX, PIO_SERCOM);
  pinPeripheral(SGPS_TX, PIO_SERCOM);
  SerialGPS.begin(9600);

  //delay(10000);
  SerialUSB.print(F("TinyGPS++ library v. "));
  SerialUSB.println(TinyGPSPlus::libraryVersion());
  SerialUSB.println();

  SerialODrive.begin(115200);
}

void loop() {

  batteryCheck();

  //SerialODrive.println("doot");
  //SerialUSB.println("doot");
  //S6C.tx("doot");
  //delay(500);
  //digitalWrite(LED, LOW);
  //SerialUSB.println("doot");

  // This sketch displays information every time a new sentence is correctly encoded.
  digitalWrite(LED, HIGH);
  //delay(50);
  if (SerialGPS.available() > 0) {
    //SerialUSB.println(SerialGPS.read());
    if (gps.encode(SerialGPS.read())) {
      displayInfo();
      delay(50);
    }
  }
  digitalWrite(LED, LOW);
  //delay(50);

  //SerialGPS.println("doot");

  // if (millis() > 35000 && gps.charsProcessed() < 10)
  // {
  //   SerialUSB.println(F("No GPS detected: check wiring."));
  //   digitalWrite(LED, HIGH);
  //   while(true);
  // }


  //SerialS6C.println("doot");
  //delay(500);
}


void displayInfo()
{
  SerialUSB.print(F("Location: "));
  if (gps.location.isValid())
  {
    SerialUSB.print(gps.location.lat(), 6);
    SerialUSB.print(F(","));
    SerialUSB.print(gps.location.lng(), 6);
  }
  else
  {
    SerialUSB.print(F("INVALID"));
  }

  SerialUSB.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    SerialUSB.print(gps.date.month());
    SerialUSB.print(F("/"));
    SerialUSB.print(gps.date.day());
    SerialUSB.print(F("/"));
    SerialUSB.print(gps.date.year());
  }
  else
  {
    SerialUSB.print(F("INVALID"));
  }

  SerialUSB.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) SerialUSB.print(F("0"));
    SerialUSB.print(gps.time.hour());
    SerialUSB.print(F(":"));
    if (gps.time.minute() < 10) SerialUSB.print(F("0"));
    SerialUSB.print(gps.time.minute());
    SerialUSB.print(F(":"));
    if (gps.time.second() < 10) SerialUSB.print(F("0"));
    SerialUSB.print(gps.time.second());
    SerialUSB.print(F("."));
    if (gps.time.centisecond() < 10) SerialUSB.print(F("0"));
    SerialUSB.print(gps.time.centisecond());
  }
  else
  {
    SerialUSB.print(F("INVALID"));
  }

  SerialUSB.println();
}

void batteryCheck(){
  SerialUSB.println("Battery check!");
  SerialODrive.write(kBatteryCommand, strlen(kBatteryCommand));

  unsigned long start = millis();
  while(SerialODrive.available() > 0 && millis() - start < kODriveTimeout) {
    Serial.println("Waiting!");
    delay(10);
  }

  SerialODrive.readBytes(buf, 64U);
  SerialUSB.println(buf);

  // SerialODrive.readBytes(buf, 64U);
  // float voltage;
  // sscanf(buf, "%f\n", &voltage);
  // SerialUSB.println(voltage);
}
