#include <Arduino.h>
#include <wiring_private.h>
#include "SSIradio.h"
#include <TinyGPS++.h>
#include "ODriveArduino.h"
#include <string.h>

void displayInfo();
void updateLatLongAlt();
uint8_t batteryCheck();
void sendCoords(int version);
void cmdODrive(char* buf);

#define LED 21

#define SRAD_TX 36
#define SRAD_RX 35

#define SGPS_TX 4
#define SGPS_RX 1

#define ODRIVE_TX 30
#define ODRIVE_RX 31

#define CUTDOWN1 38
#define RECOVERY 2

#define logFile SerialUSB

char buf[64];
char rateBuf[20];
bool battery_warned = false;
unsigned long battery_time;
const float kMinVoltage = 13.0;
const int kNumWarnings = 10;
const unsigned int kODriveTimeout = 1000;
const char kSpinQuery[] = "spin";
const char kCutdownQuery[] = "cutdown";
const char kHeatQuery[] = "heat";
const char kCoolQuery[] = "cool";
const char kResetQuery[] = "reset";
// const char kStopQuery[] = "stop";
// const char kSpeedQuery[] = "speed";
// const char kReadQuery[] = "read";
// const char kBatteryQuery[] = "battery";
const char kStartCommand[] = "w axis0.controller.vel_ramp_enable 1\n"
  "w axis0.requested_state 5\n";
const char kStopCommand[] = "w axis0.requested_state 1\n";
const char kSpeedCommand[] = "w axis0.controller.vel_ramp_target %d\n";
const char kReadCommand[] = "r axis0.sensorless_estimator.vel_estimate\n";
const char kBatteryCommand[] = "r vbus_voltage\n";
const char kBatteryError[] = "BATTERY LOW\n";

long latitude = 0;
long longitude = 0;
long altitude = -1;
uint8_t voltage = 0;
long rate = 0;

bool heating = false;
const int HEAT_PWM_MUL = 1;
int heat_pwm = 0;

Uart SerialS6C(&sercom1, SRAD_RX, SRAD_TX, SERCOM_RX_PAD_0, UART_TX_PAD_2);
void SERCOM1_Handler()
{
  SerialS6C.IrqHandler();
}

SSIradio S6C;

Uart SerialODrive(&sercom5, ODRIVE_RX, ODRIVE_TX, SERCOM_RX_PAD_3, UART_TX_PAD_2);
void SERCOM5_Handler()
{
  SerialODrive.IrqHandler();
}
// #define SerialODrive Serial

void initHeater() {
  PORT->Group[0].DIR.reg |= PORT_PA28;
  //PORT->Group[0].DIRSET.reg = (1UL << (28 % 32));
  //PORT->Group[0].PINCFG[0].reg = (uint8_t)(PORT_PINCFG_INEN);
}

//true -> on, false -> off
void toggleHeater(bool state) {
  if (state) {
    PORT->Group[0].OUTSET.reg = (1UL << (28 % 32));
  } else {
    PORT->Group[0].OUTCLR.reg = (1UL << (28 % 32));
  }
}


bool CUTTING_DOWN = false;
void receiveMsg(char* msg) {
  //SerialUSB.println("Hello from receive message");
  //SerialUSB.println(micros());
  //SerialUSB.println(msg);

  size_t i = 1; // message start index

  // spin control
  if (!strncmp(msg+1, kSpinQuery, strlen(kSpinQuery))) {
    char * pEnd;
    long oldRate = rate;
    rate = strtol(msg+i+5, &pEnd, 10);
    SerialUSB.println(rate);

    sprintf(buf, kSpeedCommand, rate);
    cmdODrive(buf);

    if (oldRate == 0 && rate > 0) {
      strncpy(buf, kStartCommand, strlen(kStartCommand) + 1);
    } else if(rate == 0) {
      strncpy(buf, kStopCommand, strlen(kStopCommand) + 1);
    }

    cmdODrive(buf);

    strncpy(buf, kReadCommand, strlen(kReadCommand) + 1);
    cmdODrive(buf);
  } else if (!strncmp(msg+1, kCutdownQuery, strlen(kCutdownQuery))) {
    CUTTING_DOWN = true;
    digitalWrite(CUTDOWN1, LOW);
    SerialUSB.println("cuttingdown!");
    digitalWrite(LED, HIGH);
    S6C.tx("Goodbye, sky...");
    delay(5000);
    S6C.tx("Goodbye, sky...");
  } else if (!strncmp(msg+1, kHeatQuery, strlen(kHeatQuery))) {
    heating = true;
    SerialUSB.println("heating");
  } else if (!strncmp(msg+1, kCoolQuery, strlen(kCoolQuery))) {
    heating = false;
    SerialUSB.println("cooling");
  } else if (!strncmp(msg+1, kResetQuery, strlen(kResetQuery))) {
    cmdODrive("sb");
    SerialUSB.println("resetting");
  } else if (msg[1] == 'i') {
    S6C.tx(msg + 2);
    delay(1000);
    S6C.tx(msg + 2);
  }

}

void cmdODrive(char* buf){
  size_t buf_len = strlen(buf);
  SerialODrive.write(buf, buf_len);
  SerialUSB.println(buf);
}

TinyGPSPlus gps;
Uart SerialGPS(&sercom0, SGPS_RX, SGPS_TX, SERCOM_RX_PAD_2, UART_TX_PAD_0);
//#define Serial1 SerialGPS // oh god no

void SERCOM0_Handler(void) {
  SerialGPS.IrqHandler();
}

void setup(){
  initHeater();
  toggleHeater(false);
  
  SerialUSB.begin(115200);
  pinMode(LED, OUTPUT);

  pinMode(CUTDOWN1, OUTPUT);
  pinMode(RECOVERY, OUTPUT);
  digitalWrite(CUTDOWN1, HIGH); // OFF
  digitalWrite(RECOVERY, HIGH); // OFF

  S6C.set_callback(receiveMsg);
  S6C.begin(9600, &SerialS6C);
  digitalWrite(LED, HIGH);
  while (!S6C) {
      delay(100);
      digitalWrite(LED, HIGH);
      delay(100);
      digitalWrite(LED, LOW);
  };

  pinPeripheral(SGPS_RX, PIO_SERCOM);
  pinPeripheral(SGPS_TX, PIO_SERCOM);
  SerialGPS.begin(9600);

  //delay(10000);
  SerialUSB.print(F("TinyGPS++ library v. "));
  SerialUSB.println(TinyGPSPlus::libraryVersion());
  SerialUSB.println();

  SerialODrive.begin(115200);
}



long lasttime = 0;
int version = 0;

void loop() {

  if (CUTTING_DOWN && altitude < 5000) digitalWrite(RECOVERY, LOW);

  if (!heating) {
    toggleHeater(false);
  } else {
    if (heat_pwm % HEAT_PWM_MUL == 0) {
      toggleHeater(true);
    } else {
      toggleHeater(false);
    }
  }
  ++heat_pwm;
  
  // latitude = (random(360)-180)*1000 + 39425000;
  // longitude = (random(360)-180)*1000 -168007860;
  // altitude = random(412500); // 412499;
  voltage = 248 + random(8);

  //SerialODrive.println("doot");
  //SerialUSB.println("doot");
  //S6C.tx("doot");

  S6C.rx();

  if (SerialGPS.available() > 0) {
    //SerialUSB.println(SerialGPS.read());
    if (gps.encode(SerialGPS.read())) {
      updateLatLongAlt();
    }
  }

  if(SerialODrive.available()) {
    size_t buf_len = SerialODrive.readBytes(rateBuf, 10);
    S6C.tx(rateBuf, buf_len);
  }

  if(millis() - lasttime > 1000){
    digitalWrite(LED, HIGH);
  }
  if (millis() - lasttime > 5000) {
    lasttime = millis();
    version++;
    if (version >= 4) version = 0; // TODO: why does every other packet get dropped? even if we set the delay to like 10s...

    digitalWrite(LED, LOW);
    sendCoords(version);
  }

  //digitalWrite(LED, LOW);
  //SerialUSB.println("doot");

  // This sketch displays information every time a new sentence is correctly encoded.

  //delay(50);
  // if (SerialGPS.available() > 0) {
  //   //SerialUSB.println(SerialGPS.read());
  //   if (gps.encode(SerialGPS.read())) {
  //     displayInfo();
  //     delay(50);
  //   }
  // }

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


void updateLatLongAlt()
{
  if (gps.location.isValid())
    {
      latitude = gps.location.lat()*1000000;
      longitude = gps.location.lng()*1000000;
    }

  if(gps.altitude.isValid()){
    altitude = gps.altitude.feet();
  }
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

uint8_t batteryCheck(){
  //SerialUSB.println("Battery check!");
  SerialODrive.write(kBatteryCommand, strlen(kBatteryCommand));

  int i = 0;
  while(!SerialODrive.available() && i < 100) {
    //Serial.println("Waiting!");
    delay(10);
    i++;
  }

  SerialODrive.readBytes(buf, 64U);
  SerialUSB.println(buf);
  float voltage;
  sscanf(buf, "%f\n", &voltage);

  SerialUSB.println(voltage*4.0);

  //SerialUSB.println((uint8_t) (voltage*4.0));

  return (uint8_t) (voltage*4.0);

  // SerialODrive.readBytes(buf, 64U);
  // float voltage;
  // sscanf(buf, "%f\n", &voltage);
  // SerialUSB.println(voltage);
}

void sendCoords(int version) {
  char out[1024];
  switch (version) {
  case 0:
  case 1:
    snprintf(out, 1024, "[%ld %ld]", latitude, longitude);
    break;

  case 2:
  case 3:
    snprintf(out, 1024, "(%ld %d)", altitude, voltage);
    break;
  }
  S6C.tx(out);
}
