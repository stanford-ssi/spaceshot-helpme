#include <SPI.h>
#include <SD.h>
#include <DFRobot_BMP388_SPI.h>
#include <wiring_private.h>
#include "min.h"
#include "bma2x2_support.hpp"
#include "bmg160_support.hpp"
#include "RadioInterface.h"
#include "TinyGPS++.h"

#define LED_PIN SCL
#define BMP_CS 12
#define BMP2_CS 26
#define SD_CS 5
#define ACCEL_CS 13
#define GYRO_CS 3
#define ADXL_CS 0

#define FET1 14
#define FET2 30
#define FET3 31
#define HEATER 27

#define SRAD_TX 11
#define SRAD_RX 10
#define SGPS_TX 4
#define SGPS_RX 1

#define MODE_RECEIVING 1
#define MODE_TRANSMITTING 2

void PrintHex8(uint8_t *data, uint8_t length) // prints 16-bit data in hex with leading zeroes
{
       char tmp[8];
       for (int i=0; i<length; i++)
       { 
         sprintf(tmp, "0x%.2X",data[i]); 
         SerialUSB.print(tmp); SerialUSB.print(" ");
       }
       SerialUSB.println();
}

Uart SerialS6C(&sercom1, SRAD_RX, SRAD_TX, SERCOM_RX_PAD_2, UART_TX_PAD_0);
Uart SerialGPS(&sercom2, SGPS_RX, SGPS_TX, SERCOM_RX_PAD_2, UART_TX_PAD_0);

void SERCOM1_Handler()
{
  SerialS6C.IrqHandler();
}

void SERCOM2_Handler()
{
  SerialGPS.IrqHandler();
}

void do_cutdown()
{
  //This fires Q3 and Q?
  digitalWrite(FET1, LOW);
  digitalWrite(FET2, LOW);
}

void do_main()
{
  digitalWrite(FET3, LOW);
  //This fires Q2
}

void min_application_handler(uint8_t min_id, uint8_t *min_payload, uint8_t len_payload, uint8_t port)
{
  min_payload++;
  len_payload--;

  if (min_payload[0] == 'H' && min_payload[1] == 'E')
  {
    SerialUSB.println("CUTTTTTDOWNWNWNW");
    do_cutdown();
  }
  else if (min_payload[0] == 'M' && min_payload[1] == 'E')
  {
    SerialUSB.println("MAIN!!!!!");
    do_main();
  }
}

uint16_t min_tx_space(uint8_t port)
{
  uint16_t n = 1;
  if (port == 0)
    n = SerialS6C.availableForWrite();
  return n;
}

void min_tx_byte(uint8_t port, uint8_t byte)
{
  if (port == 0)
    SerialS6C.write(&byte, 1U);
}

uint32_t min_time_ms()
{
  return millis();
}

void min_tx_start(uint8_t port) {}

void min_tx_finished(uint8_t port) { SerialS6C.flush(); }

struct min_context min_ctx_s6c;

struct bma2x2_t bma;

uint32_t last_tx = millis();
uint32_t last_log = millis();
uint32_t start = millis();
uint8_t cache = 0;
bool cutdown_state = false;

void setup()
{

  //initialize pins
  pinMode(LED_PIN, OUTPUT);
  pinMode(SD_CS, OUTPUT);
  pinMode(ACCEL_CS, OUTPUT);
  pinMode(BMP_CS, OUTPUT);
  pinMode(ADXL_CS, OUTPUT);
  pinMode(GYRO_CS, OUTPUT);
  pinMode(BMP2_CS, OUTPUT);

  pinMode(FET1,OUTPUT);
  pinMode(FET2,OUTPUT);
  pinMode(FET3,OUTPUT);
  pinMode(HEATER,OUTPUT);

  digitalWrite(FET1,HIGH);
  digitalWrite(FET2,HIGH);
  digitalWrite(FET3,HIGH);
  digitalWrite(HEATER,LOW);//DONT FORGET!

  digitalWrite(BMP2_CS, HIGH);
  digitalWrite(SD_CS, HIGH);
  digitalWrite(BMP_CS, HIGH);
  digitalWrite(ACCEL_CS, HIGH);
  digitalWrite(GYRO_CS, HIGH);
  digitalWrite(ADXL_CS, HIGH);

  // initialize USB connection
  SerialUSB.begin(9600);

  // init S6C BOI
  SerialS6C.begin(9600);
  min_init_context(&min_ctx_s6c, 0);
  pinPeripheral(SRAD_RX, PIO_SERCOM);
  pinPeripheral(SRAD_TX, PIO_SERCOM);

  // initialize SD card
  SD.begin(SD_CS);
  char fname[32];
  randomSeed(micros());
  sprintf(fname, "%07d.csv", random(1000000));
  File logFile = SD.open(fname, FILE_WRITE | O_CREAT);

  if (logFile)
  {
    logFile.println("millis,altitude_bmp,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,"
                    "temperature,latitude,longitude,altitude_gps,time");
  }

  // initialize GPS
  TinyGPSPlus gps;

  SerialGPS.begin(9600);
  pinPeripheral(SGPS_RX, PIO_SERCOM_ALT);
  pinPeripheral(SGPS_TX, PIO_SERCOM_ALT);

  // initialize pressure sensor
  DFRobot_BMP388_SPI bmp(BMP_CS);
  bmp.begin();
  // DFRobot_BMP388_SPI bmp(BMP2_CS); //This stupid library dosen't store the CS pins seperately...

  // initialize accelerometer and gyroscope
  bma2x2_init_accel();
  bmg160_init_gyro();

  // wait for sensors to boot
  delay(2000);

  struct bma2x2_accel_data_temp accel_data;
  struct bmg160_data_t gyro_data;

  char serial_buffer_S6C[32];

  while (true)
  {

    int available = SerialS6C.available();
    if (available > 0)
    {
      if (available > 32)
        available = 32;
      size_t buf_len = SerialS6C.readBytes(serial_buffer_S6C, available);
      min_poll(&min_ctx_s6c, (uint8_t *)serial_buffer_S6C, (uint8_t)buf_len);
    }

    if (millis() > last_log + 5)
    { //blink off every 5 milliseconds
      digitalWrite(LED_PIN, LOW);
    }

    if (millis() > last_log + 10) //log every 10 milliseconds
    {
      last_log = millis();
      SerialUSB.println("Logging...");

      float bmp_pres = bmp.readPressure();
      float bmp_alt = bmp.readAltitude();
      accel_data = bma2x2_read_accel();
      gyro_data = bmg160_read_gyro();
      float temp = bmp.readTemperature();

      while (SerialGPS.available() > 0)
        gps.encode(SerialGPS.read());

      if (logFile)
      {
        SerialUSB.println("Found file...");
        logFile.print(millis());
        logFile.print(",");

        logFile.print(bmp_pres);
        logFile.print(",");

        logFile.print(accel_data.x);
        logFile.print(",");
        logFile.print(accel_data.y);
        logFile.print(",");
        logFile.print(accel_data.z);
        logFile.print(",");

        logFile.print(gyro_data.datax);
        logFile.print(",");
        logFile.print(gyro_data.datay);
        logFile.print(",");
        logFile.print(gyro_data.dataz);
        logFile.print(",");

        logFile.print(temp*100);
        logFile.print(",");

        logFile.print(long(gps.location.lat()*1000000));
        logFile.print(",");
        logFile.print(long(gps.location.lng()*1000000));
        logFile.print(",");
        logFile.print(gps.altitude.meters());
        logFile.print(",");
        logFile.print(gps.time.value());
        logFile.println(",");

        //logFile.flush();
        cache++;
        digitalWrite(LED_PIN, HIGH);
      }

      if(cache > 5){
        cache = 0;
        logFile.flush();
      }

      if (millis() > last_tx + 5000)
      {
        last_tx = millis();
        const int msg_len = sizeof(int32_t) * 3;
        uint8_t msg[msg_len + 2];
        msg[0] = MESSAGE_SEND;
        msg[1] = msg_len;



        ((int32_t *)(msg + 2))[0] = long(gps.location.lat()*1000000);
        ((int32_t *)(msg + 2))[1] = long(gps.location.lng()*1000000);
        ((int32_t *)(msg + 2))[2] = bmp_alt;
        
        PrintHex8(msg, msg_len+2);

        min_send_frame(&min_ctx_s6c, 0, msg, msg[1] + 2);
      }

      if(((long)millis()) > 2.5*60*60*1000){
        cutdown_state = true;
        SerialUSB.println("Timer ran out!");
        do_cutdown();
      }

      if(cutdown_state && bmp_alt < 500.0){
        SerialUSB.println("Low Enough!");
        do_main();
      }

      SerialUSB.print("LAT: ");
      SerialUSB.print(long(gps.location.lat()*1000000));
      SerialUSB.print(" LON: ");
      SerialUSB.print(long(gps.location.lng()*1000000));
      SerialUSB.print(" ALT: ");
      SerialUSB.print(bmp_alt);
      SerialUSB.print(" TIME: ");
      SerialUSB.print(gps.time.value());
      SerialUSB.print(" Countdown: ");
      SerialUSB.print(((long) millis())-2.5*60*60*1000);
      SerialUSB.println();
      
    }
  }
}

void loop() {}
