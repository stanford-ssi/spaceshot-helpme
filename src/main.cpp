#include <SPI.h>
#include <SD.h>
#include <DFRobot_BMP388_SPI.h>
#include <wiring_private.h>
#include <NMEAGPS.h>
#include "min.h"
//#include "bma2x2_support.hpp"
#include "bmg160_support.hpp"

#define LED_PIN SCL
#define BMP_CS 34
#define BMP2_CS 26
#define SD_CS 5
#define ACCEL_CS 13
#define GYRO_CS 3
#define ADXL_CS 0

#define SRAD_TX 10
#define SRAD_RX 11
#define SGPS_TX 4
#define SGPS_RX 1

#define MODE_RECEIVING 1
#define MODE_TRANSMITTING 2

Uart SerialS6C(&sercom1, SRAD_RX, SRAD_TX, SERCOM_RX_PAD_0, UART_TX_PAD_2);
Uart SerialGPS(&sercom2, SGPS_RX, SGPS_TX, SERCOM_RX_PAD_2, UART_TX_PAD_0);
 
void SERCOM1_Handler()
{
  SerialS6C.IrqHandler();
}

void SERCOM2_Handler()
{
  SerialGPS.IrqHandler();
}

uint16_t min_tx_space(uint8_t port) {
	uint16_t n = 1;
	if (port == 0) n = SerialS6C.availableForWrite();
	return n;
}

void min_tx_byte(uint8_t port, uint8_t byte) {
	if (port == 0) SerialS6C.write(&byte, 1U);
}

uint32_t min_time_ms() {
  return millis();
}

void min_tx_start(uint8_t port) {}

void min_tx_finished(uint8_t port) { SerialS6C.flush(); }

struct min_context min_ctx_s6c;

void setup() {
  SerialGPS.begin(9600);
  pinPeripheral(SGPS_RX, PIO_SERCOM_ALT);
  pinPeripheral(SGPS_TX, PIO_SERCOM_ALT);
  
  PORT->Group[0].DIR.reg = PORT_PA27;
  PORT->Group[0].OUTSET.reg = (1UL << (27 % 32));
  //COMMEMNT OUT B4 LAUNCH
  PORT->Group[0].DIR.reg |= PORT_PA28;
  PORT->Group[0].OUTSET.reg = (1UL << (28 % 32));
  
  pinMode(LED_PIN, OUTPUT);
  pinMode(SD_CS, OUTPUT);
  pinMode(ACCEL_CS, OUTPUT);
  pinMode(BMP_CS, OUTPUT);
  pinMode(ADXL_CS, OUTPUT);
  pinMode(GYRO_CS, OUTPUT);
  pinMode(BMP2_CS,OUTPUT);

  digitalWrite(BMP2_CS,HIGH);
  digitalWrite(SD_CS, HIGH);
  digitalWrite(BMP_CS, HIGH);
  digitalWrite(ACCEL_CS, HIGH);
  digitalWrite(GYRO_CS, HIGH);
  digitalWrite(ADXL_CS, HIGH);

  SerialUSB.begin(9600);

  DFRobot_BMP388_SPI bmp(BMP_CS);
  DFRobot_BMP388_SPI bmp2(BMP2_CS);

  bmp.begin();
  bmp2.begin();
  SD.begin(SD_CS);

  NMEAGPS gps;
  gps_fix fix;
  while (true) {
    /* if (gps.available(SerialGPS)) {
      SerialUSB.print(fix.longitude());
    }
    delay(100); */

    delay(2000);
    //bma2x2_data_readout_template();
    SerialUSB.println("Test1");
    bmg160_data_readout_template();
    // SerialUSB.println("HEATER DISABLED. ENABLE ME");

    //
    //delay(300);
    //

    // SerialUSB.print("Pressure: ");
    // SerialUSB.println(bmp.readPressure());
    

    /*File myFile;
    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
    if (SD.remove("test.txt")) SerialUSB.println("Removed file");
    delay(100);
    myFile = SD.open("test.txt", FILE_WRITE);

    // if the file opened okay, write to it:
    if (myFile) {
      SerialUSB.print("Writing to test.txt...");
      myFile.println("testing 1, 2, 3.");
      // close the file:
      myFile.close();
      SerialUSB.println("done.");
    } else {
      // if the file didn't open, print an error:
      SerialUSB.println("error opening test.txt");
    }

    // re-open the file for reading:
    myFile = SD.open("test.txt");
    if (myFile) {
      SerialUSB.println("test.txt:");

      // read from the file until there's nothing else in it:
      while (myFile.available()) {
	SerialUSB.write(myFile.read());
      }
      // close the file:
      myFile.close();
    } else {
      // if the file didn't open, print an error:
      SerialUSB.println("error opening test.txt");
    }*/
    /*
    if (SerialGPS.available() > 0)
      SerialUSB.write(SerialGPS.read());
    delay(1);
    */
  }
}

void loop() {  
}
