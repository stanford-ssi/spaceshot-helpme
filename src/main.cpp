#include <SPI.h>
#include <SD.h>
#include <DFRobot_BMP388_SPI.h>
#include <wiring_private.h>
#include <NMEAGPS.h>
#include <GPSport.h>
#include "min.h"
#include "bma2x2_support.hpp"
#include "bmg160_support.hpp"

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

void do_cutdown(){
  digitalWrite(FET1,LOW);
  digitalWrite(FET2,LOW);
  digitalWrite(FET3,LOW);
}

void min_application_handler(uint8_t min_id, uint8_t *min_payload, uint8_t len_payload, uint8_t port) {
  min_payload++;
  len_payload--;
  SerialUSB.write(min_payload,len_payload);
  SerialUSB.println();
  
  if(min_payload[0]=='H' && min_payload[1]=='E'){
    SerialUSB.println("CUTTTTTDOWNWNWNW");
    do_cutdown();
  }
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

  SerialS6C.begin(9600);
  pinPeripheral(SRAD_RX, PIO_SERCOM);
  pinPeripheral(SRAD_TX, PIO_SERCOM);
  
  pinMode(LED_PIN, OUTPUT);
  pinMode(SD_CS, OUTPUT);
  pinMode(ACCEL_CS, OUTPUT);
  pinMode(BMP_CS, OUTPUT);
  pinMode(ADXL_CS, OUTPUT);
  pinMode(GYRO_CS, OUTPUT);
  pinMode(BMP2_CS,OUTPUT);

  pinMode(FET1, OUTPUT);
  pinMode(FET2, OUTPUT);
  pinMode(FET3, OUTPUT);
  pinMode(HEATER, OUTPUT);//DONT FORGET!

  digitalWrite(FET1,HIGH);
  digitalWrite(FET2,HIGH);
  digitalWrite(FET3,HIGH);
  digitalWrite(HEATER,HIGH);

  digitalWrite(BMP2_CS,HIGH);
  digitalWrite(SD_CS, HIGH);
  digitalWrite(BMP_CS, HIGH);
  digitalWrite(ACCEL_CS, HIGH);
  digitalWrite(GYRO_CS, HIGH);
  digitalWrite(ADXL_CS, HIGH);

  SerialUSB.begin(115200);

  DFRobot_BMP388_SPI bmp(BMP_CS);
  //DFRobot_BMP388_SPI bmp(BMP2_CS); //This stupid library dosen't store the CS pins seperately...

  min_init_context(&min_ctx_s6c, 0);

  bmp.begin();
  SD.begin(SD_CS);

  NMEAGPS gps;
  gps_fix fix;

    delay(2000);

    char serial_buffer_S6C[32];

    while(true){
      int available = SerialS6C.available();
		  if (available > 0) {
				if (available > 32) available = 32;
				size_t buf_len = SerialS6C.readBytes(serial_buffer_S6C, available);
				min_poll(&min_ctx_s6c, (uint8_t*)serial_buffer_S6C, (uint8_t)buf_len);
      }
    }

      bmg160_data_readout_template();
      bma2x2_data_readout_template();
      
      /*delay(100); //GPS is glitchy...
      if (gps.available(SerialGPS)) {
        fix = gps.read();
        SerialUSB.print(" Alt:");
        SerialUSB.print(fix.altitude());
        SerialUSB.print(" Lat:");
        printL(SerialUSB,fix.latitudeL());
        SerialUSB.print(" Lon:");
        printL(SerialUSB,fix.longitudeL());
      }
      */

      SerialUSB.print(" P:");
      SerialUSB.print(bmp.readPressure());
      SerialUSB.println("HEATER DISABLED. ENABLE ME");
      SerialUSB.println();
      delay(100);
    }
    

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
    


void loop() {  
}
