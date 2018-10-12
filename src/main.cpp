#include <SPI.h>
#include <SD.h>
#include <DFRobot_BMP388_SPI.h>
#include <Adafruit_BNO055.h>
#include "wiring_private.h"

#define LED_PIN SCL
#define BMP_CS 34
#define SD_CS 5
#define ACCEL_CS 13
#define GYRO_CS 3
#define ADXL_CS 0

void setup() {

  PORT->Group[0].DIR.reg = PORT_PA27;
  PORT->Group[0].OUTSET.reg = (1UL << (27 % 32));
  //COMMEMNT OUT B4 LAUNCH
  PORT->Group[0].DIR.reg |= PORT_PA28;
  PORT->Group[0].OUTSET.reg = (1UL << (28 % 32));
  
  pinMode(LED_PIN, OUTPUT);
  //pinMode(SD_CS, OUTPUT);
  pinMode(ACCEL_CS, OUTPUT);
  //pinMode(BMP_CS, OUTPUT);
  pinMode(ADXL_CS, OUTPUT);
  pinMode(GYRO_CS, OUTPUT);
  
  //digitalWrite(SD_CS, LOW);
  //digitalWrite(BMP_CS, HIGH);
  digitalWrite(ACCEL_CS, HIGH);
  digitalWrite(GYRO_CS, HIGH);
  digitalWrite(ADXL_CS, HIGH);
  SerialUSB.begin(9600);
  DFRobot_BMP388_SPI bmp(BMP_CS);
  bmp.begin();
  SD.begin(SD_CS);
  
  Adafruit_BNO055 bno = Adafruit_BNO055(17, 18);
  
  pinPeripheral(17, PIO_SERCOM_ALT);
  pinPeripheral(18, PIO_SERCOM_ALT);

  //pinMode(17, OUTPUT);
  
  /*while (true) {
    digitalWrite(17, HIGH);
    PORT->Group[0].OUTSET.reg = (1UL << (27 % 32));
    delay(50);
    digitalWrite(17, LOW);
    delay(50);
    }*/
  
  if (!bno.begin()) {
    while (true) {
      SerialUSB.println("BNO not working :(");
      delay(100);
    }
  }
  
  while (true) {

    SerialUSB.println("HEATER DISABLED. ENABLE ME");

    digitalWrite(LED_PIN, HIGH);
    delay(300);
    digitalWrite(LED_PIN, LOW);
    SerialUSB.print("Pressure: ");
    
    SerialUSB.println(bmp.readPressure());

    /*imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    SerialUSB.print("X: ");
    SerialUSB.print(euler.x());
    SerialUSB.print(" Y: ");
    SerialUSB.print(euler.y());
    SerialUSB.print(" Z: ");
    SerialUSB.print(euler.z());
    SerialUSB.print("\t\t");*/

    File myFile;

    /*SerialUSB.print("Initializing SD card...");

    if (!SD.begin(SD_CS)) {
      SerialUSB.println("initialization failed!");
    }
    SerialUSB.println("initialization done.");*/

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
    }

    SerialUSB.println("......");
    
    delay(200);
    
  }
}

void loop() {  
}
