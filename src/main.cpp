#include <SD.h>
#include <DFRobot_BMP388_SPI.h>

#define LED_PIN SCL
#define BMP_CS 34
#define SD_CS 5

void setup() {

  PORT->Group[0].DIR.reg = PORT_PA23;
  SerialUSB.begin(9600);
  //DFRobot_BMP388_SPI bmp(BMP_CS);
  //bmp.begin();
  //int ret = SD.begin(SD_CS);
  
  while (true) {

    PORT->Group[0].OUTSET.reg = (1UL << (23 % 32));
    //digitalWrite(LED_PIN, HIGH);
    delay(300);
    PORT->Group[0].OUTCLR.reg = (1UL << (23 % 32));
    //SerialUSB.println(bmp.readPressure());
    //digitalWrite(LED_PIN, LOW);
    //SerialUSB.println(bmp.readPressure());

    /*File myFile = SD.open("test.txt", FILE_WRITE);

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
    SerialUSB.println(ret);

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

    Sd2Card card;
    SdVolume volume;
    SdFile root;

    if (!card.init(SPI_HALF_SPEED, SD_CS)) {
      SerialUSB.println("initialization failed. Things to check:");
      SerialUSB.println("* is a card inserted?");
      SerialUSB.println("* is your wiring correct?");
      SerialUSB.println("* did you change the chipSelect pin to match your shield or module?");
    } else {
      SerialUSB.println("Wiring is correct and a card is present.");
    }

    // print the type of card
    SerialUSB.println();
    SerialUSB.print("Card type:         ");
    switch (card.type()) {
    case SD_CARD_TYPE_SD1:
      SerialUSB.println("SD1");
      break;
    case SD_CARD_TYPE_SD2:
      SerialUSB.println("SD2");
      break;
    case SD_CARD_TYPE_SDHC:
      SerialUSB.println("SDHC");
      break;
    default:
      SerialUSB.println("Unknown");
    }

    // Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32
    if (!volume.init(card)) {
      SerialUSB.println("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card");
    }

    SerialUSB.print("Clusters:          ");
    SerialUSB.println(volume.clusterCount());
    SerialUSB.print("Blocks x Cluster:  ");
    SerialUSB.println(volume.blocksPerCluster());

    SerialUSB.print("Total Blocks:      ");
    SerialUSB.println(volume.blocksPerCluster() * volume.clusterCount());
    SerialUSB.println();

    // print the type and size of the first FAT-type volume
    uint32_t volumesize;
    SerialUSB.print("Volume type is:    FAT");
    SerialUSB.println(volume.fatType(), DEC);

    volumesize = volume.blocksPerCluster();    // clusters are collections of blocks
    volumesize *= volume.clusterCount();       // we'll have a lot of clusters
    volumesize /= 2;                           // SD card blocks are always 512 bytes (2 blocks are 1KB)
    SerialUSB.print("Volume size (Kb):  ");
    SerialUSB.println(volumesize);
    SerialUSB.print("Volume size (Mb):  ");
    volumesize /= 1024;
    SerialUSB.println(volumesize);
    SerialUSB.print("Volume size (Gb):  ");
    SerialUSB.println((float)volumesize / 1024.0);

    SerialUSB.println("\nFiles found on the card (name, date and size in bytes): ");
    root.openRoot(volume);

    // list all files in the card with date and size
    root.ls(LS_R | LS_DATE | LS_SIZE);
    
    delay(100);
  }
}

void loop() {  
}
