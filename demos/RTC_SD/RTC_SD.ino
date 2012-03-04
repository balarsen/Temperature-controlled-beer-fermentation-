
#include <Wire.h>
#include "RTClib.h"
#include <SD.h>


///////////////////////////////////////////////
////// constants
///////////////////////////////////////////////
const int chipSelect = 10;    
// the digital pins that connect to the LEDs
const int redLEDpin = 3;    
const int greenLEDpin = 4;    
const unsigned long SEC1=1000;
const unsigned long SEC3=3000;
const unsigned long SEC5=5000;

///////////////////////////////////////////////
////// globals 
///////////////////////////////////////////////
RTC_DS1307 RTC;
Sd2Card card;
SdVolume volume;
SdFile root;
int ledStateGreen = LOW;             // ledState used to set the LED
int ledStateRed   = LOW;             // ledState used to set the LED
unsigned long trigger_millis=0; 


void formatTimeDigits(char strOut[3], int num)
{
  strOut[0] = '0' + (num / 10);
  strOut[1] = '0' + (num % 10);
  strOut[2] = '\0';
}

String ISOTIME(DateTime inval) {
  /* Format the time and date and insert into the temporary buffer */
  // 2012-03-02T20:14:00
  char strOut[3];
  String outval;
  outval += String(inval.year());
  outval += "-";
  formatTimeDigits(strOut, inval.month());
  outval += String(strOut); 
  outval += "-";
  formatTimeDigits(strOut, inval.day());
  outval += String(strOut); 
  outval += "T"; 
  formatTimeDigits(strOut, inval.hour());
  outval += String(strOut);
  outval += ":";
  formatTimeDigits(strOut, inval.minute());
  outval += String(strOut);
  outval += ":";
  formatTimeDigits(strOut, inval.second());
  outval += String(strOut);
  return outval;
}


void setup () {
  delay(3000);  // boot up delay
  Serial.begin(19200);
  pinMode(greenLEDpin, OUTPUT);    
  pinMode(redLEDpin, OUTPUT);        
  
  Wire.begin();
  RTC.begin();

  if (! RTC.isrunning()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    RTC.adjust(DateTime(__DATE__, __TIME__));
  }
  Serial.print("\nInitializing SD card...");
  // On the Ethernet Shield, CS is pin 4. It's set as an output by default.
  // Note that even if it's not used as the CS pin, the hardware SS pin 
  // (10 on most Arduino boards, 53 on the Mega) must be left as an output 
  // or the SD library functions will not work. 
  pinMode(10, OUTPUT);     // change this to 53 on a mega
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }

//  // we'll use the initialization code from the utility libraries
//  // since we're just testing if the card is working!
  if (!card.init(SPI_HALF_SPEED, chipSelect)) {
    Serial.println("initialization failed. Things to check:");
    Serial.println("* is a card is inserted?");
    Serial.println("* Is your wiring correct?");
    Serial.println("* did you change the chipSelect pin to match your shield or module?");
    return;
  } else {
   Serial.println("Wiring is correct and a card is present."); 
  }

  // print the type of card
  Serial.print("\nCard type: ");
  switch(card.type()) {
    case SD_CARD_TYPE_SD1:
      Serial.println("SD1");
      break;
    case SD_CARD_TYPE_SD2:
      Serial.println("SD2");
      break;
    case SD_CARD_TYPE_SDHC:
      Serial.println("SDHC");
      break;
    default:
      Serial.println("Unknown");
  }

  // Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32
  if (!volume.init(card)) {
    Serial.println("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card");
    return;
  }


  // print the type and size of the first FAT-type volume
  uint32_t volumesize;
  Serial.print("\nVolume type is FAT");
  Serial.println(volume.fatType(), DEC);
  Serial.println();
  
  volumesize = volume.blocksPerCluster();    // clusters are collections of blocks
  volumesize *= volume.clusterCount();       // we'll have a lot of clusters
  volumesize *= 512;                            // SD card blocks are always 512 bytes
  volumesize /= 1024;
  Serial.print("Volume size (Mbytes): ");
  volumesize /= 1024;
  Serial.println(volumesize);
  
  root.openRoot(volume);
  root.close();
} 

File fp;
  
  
void loop () {
    if (millis() >= trigger_millis) {
      trigger_millis = millis() + SEC5;
      DateTime now = RTC.now();
      Serial.print(ISOTIME(now));
      Serial.print("\t");
      Serial.println(millis());
  
      fp = SD.open("datafile.txt", FILE_WRITE);
      if (fp.print(ISOTIME(now)) == 0)
        Serial.println("got zero");
      fp.print("\t");
      fp.println(millis());
      fp.close();
    }    

}
