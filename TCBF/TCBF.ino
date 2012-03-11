#include <SD.h>
#include "RTClib.h"

// include the library code:
#include <Wire.h>
#include <Adafruit_MCP23017.h>
#include <Adafruit_RGBLCDShield.h>

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

#define ECHO_TO_SERIAL 1

// the digital pins that connect to the LEDs
#define redLEDpin 2
#define greenLEDpin 3

// These #defines make it easy to set the backlight color
#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7

// the ranges are
#define GOOD GREEN
#define CLOSE YELLOW
#define BAD RED
#define CLOSERNG 5
#define BADRNG 10

// the TEMP defines
#define TEMP_RNG 2
#define COOLING 1
#define HEATING 2
#define STATIC 0
#define HEATING_PIN 4
#define COOLING_PIN 5
uint8_t state=STATIC;

// how many milliseconds between checking the temp data and logging it. 
#define LOG_INTERVAL  5000 
#define TEMP_INTERVAL 500
// how many milliseconds before writing the logged data permanently to disk
// set it to 10*LOG_INTERVAL to write all data every 10 datareads, you could lose up to 
// the last 10 reads if power is lost but it uses less power and is much faster!
#define SYNC_INTERVAL 10
uint32_t lastLogMillis=0;
uint32_t lastReadMillis=0;
uint8_t sync_cnt=0;

// The analog pins that connect to the sensors
#define tempPin 1                // analog 1
#define aref_voltage 3.3         // we tie 3.3V to ARef and measure it with a multimeter!

// for the data logging shield, we use digital pin 10 for the SD cs line
const int chipSelect = 10;

// the logging file
File logfile;

const char* BLANK={"                "};

uint8_t tempPt = 70;

RTC_DS1307 RTC; // define the Real Time Clock object

void error(char *str)
{
  Serial.print("error: ");
  Serial.println(str);
  // red LED indicates error
  digitalWrite(redLEDpin, HIGH);
  lcd.clear();
  lcd.print("   !!Error!!");
  lcd.setBacklight(RED);
  while(1);
}

void setup() {
  // We are going to use the 3.3V ref
  analogReference(EXTERNAL);
  ////////////////////////////////
  /// SETUP THE SD CARD
  ///////////////////////////////
  // use debugging LEDs
  pinMode(redLEDpin, OUTPUT);
  pinMode(greenLEDpin, OUTPUT);
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(chipSelect, OUTPUT);
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    error("Card failed, or not present");
  }
  Serial.println("card initialized.");

  //////////////////////////////////
  //  heat cool pin setup
  /////////////////////////////////
  pinMode(HEATING_PIN, OUTPUT);
  pinMode(COOLING_PIN, OUTPUT);
  digitalWrite(HEATING_PIN, LOW); 
  digitalWrite(COOLING_PIN, LOW);

  //////////////////////////////////////
  //  Setup the LCD
  //////////////////////////////////////
  // Debugging output
  Serial.begin(19200);
  // set up the LCD's number of rows and columns: 
  lcd.begin(16, 2);

  int time = millis();
  lcd.setCursor(0, 0);
  lcd.setBacklight(BLUE);
  lcd.print("Welcome to TCBF");
  lcd.setCursor(0, 1);
  lcd.print("  Setting up...");  
  time = millis() - time;
  Serial.print("Took "); Serial.print(time); Serial.println(" ms to setup the LCD");
  // add a bootup delay, If other setup takes a while this delay can go away
  delay(3000);
  //////////////////////////
  // Setup the RTC
  /////////////////////////
  RTC.begin();
  if (! RTC.isrunning()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
  }
  RTC.adjust(DateTime(__DATE__, __TIME__));

  ////////////////////////////
  // files, start one
  ////////////////////////////
    // create a new file
  char filename[] = "LOGGER00.CSV";
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i/10 + '0';
    filename[7] = i%10 + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      logfile = SD.open(filename, FILE_WRITE); 
      break;  // leave the loop!
    }
  }
  if (! logfile) {
    error("couldnt create file");
  }
  
  /////////////////////////////
  // setup the controller temp
  /////////////////////////////
  lcd.clear();
  setTemp();
  printSetTemp();
  //lcd.noDisplay();
  //lcd.setBacklight(0x0);
  lcd.clear();
}

void printSetTemp() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temperature set:");
  lcd.setCursor(0, 1);
  char line[10];
  sprintf(line, "    %3u F", tempPt);
  lcd.setCursor(0, 1);
  lcd.print(line);
  lcd.setBacklight(RED);
  delay(200);
  lcd.setBacklight(BLUE);
  delay(200);
  lcd.setBacklight(RED);
  delay(200);
  lcd.setBacklight(BLUE);
}

void clearLine2() {
  lcd.setCursor(0, 1);
  lcd.print(BLANK);
  lcd.setCursor(0, 1);
}

void displayTempSet() {
Serial.println("In displayTempSet");
  lcd.setCursor(0, 0);
  lcd.print("Set temperature:");
  char line[10];
  sprintf(line, "    %3u F", tempPt);
  lcd.setCursor(0, 1);
  lcd.print(line);
  delay(100);  // slow things down
}


  // which analog pin to connect
#define THERMISTORPIN A0         
// resistance at 25 degrees C
#define THERMISTORNOMINAL 10000      
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25   
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define NUMSAMPLES 5
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3950
// the value of the 'other' resistor
#define SERIESRESISTOR 10000   // TODO measure this 
uint8_t currentTemp() {
  // thanks ladyada!!
  // http://www.ladyada.net/learn/sensors/thermistor.html 
  uint8_t i;
  float average=0; 
  for (i=0; i< NUMSAMPLES; i++) {
     average += analogRead(THERMISTORPIN);
     delay(10);
  }
  average /= NUMSAMPLES;
 
  Serial.print("Average analog reading "); 
  Serial.println(average);
 
  // convert the value to resistance
  average = 1023 / average - 1;
  average = SERIESRESISTOR / average;
  Serial.print("Thermistor resistance "); 
  Serial.println(average);
  float steinhart;
  steinhart = average / THERMISTORNOMINAL;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert to C
  steinhart = (9./5.)*steinhart + 32;
  Serial.print("Temperature "); 
  Serial.print(steinhart);
  Serial.println(" *F");
  return (uint8_t)steinhart;   // a bit of waste to do a real calc then (uint8_t) but for now 
}

uint8_t displayCurrTemp() {
Serial.println("In displayCurrTemp");
  lcd.setCursor(0, 0);
  char line[16];
  uint8_t curTmp;
  curTmp = currentTemp();
  sprintf(line, "Curr T: %3u F", curTmp);
  lcd.print(line);
  lcd.setCursor(0, 1);
  sprintf(line, " Set T: %3u F", tempPt);
  lcd.print(line);
  return(curTmp);
}

void setTemp() {
  while (1) {
    uint8_t buttons = lcd.readButtons();    
    displayTempSet();
    if (buttons) {
      if (buttons & BUTTON_UP) {
        if (tempPt < 100)
          tempPt++;
Serial.println("tempPt++");
      }
      if (buttons & BUTTON_DOWN) {
        if (tempPt > 32)
          tempPt--;
Serial.println("tempPt--");
      }
  // These will be for +/- on the control, fixed for now
  //    if (buttons & BUTTON_LEFT) {
  //    }
  //    if (buttons & BUTTON_RIGHT) {
  //    }
      if (buttons & BUTTON_SELECT) {
        return; // done move on to the controlling
      }
    }  

  }
}

uint8_t setBkgd(uint8_t curTmp) {
  if (abs(curTmp-tempPt) > BADRNG) {
    lcd.setBacklight(BAD);
    return (RED);
  }
  else if (abs(curTmp-tempPt) > CLOSERNG) {
    lcd.setBacklight(CLOSE);
    return (YELLOW);
  }
  else {
    lcd.setBacklight(GOOD);
    return (GREEN); 
  }
}


void writeTime() {
  DateTime now;
  now = RTC.now();
  // log time
  logfile.print(millis());
  logfile.print(", "); 
  logfile.print(now.unixtime()); // seconds since 1/1/1970
  logfile.print(", ");
  logfile.print('"');
  logfile.print(now.month(), DEC);
  logfile.print("/");
  logfile.print(now.day(), DEC);
  logfile.print("/");
  logfile.print(now.year(), DEC);
  logfile.print(" ");
  logfile.print(now.hour(), DEC);
  logfile.print(":");
  logfile.print(now.minute(), DEC);
  logfile.print(":");
  logfile.print(now.second(), DEC);
  logfile.print('"');
#if ECHO_TO_SERIAL
  Serial.print(millis());
  Serial.print(", ");  
  Serial.print(now.unixtime()); // seconds since 1/1/1970
  Serial.print(", ");
  Serial.print('"');
  Serial.print(now.year(), DEC);
  Serial.print("/");
  Serial.print(now.month(), DEC);
  Serial.print("/");
  Serial.print(now.day(), DEC);
  Serial.print(" ");
  Serial.print(now.hour(), DEC);
  Serial.print(":");
  Serial.print(now.minute(), DEC);
  Serial.print(":");
  Serial.print(now.second(), DEC);
  Serial.print('"');
#endif //ECHO_TO_SERIAL
}

void heatCool() {
  uint8_t curTmp;
  curTmp = displayCurrTemp();
  if ( (curTmp-tempPt) > TEMP_RNG) {
    state = COOLING;
    digitalWrite(COOLING_PIN, HIGH);
    digitalWrite(HEATING_PIN, LOW); 
  }
  else if ( (tempPt-curTmp) > TEMP_RNG) {
    state = HEATING;
    digitalWrite(COOLING_PIN, LOW);
    digitalWrite(HEATING_PIN, HIGH); 
  }
  else {
    state = STATIC;
    digitalWrite(COOLING_PIN, LOW);
    digitalWrite(HEATING_PIN, LOW); 
  }
}

void printState() {
  switch (state) {
    case HEATING: // heating
      logfile.print("heating");
      #if ECHO_TO_SERIAL
      Serial.print("heating"); 
      #endif //ECHO_TO_SERIAL
      break;
    case COOLING:  // cooling
      logfile.print("cooling"); 
      #if ECHO_TO_SERIAL
      Serial.print("cooling"); 
      #endif //ECHO_TO_SERIAL
      break;
    case STATIC: // nothing
      logfile.print("static"); 
      #if ECHO_TO_SERIAL
      Serial.print("static"); 
      #endif //ECHO_TO_SERIAL
  }
}

void loop() {
  uint32_t curMillis=millis();
  if ( (curMillis-lastLogMillis)>LOG_INTERVAL) {
    digitalWrite(greenLEDpin, HIGH);
    lastLogMillis = curMillis;
    uint8_t curTmp, bkgd;
    curTmp = displayCurrTemp();
    bkgd = setBkgd(curTmp);
    writeTime();
    logfile.print(", ");
    logfile.print(curTmp);
    logfile.print(", ");
    #if ECHO_TO_SERIAL
    Serial.print(", ");
    Serial.print(curTmp);
    Serial.print(", ");
    #endif //ECHO_TO_SERIAL
    printState();
    switch (bkgd) {
      case RED: // heating
        logfile.println(", red");
        #if ECHO_TO_SERIAL
        Serial.println(", red");
        #endif //ECHO_TO_SERIAL
        break;
      case YELLOW:  // cooling
        logfile.println(", yellow"); 
        #if ECHO_TO_SERIAL
        Serial.println(", yellow"); 
        #endif //ECHO_TO_SERIAL
        break;
      case GREEN: // nothing
        logfile.println(", green");
        #if ECHO_TO_SERIAL
        Serial.println(", green");
        #endif //ECHO_TO_SERIAL
    }
    digitalWrite(greenLEDpin, LOW);
    if ( (sync_cnt++) >= SYNC_INTERVAL) {
      digitalWrite(redLEDpin, HIGH);
      sync_cnt = 0;
      logfile.flush();
      digitalWrite(redLEDpin, LOW);
      // #define TEMP_RNG 5
      /////////////////////////
      // Change the state for the Heat/Cool
      /////////////////////////
      heatCool();
    }
  }
  if ( (curMillis-lastReadMillis)>TEMP_INTERVAL) {
    lastReadMillis = curMillis;
    uint8_t curTmp;
    curTmp = displayCurrTemp();
    setBkgd(curTmp);
  }

  
}








