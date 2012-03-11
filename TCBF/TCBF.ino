#include <SD.h>
//#include "RTClib.h"

// include the library code:
#include <Wire.h>
#include <Adafruit_MCP23017.h>
#include <Adafruit_RGBLCDShield.h>

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

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

// The analog pins that connect to the sensors
#define tempPin 1                // analog 1
#define aref_voltage 3.3         // we tie 3.3V to ARef and measure it with a multimeter!


const char* BLANK={"                "};

uint8_t tempPt = 70;



void setup() {
  // We are going to use the 3.3V ref
  analogReference(EXTERNAL);

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
byte currentTemp() {
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
  return (byte)steinhart;   // a bit of waste to do a real calc then (byte) but for now 
}

void displayCurrTemp() {
Serial.println("In displayCurrTemp");
  lcd.setCursor(0, 0);
  char line[16];
  sprintf(line, "Curr T: %3u F", currentTemp());
  lcd.print(line);
  lcd.setCursor(0, 1);
  sprintf(line, " Set T: %3u F", tempPt);
  lcd.print(line);
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



uint8_t i=0;
void loop() {
  displayCurrTemp();
  delay(1000);
  
}








