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

unsigned int tempPt = 70;



void setup() {
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
  while (1);
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

float currentTemp() {
  int tempReading = analogRead(tempPin);    
  // converting that reading to voltage, for 3.3v arduino use 3.3, for 5.0, use 5.0
  float voltage = tempReading * aref_voltage / 1024;  
  float temperatureC = (voltage - 0.5) * 100 ;
  float temperatureF = (temperatureC * 9 / 5) + 32;  

}


void displayCurrTemp() {
Serial.println("In displayCurrTemp");
  lcd.setCursor(0, 0);
  lcd.print("Curr T: ");
  char line[5];
  sprintf(line, "%3u F", tempPt);
  lcd.setCursor(0, 1);
  lcd.print(line);
  delay(100);  // slow things down
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
  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  lcd.setCursor(0, 1);
  // print the number of seconds since reset:
  lcd.print(millis()/1000);

  uint8_t buttons = lcd.readButtons();

  if (buttons) {
    lcd.clear();
    lcd.setCursor(0,0);
    if (buttons & BUTTON_UP) {
      lcd.print("UP ");
      lcd.setBacklight(RED);
      delay(1000);
      lcd.setCursor(0,0);
      lcd.print("Hello, world!");
    }
    if (buttons & BUTTON_DOWN) {
      lcd.print("DOWN ");
      lcd.setBacklight(YELLOW);
      delay(1000);
      lcd.setCursor(0,0);
      lcd.print("Hello, world!");
    }
    if (buttons & BUTTON_LEFT) {
      lcd.print("LEFT ");
      lcd.setBacklight(GREEN);
      delay(1000);
      lcd.setCursor(0,0);
      lcd.print("Hello, world!");
    }
    if (buttons & BUTTON_RIGHT) {
      lcd.print("RIGHT ");
      lcd.setBacklight(TEAL);
      delay(1000);
      lcd.setCursor(0,0);
      lcd.print("Hello, world!");
    }
    if (buttons & BUTTON_SELECT) {
      lcd.print("SELECT ");
      lcd.setBacklight(VIOLET);
      delay(1000);
      lcd.setCursor(0,0);
      lcd.print("Hello, world!");
    }
  }
}








