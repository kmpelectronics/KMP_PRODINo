#include "Melopero_RV3028.h"

#include <Arduino.h>

#include "KMPCommon.h"
#include "KMPSmarti8ESP32.h"

Melopero_RV3028 rtc;

KMPSmarti8ESP32Class board;

#define SDA 14
#define SCL 13


//prototype
void printTime();


void setup() {
  Serial.begin(115200);
  
  board.begin(SMARTI8_ESP32_Ethernet);
  board.setStatusLed(green);
  board.setAllRelaysOff();
  
  // First initialize and create the rtc device
  Wire.begin(SDA,SCL);
  rtc.initI2C();
  // use the following to use Wire1 instead of Wire:
  // Wire1.begin();
  // rtc.initI2C(Wire1);
 
  // Set the device to use the 24hour format (default) instead of the 12 hour format
  rtc.set24HourMode();

  // Set the date and time:
  rtc.setTime(2024, 9, 2, 1, 23, 01, 0);
  // year, month, weekday, date, hour, minute, second
  // Note: time is always set in 24h format
  // Note: month value ranges from 1 (Jan) to 12 (Dec)
  // Note: date value ranges from 1 to 31

  // print datetime to make sure everything works
  printTime();
}

void loop() {
  // print datetime every second
  printTime();
  delay(1000);
}

void printTime(){
  Serial.print(rtc.getHour());
  Serial.print(":");
  Serial.print(rtc.getMinute());
  Serial.print(":");
  Serial.print(rtc.getSecond());
  Serial.print("   ");
  Serial.print(rtc.getDate());
  Serial.print("/");
  Serial.print(rtc.getMonth());
  Serial.print("/");
  Serial.println(rtc.getYear());
  Serial.println();
  // print the unix time
  Serial.println(rtc.getUnixTime());
  
}