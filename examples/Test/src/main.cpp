#include <Arduino.h>
#include "KMPCommon.h"

#define SMARTI8
//#define PRODINoESP32

#ifdef SMARTI8
#include "KMPSmarti8ESP32.h"
KMPSmarti8ESP32Class board;
#endif

#ifdef PRODINoESP32
#include "KMPProDinoESP32.h"
KMPProDinoESP32Class board;
#endif



TaskHandle_t toggleLEDHandle = NULL;
void toggleLED(void* parameter);


// The setup() function runs once each time the micro-controller starts
void setup()
{
  Serial.begin(115200);
  delay(200);
  Serial.println(F("Starting..."));

  

  #ifdef SMARTI8
  board.begin(SMARTI8_ESP32_Ethernet);
  board.setStatusLed(green, 0);
  board.setStatusLed(red, 1);
  #endif

  #ifdef PRODINoESP32
  board.begin(ProDino_ESP32_Ethernet);
  board.setStatusLed(green);
  #endif

  // Reset Relay status.
  board.setAllRelaysOff();

    xTaskCreate(
        toggleLED,    // Function that should be called
        "Toggle LED",   // Name of the task (for debugging)
        1000,            // Stack size (bytes)
        NULL,            // Parameter to pass
        1,               // Task priority
        &toggleLEDHandle // Task handle
    );


}

void loop() {

    for(uint8_t i=0;i<RELAY_COUNT;i++)
  {
    board.setRelayState(i, true);
    delay(500);
  }

  delay(500);

    for(uint8_t i=0;i<RELAY_COUNT;i++)
  {
    board.setRelayState(i, false);
    delay(500);
  }

  delay(500);
}

void toggleLED(void* parameter) {

for (;;) 
  { // infinite loop

  #ifdef SMARTI8

  board.setStatusLed(red, 0);
  vTaskDelay(500 / portTICK_PERIOD_MS);

  board.setStatusLed(green, 0);
  vTaskDelay(500 / portTICK_PERIOD_MS);

  board.setStatusLed(blue, 0);
  vTaskDelay(500 / portTICK_PERIOD_MS);

  board.offStatusLed();
  vTaskDelay(500 / portTICK_PERIOD_MS);

  board.setStatusLed(red, 1);
  vTaskDelay(500 / portTICK_PERIOD_MS);

  board.setStatusLed(green, 1);
  vTaskDelay(500 / portTICK_PERIOD_MS);

  board.setStatusLed(blue, 1);
  vTaskDelay(500 / portTICK_PERIOD_MS);

  board.offStatusLed();
  vTaskDelay(500 / portTICK_PERIOD_MS);
  #else

  board.setStatusLed(red);
  vTaskDelay(500 / portTICK_PERIOD_MS);
  board.setStatusLed(green);
  vTaskDelay(500 / portTICK_PERIOD_MS);
  board.setStatusLed(blue);
  vTaskDelay(500 / portTICK_PERIOD_MS);
  board.offStatusLed();
  vTaskDelay(500 / portTICK_PERIOD_MS);
  #endif
  
  }
}

