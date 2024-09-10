#include <Arduino.h>
#include "KMPCommon.h"
#include "KMPSmarti8ESP32.h"
#include "driver/pcnt.h"  // Include the Pulse Counter library
#include <ESP32_FTPClient.h>


// Define the pulse counter unit and the GPIO pin to be used
#define PCNT_UNIT PCNT_UNIT_0    // Pulse Counter Unit
#define PULSE_INPUT_PIN 21       // Define GPIO pin for input pulses (21 or 22)

void IRAM_ATTR handlePulse() {
    // This function will be called on each pulse (if interrupts are used)
}

// Create an instance of the KMPSmarti8ESP32Class
KMPSmarti8ESP32Class board;

// MAC address for Ethernet
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};

// Create an instance of the EthernetServer
EthernetServer server(80);


char ftp_server[] = "79.116.72.34";
char ftp_user[]   = "Test";
char ftp_pass[]   = "Test1234";

// you can pass a FTP timeout and debbug mode on the last 2 arguments
ESP32_FTPClient ftp (ftp_server,ftp_user,ftp_pass, 5000, 2);

// Create a task handle for the toggleLED task
TaskHandle_t toggleLEDHandle = NULL;


#define FTP_UPLOAD_INTERVAL 20000  // Upload interval in milliseconds
unsigned long lastUploadTime = 0;  // Last upload time


// Prototypes
void toggleLED(void* parameter);
void configurePulseCounter();
void configureEthernet();
void handleClient();
void uploadPilseCount(uint16_t pulseCount);


// The setup() function runs once each time the micro-controller starts
void setup()
{
  Serial.begin(115200);
  delay(200);
  Serial.println(F("Starting..."));

  board.begin(SMARTI8_ESP32_Ethernet);
  board.setStatusLed(green, 0);
  board.setStatusLed(red, 1);

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


  configurePulseCounter();
  configureEthernet();


}

void loop() {

  if(lastUploadTime < millis())
  {
    // Serial.println(lastUploadTime);
    // Serial.println(millis());

    lastUploadTime = millis() + FTP_UPLOAD_INTERVAL;

    int16_t pulseCount = 0;
    
    // Read the pulse counter
    pcnt_get_counter_value(PCNT_UNIT, &pulseCount);
    
    Serial.print("Pulse Count: ");
    Serial.println(pulseCount);

    // Upload the file
    uploadPilseCount(pulseCount);
  }

  //handleClient();
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


void configurePulseCounter() {
    
    // Configure Pulse Counter (PCNT)
    pcnt_config_t pcnt_config;
    pcnt_config.pulse_gpio_num = PULSE_INPUT_PIN;  // GPIO for pulse input
    pcnt_config.ctrl_gpio_num = PCNT_PIN_NOT_USED; // No control pin used
    pcnt_config.channel = PCNT_CHANNEL_0;
    pcnt_config.unit = PCNT_UNIT;
    pcnt_config.pos_mode = PCNT_COUNT_INC;         // Count rising edges
    pcnt_config.neg_mode = PCNT_COUNT_DIS;         // Ignore falling edges
    pcnt_config.lctrl_mode = PCNT_MODE_KEEP;       // No control for low
    pcnt_config.hctrl_mode = PCNT_MODE_KEEP;       // No control for high
    pcnt_config.counter_h_lim = 10000;             // Set high limit
    pcnt_config.counter_l_lim = 0;                 // Set low limit

    // Initialize Pulse Counter
    pcnt_unit_config(&pcnt_config);

    // Filter pulses shorter than 10.23uS
    pcnt_set_filter_value(PCNT_UNIT, 1023);
    pcnt_filter_enable(PCNT_UNIT);

    // Enable events on zero or high limit
    pcnt_event_enable(PCNT_UNIT, PCNT_EVT_H_LIM);
    pcnt_event_enable(PCNT_UNIT, PCNT_EVT_ZERO);

    // Initialize pulse counter value to zero
    pcnt_counter_pause(PCNT_UNIT);                 // Stop counter
    pcnt_counter_clear(PCNT_UNIT);                 // Reset counter to 0
    pcnt_counter_resume(PCNT_UNIT);                // Start counter

    Serial.println("Pulse Counter Initialized");
}

void configureEthernet() {
    // Start Ethernet
    Ethernet.begin(mac);

      // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet controller was not found.");
    while (true) {
      delay(1); // do nothing, no point running without Ethernet hardware
    }
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
  }

  // start the server
  server.begin();
  Serial.print("server is at ");
  Serial.println(Ethernet.localIP());

}

void handleClient() {
  // listen for incoming clients
  EthernetClient client = server.available();
  if (client) {
    Serial.println("new client");
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        Serial.write(c);
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {
          // send a standard http response header
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/html");
          client.println("Connection: close");  // the connection will be closed after completion of the response
          client.println("Refresh: 5");  // refresh the page automatically every 5 sec
          client.println();
          client.println("<!DOCTYPE HTML>");
          client.println("<html>");

          client.print("Refresh the page automatically every 5 sec.");
          client.println("<br />");
          client.print("Pulse Count: ");
          int16_t pulseCount = 0;
          pcnt_get_counter_value(PCNT_UNIT, &pulseCount);
          client.print(pulseCount);
          client.println("<br />");

          client.println("</html>");
          break;
        }
        if (c == '\n') {
          // you're starting a new line
          currentLineIsBlank = true;
        } else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
    // give the web browser time to receive the data
    delay(1);
    // close the connection:
    client.stop();
    Serial.println("client disconnected");
  }
}

void uploadPilseCount(uint16_t pulseCount) {
  
  ftp.OpenConnection();

  // // Get directory content
  // ftp.InitFile("Type A");

  // String list[128];

  // ftp.ContentList("", list);


  // Serial.println("\nDirectory info: ");
  // for(int i = 0; i < sizeof(list); i++)
  // {
  //   if(list[i].length() > 0)
  //     Serial.println(list[i]);
  //   else
  //     break;
  // }

  // // Make a new directory
  // ftp.MakeDir("PRODINo_TEST");

  //Change directory
  ftp.ChangeWorkDir("Publico");
  
  // Create a new file to use as the download example below:
  ftp.InitFile("Type A");
  ftp.NewFile("pulseCount.txt");

  char pulseCountStr[10];
  sprintf(pulseCountStr, "%d", pulseCount);

  ftp.Write(pulseCountStr);
  ftp.CloseFile();

  //Download the text file or read it
  String response = "";
  ftp.InitFile("Type A");
  ftp.DownloadString("pulseCount.txt", response);
  Serial.println("The pulseCount is: " + response);

  ftp.CloseConnection();

}