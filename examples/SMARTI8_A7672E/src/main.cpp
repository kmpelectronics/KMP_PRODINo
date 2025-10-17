#include <Arduino.h>
#include "KMPCommon.h"
#include "KMPSmarti8ESP32.h"
#include <Adafruit_ADS1X15.h>


Adafruit_ADS1015 ADS1;     /* Use this for the 12-bit version */

#define SDA 21
#define SCL 22

//GSM
#define TINY_GSM_MODEM_A7672X
//#define TINY_GSM_MODEM_SIM7600

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial

// Set serial for AT commands (to the module)
#define SerialAT Serial1


// Increase RX buffer to capture the entire response
// Chips without internal buffering (A6/A7, ESP8266, M590)
// need enough space in the buffer for the entire response
// else data will be lost (and the http library will fail).
#if !defined(TINY_GSM_RX_BUFFER)
#define TINY_GSM_RX_BUFFER 650
#endif

// See all AT commands, if wanted
#define DUMP_AT_COMMANDS

// Define the serial console for debug prints, if needed
#define TINY_GSM_DEBUG SerialMon

#define DEBUG true

// Add a reception delay, if needed.
// This may be needed for a fast processor at a slow baud rate.
// #define TINY_GSM_YIELD() { delay(2); }

// Uncomment this if you want to use SSL
// #define USE_SSL

// set GSM PIN, if any
#define GSM_PIN "0000"

// Your GPRS credentials, if any
const char apn[] = "internet.a1.bg";
const char gprsUser[] = "";
const char gprsPass[] = "";

// Server details
const char server[] = "vsh.pp.ua";
const char resource[] = "/TinyGSM/logo.txt";

#define GSM_RESETN  14
#define GSM_POWERON 26

#define RXD2 34
#define TXD2 25

// baud rate used for both Serial ports
unsigned long baud = 115200;

#include <TinyGsmClient.h>

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm        modem(debugger);
#else
TinyGsm        modem(SerialAT);
#endif

#ifdef USE_SSL
TinyGsmClientSecure client(modem);
const int           port = 443;
#else
TinyGsmClient  client(modem);
const int      port = 80;
#endif

#define SMS_TARGET  "+359877738070"

/////////////////////////////////////////////////

#define DI_COUNT 8
#define RELAY_COUNT 8

KMPSmarti8ESP32Class board;

//prototype functions
void initBoard(void);
void initGSM(void);
bool sendSMS(const char* number, const String& text);
bool sendIOStatusSMS(const char* number);

void initADC(void);
void readLocalADCVoltage(void);
uint16_t adctomv(uint16_t adc);

float scaleFactor;       // (R1 + R2) / R2
float referenceVoltage;  // ADC reference voltage
int adcResolution;       // ADC resolution
float resistorValue;     //Shunt resistor value in Ohms

void setup() 
{
  // Set console baud rate
  SerialMon.begin(115200);
  delay(10);

  initBoard();

  initGSM();

  initADC();


}

void loop() 
{
  //check input 7 status and send sendIOStatusSMS
  static bool lastDI7State = false;
  bool currentDI7State = board.getOptoInState(7);
  if(currentDI7State != lastDI7State)
  {
    lastDI7State = currentDI7State;
    sendIOStatusSMS(SMS_TARGET);
  } 

  readLocalADCVoltage();

  delay(500);


}

void initBoard(void)
{
  board.begin(SMARTI8_ESP32);
  board.setStatusLed(green);
  board.setAllRelaysOff();
  board.setStatusLed(blue);
}

void initGSM(void)
{
  pinMode(GSM_RESETN, OUTPUT);
  pinMode(GSM_POWERON, OUTPUT);
  digitalWrite(GSM_RESETN, LOW);
  digitalWrite(GSM_POWERON, LOW);

  delay(100); //The time from power on to pwrkey can bepulled down 30ms
  digitalWrite(GSM_POWERON, HIGH);
  delay(50); // Power on low level pulse width 50ms
  digitalWrite(GSM_POWERON, LOW);

  SerialMon.println("Wait...");

  // Set GSM module baud rate
  SerialAT.begin(baud, SERIAL_8N1, RXD2, TXD2);
  //delay(1000);

  DBG("Initializing modem...");
    if (!modem.init()) {
      
      while(1){
        DBG("Failed to initialize modem.");
        esp_restart();
        delay(1000);
      }
    }

  String modemInfo = modem.getModemInfo();
  DBG("Modem Info:", modemInfo);

  String hw_ver = modem.getModemModel();
  DBG("Modem Hardware Version:", hw_ver);

  String fv_ver = modem.getModemRevision();
  DBG("Modem Firware Version:", fv_ver);

  String mod_sn = modem.getModemSerialNumber();
  DBG("Modem Serial Number (may be SIM CCID):", mod_sn);

  // Unlock your SIM card with a PIN if needed
  if (GSM_PIN && modem.getSimStatus() != 3) { modem.simUnlock(GSM_PIN); }

  DBG("Waiting for network...");
  if (!modem.waitForNetwork()) {
      while(1){
      DBG("Failed to connect to network.");
      delay(1000);
    }
    }
    DBG("Connected to network");

  if (modem.isNetworkConnected()) { DBG("Network connected"); }
  else {
      while(1){
      DBG("Network not connected.");
      delay(1000);
    }
  }

  sendIOStatusSMS(SMS_TARGET);
  
}


// Send SMS with custom text
bool sendSMS(const char* number, const String& text)
{
  bool res = modem.sendSMS(number, text);
  DBG("SMS to " + String(number) + ":", res ? "OK" : "fail");
  return res;
}


// Send SMS with DI and RELAY status
bool sendIOStatusSMS(const char* number)
{
  String smsText = "DI Status: ";
  for(int i = 0; i < DI_COUNT; i++)
  {
    smsText += "DI" + String(i) + ":" + String(board.getOptoInState(i)) + " ";
  }
  smsText += "\nPower State: ";
  if(board.getOptoInState(7))
  {
    smsText += "ON\n";
  }
  else
  {
    smsText += "OFF\n";
  }

  return sendSMS(number, smsText);
}


void readLocalADCVoltage(void)
{
  Serial.print("Analog In Voltage: ");
  for(int i = 0; i < 4; i++)
  {
    uint16_t tADCvalue = ADS1.readADC_SingleEnded(i);
    Serial.print("AI" + String(i) + ":" + String(adctomv(tADCvalue)) + "mV ");
  }
  Serial.println();
  Serial.flush();
}

void initADC(void)
{
  Wire.begin(SDA,SCL);

 if(ADS1.begin(0x48))//4A
  {

    scaleFactor = 50.2 / 20.0;  // (R1 + R2) / R2
    referenceVoltage = 2048.0;  // Reference voltage in mV
    adcResolution = 2048;       // resolution 11-bit
    resistorValue = 100;        //Shunt resistor value in Ohms

    ADS1.setGain(GAIN_TWO);

    Serial.println("ADC_HW_V3 init OK");
  }
  else
  {
    
    while (1)
    {
      Serial.println("ADC not initialized");
      delay(1000);
    }
    
  }

}

uint16_t adctomv(uint16_t adc)
{
  return (adc * referenceVoltage / adcResolution) * scaleFactor;
}
