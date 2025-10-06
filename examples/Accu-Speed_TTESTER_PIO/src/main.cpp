#include <Arduino.h>

#include "RAK3172.h"

#include <Adafruit_ADS1X15.h>

#include <NeoPixelBus.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define SDA2 14
#define SCL2 13
TwoWire I2C2 = TwoWire(0);

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &I2C2);


const uint16_t PixelCount = 28; // make sure to set this to the number of pixels in your strip
const uint8_t PixelPin = 0;  // make sure to set this to the correct pin, ignored for Esp8266

NeoPixelBus<NeoGrbFeature, NeoEsp32BitBang800KbpsMethod> pixel(PixelCount, PixelPin);


//RAK3172 LoRa(true); //true for debug mode
RAK3172 LoRa(false); //false for normal mode

Adafruit_ADS1015 ADS1;     /* Use this for the 12-bit version */
Adafruit_ADS1015 ADS2;     /* Use this for the 12-bit version */

// #define SDA 14
// #define SCL 13

uint32_t frequency = 868000000;
uint8_t sf = 7;
uint8_t bw = 0;
uint8_t cr = 0;
uint8_t prlen = 10;
uint8_t pwr = 14;

size_t msglen;
uint8_t rxMessage[128];

#define SLAVE_ADDRESS 0x01
#define MASTER_ADDRESS 0X00

typedef enum {
  CMD_READ_ADC = 0x11,
  CMD_READ_DI = 0x12,
  CMD_READ_RELAY = 0x13,
  CMD_SET_RELAY = 0x14,
  CMD_ERROR = 0xFF
} CommandType;


typedef enum {
  ERROR_INVALID_CMD = 0x01,
  ERROR_INVALID_SLAVE_ADDRESS = 0x02,
  ERROR_INVALID_MASTER_ADDRESS = 0x03
} ErrorCode;
// Function Prototypes
void LoRaInit(void);
void initADC(void);
uint16_t ReadADCChannel(uint8_t channel);
void readLocalADCVoltage(void);
uint16_t adctomv(uint16_t adc);
uint16_t adctoua(uint16_t adc);
void InitDisplay(void);


typedef enum {
  HW_NONE = 0x00,
  HW_VER2 = 0x01,
  HW_VER3 = 0x02
} HW_Version;

HW_Version ADC_HW_VER = HW_NONE;


float scaleFactor;       // (R1 + R2) / R2
float referenceVoltage;  // ADC reference voltage
int adcResolution;       // ADC resolution
float resistorValue;     //Shunt resistor value in Ohms



void setup() 
{
  Serial.begin(115200);

  pixel.Begin();
  pixel.Show();

  I2C2.begin(SDA2, SCL2);

  delay(100);

  LoRaInit();
  initADC();
  InitDisplay();

  //create led task
  xTaskCreate(
    [](void*){
      for(;;)
      {
        for (uint16_t i = 0; i < PixelCount; i++)
        {
            pixel.SetPixelColor(i, RgbColor(0, 0, 100));
            pixel.Show();
            delay(50);
        }
        delay(500);
        for (uint16_t i = 0; i < PixelCount; i++)
        {
            pixel.SetPixelColor(i, RgbColor(0, 100, 0));
            pixel.Show();
            delay(50);
        }
        delay(500);
                for (uint16_t i = 0; i < PixelCount; i++)
        {
            pixel.SetPixelColor(i, RgbColor(100, 0, 0));
            pixel.Show();
            delay(50);
        }
        delay(500);
      }
    },
    "LED_Task",
    2048,
    NULL,
    1,
    NULL
  );  
}

void loop() 
{
  readLocalADCVoltage();
 
  Serial.println("============");
  delay(2000);

}


void readLocalADCVoltage(void)
{
  Serial.print("Local AI Voltage: ");
  for(int i = 0; i < 8; i++)
  {
    uint16_t tADCvalue = ReadADCChannel(i);
    Serial.print("AI" + String(i) + ":" + String(adctomv(tADCvalue)) + "mV ");
  }
  Serial.println();
  Serial.flush();
}

void LoRaInit(void)
{
   Serial.println("Starting RAK3172");
   Serial.flush();

  if(LoRa.begin())
  {
    Serial.println("RAK3172 Init success!");
    Serial.flush();
    
  }
  else
  {
    Serial.println("RAK3172 Init failed!");
    Serial.flush();
  }

  Serial.println("Version:" + LoRa.getVersion());
  Serial.flush();

  if(LoRa.setLoRaMode(RAK3172_MODE_LORAP2P))
  {
	  Serial.println("LoRa Mode set to LoRaP2P Sucssefully");
    Serial.flush();
  }
  else
  {
    Serial.println("LoRa Mode set to LoRaP2P Failed");
    Serial.flush();
  }

  if(LoRa.setLoRaP2PParameters(frequency, sf, bw, cr, prlen, pwr))
  {
    Serial.println("LoRa P2P Parameters set successfully");
    Serial.flush();
  }
  else
  {
    Serial.println("LoRa P2P Parameters set failed");
    Serial.flush();
  }

  if(LoRa.getLoRaP2PParameters(&frequency, &sf, &bw, &cr, &prlen, &pwr))
  {
    Serial.println("LoRa P2P Parameters:");
    Serial.println("Frequency: " + String(frequency));
    Serial.println("SF: " + String(sf));
    Serial.println("BW: " + String(bw));
    Serial.println("CR: " + String(cr));
    Serial.println("PRLen: " + String(prlen));
    Serial.println("PWR: " + String(pwr));
    Serial.flush();
  }
  else
  {
    Serial.println("LoRa P2P Parameters failed!");
    Serial.flush();
  }

  if(LoRa.setReceiveMode(RAK3172_TX_MODE))
  {
    Serial.println("LoRa set to TX Mode");
    Serial.flush();
  }
  else
  {
    Serial.println("LoRa set to TX Mode failed");
    Serial.flush();
  }
}

void initADC(void)
{
  //Wire.begin(SDA,SCL);

  if((ADS1.begin(0x48,&I2C2))&&(ADS2.begin(0x49,&I2C2)))
  {
    ADC_HW_VER = HW_VER2;
    scaleFactor = 32.0 / 10.0;  // (R1 + R2) / R2
    referenceVoltage = 4096.0;  // Reference voltage in mV
    adcResolution = 2048;       // resolution 11-bit
    resistorValue = 162;        //Shunt resistor value in Ohms

    ADS1.setGain(GAIN_ONE);
    ADS2.setGain(GAIN_ONE);

    Serial.println("ADC_HW_V2 init OK");
  }
  else if((ADS1.begin(0x4A,&I2C2))&&(ADS2.begin(0x4B,&I2C2)))
  {
    ADC_HW_VER = HW_VER3;

    scaleFactor = 50.2 / 10.0;  // (R1 + R2) / R2
    referenceVoltage = 2048.0;  // Reference voltage in mV
    adcResolution = 2048;       // resolution 11-bit
    resistorValue = 100;        //Shunt resistor value in Ohms

    ADS1.setGain(GAIN_TWO);
    ADS2.setGain(GAIN_TWO);

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

uint16_t ReadADCChannel(uint8_t channel)
{
  if(channel<4)
  {
    return ADS1.readADC_SingleEnded(channel); 
  }
  else
  {
    return ADS2.readADC_SingleEnded(channel-4);
  }
}

uint16_t adctomv(uint16_t adc)
{
  return (adc * referenceVoltage / adcResolution) * scaleFactor;
}

uint16_t adctoua(uint16_t adc)
{
  float voltage_mV = (adc * referenceVoltage) / adcResolution;
  uint16_t current_uA = (voltage_mV * 1000) / resistorValue;
  return current_uA;
}

void InitDisplay(void)
{
    
    delay(1);
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
        Serial.println(F("display allocation failed"));
        for (;;); // Don't proceed, loop forever
    }
    display.clearDisplay();
    display.display();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Starting...");
    display.println("Testing...");
    display.display();
    delay(2);
}