#include <Arduino.h>

#include "RAK3172.h"

#include <Adafruit_ADS1X15.h>

#include <NeoPixelBus.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <Adafruit_MCP23X17.h>

#include <Ethernet.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define SDA2 14
#define SCL2 13
TwoWire I2C2 = TwoWire(0);

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &I2C2);


const uint16_t PixelCount = 33; // make sure to set this to the number of pixels in your strip
const uint8_t PixelPin = 0;  // make sure to set this to the correct pin, ignored for Esp8266

NeoPixelBus<NeoGrbFeature, NeoEsp32BitBang800KbpsMethod> pixel(PixelCount, PixelPin);


//RAK3172 LoRa(true); //true for debug mode
RAK3172 LoRa(false); //false for normal mode

Adafruit_ADS1015 ADS1;     /* Use this for the 12-bit version */
Adafruit_ADS1015 ADS2;     /* Use this for the 12-bit version */

Adafruit_MCP23X17 mcp_2;
Adafruit_MCP23X17 mcp_1;
Adafruit_MCP23X17 mcp_0;

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
void createledtask(void);
void InitRelay(void);
void SetRelay(uint8_t relay, uint8_t state);
void InitEthernet(void);
void InitRS485(void);
void initDIO(void);
void initButtons(void);


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

uint8_t RelayLed[13] = {15,14,13,12,19,18,17,16,24,23,22,21,20};
uint8_t Relays[13] = {3,2,1,0,7,6,5,4,10,9,8,12,11};
uint8_t RelayCount = 13;


// W5500 pins.
#define W5500ResetPin 15  
#define W5500CSPin    33 
#define W5500Int      5 

byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};


//RS485
#define RS1DE 12
#define RS1TX 32
#define RS1RX 36

#define RS2DE 2
#define RS2TX 16
#define RS2RX 39

#define RS3DE 4
#define RS3TX 17
#define RS3RX 34

void setup() 
{
  Serial.begin(115200);

  pixel.Begin();
  pixel.Show();

  I2C2.begin(SDA2, SCL2);

  mcp_0.begin_I2C(0x20, &I2C2);
  mcp_1.begin_I2C(0x21, &I2C2);  
  mcp_2.begin_I2C(0x22, &I2C2);

  InitRelay();

  //InitEthernet();

  //delay(100);

  LoRaInit();
  initADC();
  InitDisplay();

  

  // for (size_t i = 0; i < RelayCount; i++)
  // {
  //   SetRelay(i, HIGH);
  //   delay(200);
  //   SetRelay(i, LOW);
  //   delay(200);
  // }

  // delay(200);
  // createledtask();

  //InitRS485();
  initDIO();
  //initButtons();
  
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

void createledtask(void)
{
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

void InitRelay(void)
{
  for (int i = 0; i < 13; i++)
  {
    mcp_1.pinMode(i, OUTPUT);
    mcp_1.digitalWrite(i, LOW);
  }

  for (size_t i = 0; i < sizeof(RelayLed); i++)
  {
    pixel.SetPixelColor(RelayLed[i], RgbColor(0, 50, 0));
    pixel.Show();
    delay(100);
  }
  
}

void SetRelay(uint8_t relay, uint8_t state)
{
  if(relay < 13)
  {
    mcp_1.digitalWrite(relay, state);
    state == HIGH  ? pixel.SetPixelColor(RelayLed[relay], RgbColor(50, 0, 0)) : pixel.SetPixelColor(RelayLed[relay], RgbColor(0, 50, 0));
    pixel.Show();
  }
}

void InitEthernet(void)
{
	// W5500 pin init.
	pinMode(W5500ResetPin, OUTPUT);

	digitalWrite(W5500ResetPin, LOW);
	delay(600);
	digitalWrite(W5500ResetPin, HIGH);
	Ethernet.init(W5500CSPin);
  if(Ethernet.begin(mac) == 0)
  {
    Serial.println("Failed to configure Ethernet using DHCP");
    // no point in carrying on, so do nothing forevermore:
  }

  Serial.println("Ethernet configured via DHCP");
  Serial.print("IP address: ");
  Serial.println(Ethernet.localIP()); 
  Serial.flush();
}

void InitRS485(void)
{
  pinMode(RS1DE, OUTPUT);
  pinMode(RS2DE, OUTPUT);
  pinMode(RS3DE, OUTPUT);

  digitalWrite(RS1DE, LOW);
  digitalWrite(RS2DE, LOW);
  digitalWrite(RS3DE, LOW);

  Serial1.begin(9600, SERIAL_8N1, RS1RX, RS1TX);

  uint32_t count = 0;

  do
  {
    digitalWrite(RS1DE, HIGH); //Enable RS485 Transmit
    delay(10);
    Serial1.println("Serial1");
    Serial1.flush();
    digitalWrite(RS1DE, LOW); //Disable RS485 Transmit
    if(Serial1.available())
    {
      String rcv = Serial1.readStringUntil('\n');
      Serial.println("Received from RS485-1: " + rcv);
      Serial.flush();
    }

    delay(100);
    count++;
  } while (count < 50);

  Serial1.end();

  Serial1.begin(9600, SERIAL_8N1, RS2RX, RS2TX);
  count = 0;
  
  do
  {
    digitalWrite(RS2DE, HIGH); //Enable RS485 Transmit
    delay(10);
    Serial1.println("Serial2");
    Serial1.flush();
    digitalWrite(RS2DE, LOW); //Disable RS485 Transmit
    if(Serial1.available())
    {
      String rcv = Serial1.readStringUntil('\n');
      Serial.println("Received from RS485-2: " + rcv);
      Serial.flush();
    }

    delay(100);
    count++;
  } while (count < 50);

  Serial1.end();



  Serial1.begin(9600, SERIAL_8N1, RS3RX, RS3TX);
  count = 0;
  
  do
  {
    digitalWrite(RS3DE, HIGH); //Enable RS485 Transmit
    delay(10);
    Serial1.println("Serial3");
    Serial1.flush();
    digitalWrite(RS3DE, LOW); //Disable RS485 Transmit
    if(Serial1.available())
    {
      String rcv = Serial1.readStringUntil('\n');
      Serial.println("Received from RS485-3: " + rcv);
      Serial.flush();
    }

    delay(100);
    count++;
  } while (count < 50);

  Serial1.end();
}

void initDIO(void)
{
  for (int i = 0; i < 16; i++)
  {
    mcp_0.pinMode(i, INPUT);
  }

  mcp_0.pinMode(15, OUTPUT);

  do
  {
    for (int i = 0; i < 16; i++)
    {
      Serial.print("DI" + String(i) + ":");
      mcp_0.digitalRead(i) == HIGH ? Serial.print("HIGH ") : Serial.print("LOW ");
      delay(1);
    }
    Serial.println("----");
    Serial.flush();
    mcp_0.digitalWrite(15, HIGH);
    delay(300);
    mcp_0.digitalWrite(15, LOW);
    //delay(100);
  } while (1);
  

  
}

void initButtons(void)
{
  for (int i = 0; i < 16; i++)
  {
    mcp_2.pinMode(i, INPUT);
  }

  do
  {
    for (int i = 0; i < 16; i++)
    {
      Serial.print("DI" + String(i) + ":");
      mcp_2.digitalRead(i) == HIGH ? Serial.print("HIGH ") : Serial.print("LOW ");
      delay(1);
    }
    Serial.println("----");
    Serial.flush();
    delay(100);
  } while (1);
  
}

