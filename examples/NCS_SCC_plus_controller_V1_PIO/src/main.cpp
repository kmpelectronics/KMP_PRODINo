#include <Arduino.h>

#include "RAK3172.h"

#include <Adafruit_ADS1X15.h>

#include <NeoPixelBus.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <Adafruit_MCP23X17.h>

#include <Ethernet.h>

#include <OneWire.h>

#include "SC16IS7X0.h"

#include <SPI.h>
#include <L9822E.h>

// RESET - GPIO36
// CE    - GPIO35
// SI    - GPIO37 (MOSI)
// SO    - GPIO38 (MISO)
// CLK   - GPIO48 (SCK)

static constexpr int DRV_RESET = 36;
static constexpr int DRV_CE    = 35;
static constexpr int DRV_MOSI  = 37;
static constexpr int DRV_MISO  = 38;
static constexpr int DRV_SCK   = 48;

L9822EChain Driver(2, DRV_CE, DRV_RESET);

SPIClass spiL9822(HSPI);

constexpr uint32_t CRYSTAL_FREQ = 14745600;
constexpr uint32_t UART_BAUD = 115200;
constexpr uint8_t UART_RESET = 47;

constexpr uint8_t I2CUART1_ADD = 0x90 >> 1;
constexpr uint8_t I2CUART2_ADD = 0x92 >> 1;
constexpr uint8_t I2CUART3_ADD = 0x94 >> 1;
constexpr uint8_t I2CUART4_ADD = 0x96 >> 1;
constexpr uint8_t I2CUART5_ADD = 0x98 >> 1;
constexpr uint8_t I2CUART6_ADD = 0x9A >> 1;

SC16IS7X0 I2CUART1(CRYSTAL_FREQ);
SC16IS7X0 I2CUART2(CRYSTAL_FREQ);
SC16IS7X0 I2CUART3(CRYSTAL_FREQ);
SC16IS7X0 I2CUART4(CRYSTAL_FREQ);
SC16IS7X0 I2CUART5(CRYSTAL_FREQ);
SC16IS7X0 I2CUART6(CRYSTAL_FREQ);


#define SDA1 8
#define SCL1 9
//TwoWire I2C1 = TwoWire(1);


#define OneWierePin 4
OneWire  ds(OneWierePin);  // on pin 

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define SDA2 41
#define SCL2 40
TwoWire I2C2 = TwoWire(1);

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &I2C2);


const uint16_t PixelCount = 39; // make sure to set this to the number of pixels in your strip
const uint8_t PixelPin = 46;  // make sure to set this to the correct pin, ignored for Esp8266
const uint8_t PixelBrightness = 60; // set brightness 0-255

NeoPixelBus<NeoGrbFeature, NeoEsp32BitBang800KbpsMethod> pixel(PixelCount, PixelPin);

#define Lora_rx 2
#define Lora_tx 1
#define Lora_rst 42


//RAK3172 LoRa(true); //true for debug mode
RAK3172 LoRa(false); //false for normal mode

Adafruit_ADS1015 ADS1;     /* Use this for the 12-bit version */
Adafruit_ADS1015 ADS2;     /* Use this for the 12-bit version */

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


// W5500 pins.
#define W5500ResetPin 14  
#define W5500CSPin    10 
#define W5500Int      21 

byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};

//Solid state relays pins definition
#define SSR0_PIN 6
#define SSR1_PIN 7
#define SSR2_PIN 15
#define SSR3_PIN 16

//Setup Button definition
#define SETUP_BUTTON_PIN 5 //Onboard button


// Function Prototypes
void createledtask(void);
void InitEthernet(void);
void LoRaInit(void);
void InitDisplay(void);
void initADC(void);
void Initds18b20(void);
void initUart(void);
void InitL9822E(void);
void PrintAllAIChannels(void);
uint16_t adctomv(uint16_t adc);
void PrintAIChannel(uint8_t channel);
uint16_t ReadAIChanelmV(uint8_t channel);
bool CheckValueDeviation(uint16_t value1, uint16_t value2, uint8_t percent);
void initDIO(void);
void PrintAllDIOStates(void);
void initButtons(void);
void PrintAllDIChannels(void);


void setup() {

  Serial.begin(115200);
  delay(500);
  Serial.println("NCS SCC+ Controller V1 Starting...");

  pixel.Begin();
  pixel.Show();

  Wire.setPins(SDA1, SCL1);
  //I2C1.begin(SDA1, SCL1);
  I2C2.begin(SDA2, SCL2);


  mcp_0.begin_I2C(0x20, &I2C2);
  mcp_1.begin_I2C(0x21, &I2C2);  

  LoRaInit();
  InitEthernet();
  InitDisplay();
  initADC();
  InitL9822E();
  initUart();
  Initds18b20();
  initDIO();
  display.setRotation(0);
  display.fillRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_WHITE);
  display.display();
  initButtons();

  //createledtask();
}

void loop() {
  // put your main code here, to run repeatedly:
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
            pixel.SetPixelColor(i, RgbColor(0, 0, PixelBrightness));
            pixel.Show();
            delay(50);
        }
        delay(500);
        for (uint16_t i = 0; i < PixelCount; i++)
        {
            pixel.SetPixelColor(i, RgbColor(0, PixelBrightness, 0));
            pixel.Show();
            delay(50);
        }
        delay(500);
                for (uint16_t i = 0; i < PixelCount; i++)
        {
            pixel.SetPixelColor(i, RgbColor(PixelBrightness, 0, 0));
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

void InitEthernet(void)
{

  bool test_result = true;

	// W5500 pin init.
	pinMode(W5500ResetPin, OUTPUT);

	digitalWrite(W5500ResetPin, LOW);
	delay(600);
	digitalWrite(W5500ResetPin, HIGH);
	Ethernet.init(W5500CSPin);
  if(Ethernet.begin(mac) == 0)
  {
    Serial.println("Failed to configure Ethernet using DHCP");
    Serial.flush();
    test_result = false;
    // no point in carrying on, so do nothing forevermore:
  }

  //Serial.println("Ethernet configured via DHCP");
  //Serial.print("IP address: ");
  //Serial.println(Ethernet.localIP()); 
  //Serial.flush();

  if(test_result)
  {
    Serial.println("Ethernet test Passed ");
    Serial.flush();
  }
  else
  {
    Serial.println("Ethernet test Failed==============================================================================!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    Serial.flush();
  }

}

void LoRaInit(void)
{
  bool test_result = true;

  Serial.println("Starting RAK3172 test...");
  Serial.flush();

  if(LoRa.begin(Lora_rx, Lora_tx, Lora_rst))
  {
    // Serial.println("RAK3172 Init success!");
    // Serial.flush();
    
  }
  else
  {
    Serial.println("RAK3172 Init failed!");
    Serial.flush();
    test_result = false;
  }

  // Serial.println("Version:" + LoRa.getVersion());
  // Serial.flush();

  if(LoRa.setLoRaMode(RAK3172_MODE_LORAP2P))
  {
	  // Serial.println("LoRa Mode set to LoRaP2P Sucssefully");
    // Serial.flush();
  }
  else
  {
    Serial.println("LoRa Mode set to LoRaP2P Failed");
    Serial.flush();
    test_result = false;
  }

  if(LoRa.setLoRaP2PParameters(frequency, sf, bw, cr, prlen, pwr))
  {
    // Serial.println("LoRa P2P Parameters set successfully");
    // Serial.flush();
  }
  else
  {
    Serial.println("LoRa P2P Parameters set failed");
    Serial.flush();
    test_result = false;
  }

  if(LoRa.getLoRaP2PParameters(&frequency, &sf, &bw, &cr, &prlen, &pwr))
  {
    // Serial.println("LoRa P2P Parameters:");
    // Serial.println("Frequency: " + String(frequency));
    // Serial.println("SF: " + String(sf));
    // Serial.println("BW: " + String(bw));
    // Serial.println("CR: " + String(cr));
    // Serial.println("PRLen: " + String(prlen));
    // Serial.println("PWR: " + String(pwr));
    // Serial.flush();
  }
  else
  {
    Serial.println("LoRa P2P Parameters failed!");
    Serial.flush();
    test_result = false;
  }

  if(LoRa.setReceiveMode(RAK3172_TX_MODE))
  {
    // Serial.println("LoRa set to TX Mode");
    // Serial.flush();
  }
  else
  {
    Serial.println("LoRa set to TX Mode failed");
    Serial.flush();
    test_result = false;
  }

  if(test_result)
  {
    Serial.println("RAK3172 test Passed ");
    Serial.flush();
  }
  else
  {
    Serial.println("RAK3172 test Failed==============================================================================!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    Serial.flush();
  }
}

void InitDisplay(void)
{
  bool test_result = true;
    
    delay(1);
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
        Serial.println(F("SSD1306 allocation failed"));
        Serial.flush();
        test_result = false;
    }
    display.setRotation(1);
    display.clearDisplay();
    display.display();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Starting");
    display.println("IP Address:");
    display.println(Ethernet.localIP());
    display.display();
    delay(2);

    // display.fillRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_WHITE);
    // display.display();

  if(test_result)
  {
    Serial.println("Display test Passed ");
    Serial.flush();
  }
  else
  {
    Serial.println("Display test Failed==============================================================================!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    Serial.flush();
  }
}

void initADC(void)
{

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

    //Serial.println("ADC_HW_V3 init OK");
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

void Initds18b20(void) {
  byte i;
  byte present = 0;
  byte type_s;
  byte data[9];
  byte addr[8];
  float celsius, fahrenheit;

  Serial.println("1-Wire test starting...");

  uint8_t numDevices = 0;

  while (ds.search(addr))
  {
      //Serial.print("ROM =");
      for( i = 0; i < 8; i++) {
      //Serial.write(' ');
      //Serial.print(addr[i], HEX);
    }

    if (OneWire::crc8(addr, 7) != addr[7]) {
        Serial.println("CRC is not valid!");
        return;
    }
    //Serial.println();
 
    // // the first ROM byte indicates which chip
    // switch (addr[0]) {
    //   case 0x10:
    //     Serial.println("  Chip = DS18S20");  // or old DS1820
    //     type_s = 1;
    //     break;
    //   case 0x28:
    //     Serial.println("  Chip = DS18B20");
    //     type_s = 0;
    //     break;
    //   case 0x22:
    //     Serial.println("  Chip = DS1822");
    //     type_s = 0;
    //     break;
    //   default:
    //     Serial.println("Device is not a DS18x20 family device.");
    //     return;
    // } 

    ds.reset();
    ds.select(addr);
    ds.write(0x44, 1);        // start conversion, with parasite power on at the end
  
    delay(1000);     // maybe 750ms is enough, maybe not
    // we might do a ds.depower() here, but the reset will take care of it.
  
    present = ds.reset();
    ds.select(addr);    
    ds.write(0xBE);         // Read Scratchpad

    //Serial.print("  Data = ");
    //Serial.print(present, HEX);
    //Serial.print(" ");
    for ( i = 0; i < 9; i++) {           // we need 9 bytes
      data[i] = ds.read();
      //Serial.print(data[i], HEX);
      //Serial.print(" ");
    }
    //Serial.print(" CRC=");
    //Serial.print(OneWire::crc8(data, 8), HEX);
    //Serial.println();

    // Convert the data to actual temperature
    // because the result is a 16 bit signed integer, it should
    // be stored to an "int16_t" type, which is always 16 bits
    // even when compiled on a 32 bit processor.
    int16_t raw = (data[1] << 8) | data[0];
    if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
    } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
    }
    celsius = (float)raw / 16.0;
    fahrenheit = celsius * 1.8 + 32.0;
    //Serial.print(" T = ");
    //Serial.println(celsius);
    // Serial.print(" Celsius, ");
    // Serial.print(fahrenheit);
    // Serial.println(" Fahrenheit");
    numDevices++;

  }


  ds.reset_search();

  if(numDevices != 3) {
    Serial.println("1-Wire test failed! Not all devices detected.===============================================================================!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  }
  else {
    Serial.println("1-Wire test Passed ");
  }
  
  
}

void initUart(void)
{

  pinMode(UART_RESET, OUTPUT);
  digitalWrite(UART_RESET, LOW);
  delay(100);
  digitalWrite(UART_RESET, HIGH);
  delay(100);

  bool test_result = true;

  if (I2CUART1.begin_I2C(I2CUART1_ADD))
  {
    //Serial.println("I2C UART1 init success");
    I2CUART1.disableTCR_TLR();
    I2CUART1.enableAutors485();
    I2CUART1.begin_UART(UART_BAUD);
    I2CUART1.enableHardwareRTS();
  }
  else
  {
    Serial.println("I2C UART1 init failed");
  }

  if(I2CUART2.begin_I2C(I2CUART2_ADD))
  {
    //Serial.println("I2C UART2 init success");
    I2CUART2.disableTCR_TLR();
    I2CUART2.enableAutors485();
    I2CUART2.begin_UART(UART_BAUD);
    I2CUART2.enableHardwareRTS();
  }
  else
  {
    Serial.println("I2C UART2 init failed");
  }

  if(I2CUART3.begin_I2C(I2CUART3_ADD))
  {
    //Serial.println("I2C UART3 init success");
    I2CUART3.disableTCR_TLR();
    I2CUART3.enableAutors485();
    I2CUART3.begin_UART(UART_BAUD);
    I2CUART3.enableHardwareRTS();
  }
  else
  {
    Serial.println("I2C UART3 init failed");
  }

  if(I2CUART4.begin_I2C(I2CUART4_ADD))
  {
    //Serial.println("I2C UART4 init success");
    I2CUART4.disableTCR_TLR();
    I2CUART4.enableAutors485();
    I2CUART4.begin_UART(UART_BAUD);
    I2CUART4.enableHardwareRTS();
  }
  else
  {
    Serial.println("I2C UART4 init failed");
  }

  if(I2CUART5.begin_I2C(I2CUART5_ADD))
  {
    //Serial.println("I2C UART5 init success");
    I2CUART5.disableTCR_TLR();
    I2CUART5.enableAutors485();
    I2CUART5.begin_UART(UART_BAUD);
    I2CUART5.enableHardwareRTS();
  }
  else
  {
    Serial.println("I2C UART5 init failed");
  }

  if(I2CUART6.begin_I2C(I2CUART6_ADD))
  {
    //Serial.println("I2C UART6 init success");
    I2CUART6.disableTCR_TLR();
    I2CUART6.enableAutors485();
    I2CUART6.begin_UART(UART_BAUD);
    I2CUART6.enableHardwareRTS();
  }
  else
  {
    Serial.println("I2C UART6 init failed");
  }

  constexpr uint8_t test_loops = 3; //number of test loops

  Serial.println("Starting UART test...");

  for(uint8_t i=0; i<test_loops; i++)
  {
    I2CUART1.printf("UART1 loop %d\n", i);
    delay(100);
    if(I2CUART2.readStringUntil('\n') != String("UART1 loop " + String(i)))
    {
      Serial.println("UART1 to UART2 communication failed");
      test_result = false;
    }
    if(I2CUART3.readStringUntil('\n') != String("UART1 loop " + String(i)))
    {
      Serial.println("UART1 to UART3 communication failed");
      test_result = false;
    }
    if(I2CUART4.readStringUntil('\n') != String("UART1 loop " + String(i)))
    {
      Serial.println("UART1 to UART4 communication failed");
      test_result = false;
    }
    if(I2CUART5.readStringUntil('\n') != String("UART1 loop " + String(i)))
    {
      Serial.println("UART1 to UART5 communication failed");
      test_result = false;
    }
    if(I2CUART6.readStringUntil('\n') != String("UART1 loop " + String(i)))
    {
      Serial.println("UART1 to UART6 communication failed");
      test_result = false;
    }
  }

  for(uint8_t i=0; i<test_loops; i++)
  {
    I2CUART2.printf("UART2 loop %d\n", i);
    delay(100);
    if(I2CUART1.readStringUntil('\n') != String("UART2 loop " + String(i)))
    {
      Serial.println("UART2 to UART1 communication failed");
      test_result = false;
    }
    if(I2CUART3.readStringUntil('\n') != String("UART2 loop " + String(i)))
    {
      Serial.println("UART2 to UART3 communication failed");
      test_result = false;
    }
    if(I2CUART4.readStringUntil('\n') != String("UART2 loop " + String(i)))
    {
      Serial.println("UART2 to UART4 communication failed");
      test_result = false;
    }
    if(I2CUART5.readStringUntil('\n') != String("UART2 loop " + String(i)))
    {
      Serial.println("UART2 to UART5 communication failed");
      test_result = false;
    }
    if(I2CUART6.readStringUntil('\n') != String("UART2 loop " + String(i)))
    {
      Serial.println("UART2 to UART6 communication failed");
      test_result = false;
    }
  }

  for(uint8_t i=0; i<test_loops; i++)
  {
    I2CUART3.printf("UART3 loop %d\n", i);
    delay(100);
    if(I2CUART1.readStringUntil('\n') != String("UART3 loop " + String(i)))
    {
      Serial.println("UART3 to UART1 communication failed");
      test_result = false;
    }
    if(I2CUART2.readStringUntil('\n') != String("UART3 loop " + String(i)))
    {
      Serial.println("UART3 to UART2 communication failed");
      test_result = false;
    }
    if(I2CUART4.readStringUntil('\n') != String("UART3 loop " + String(i)))
    {
      Serial.println("UART3 to UART4 communication failed");
      test_result = false;
    }
    if(I2CUART5.readStringUntil('\n') != String("UART3 loop " + String(i)))
    {
      Serial.println("UART3 to UART5 communication failed");
      test_result = false;
    }
    if(I2CUART6.readStringUntil('\n') != String("UART3 loop " + String(i)))
    {
      Serial.println("UART3 to UART6 communication failed");
      test_result = false;
    }
  }

  for(uint8_t i=0; i<test_loops; i++)
  {
    I2CUART4.printf("UART4 loop %d\n", i);
    delay(100);
    if(I2CUART1.readStringUntil('\n') != String("UART4 loop " + String(i)))
    {
      Serial.println("UART4 to UART1 communication failed");
      test_result = false;
    }
    if(I2CUART2.readStringUntil('\n') != String("UART4 loop " + String(i)))
    {
      Serial.println("UART4 to UART2 communication failed");
      test_result = false;
    }
    if(I2CUART3.readStringUntil('\n') != String("UART4 loop " + String(i)))
    {
      Serial.println("UART4 to UART3 communication failed");
      test_result = false;
    }
    if(I2CUART5.readStringUntil('\n') != String("UART4 loop " + String(i)))
    {
      Serial.println("UART4 to UART5 communication failed");
      test_result = false;
    }
    if(I2CUART6.readStringUntil('\n') != String("UART4 loop " + String(i)))
    {
      Serial.println("UART4 to UART6 communication failed");
      test_result = false;
    }
  }

  for(uint8_t i=0; i<test_loops; i++)
  {
    I2CUART5.printf("UART5 loop %d\n", i);
    delay(100);
    if(I2CUART1.readStringUntil('\n') != String("UART5 loop " + String(i)))
    {
      Serial.println("UART5 to UART1 communication failed");
      test_result = false;
    }
    if(I2CUART2.readStringUntil('\n') != String("UART5 loop " + String(i)))
    {
      Serial.println("UART5 to UART2 communication failed");
      test_result = false;
    }
    if(I2CUART3.readStringUntil('\n') != String("UART5 loop " + String(i)))
    {
      Serial.println("UART5 to UART3 communication failed");
      test_result = false;
    }
    if(I2CUART4.readStringUntil('\n') != String("UART5 loop " + String(i)))
    {
      Serial.println("UART5 to UART4 communication failed");
      test_result = false;
    }
    if(I2CUART6.readStringUntil('\n') != String("UART5 loop " + String(i)))
    {
      Serial.println("UART5 to UART6 communication failed");
      test_result = false;
    }
  }

  for(uint8_t i=0; i<test_loops; i++)
  {
    I2CUART6.printf("UART6 loop %d\n", i);
    delay(100);
    if(I2CUART1.readStringUntil('\n') != String("UART6 loop " + String(i)))
    {
      Serial.println("UART6 to UART1 communication failed");
      test_result = false;
    }
    if(I2CUART2.readStringUntil('\n') != String("UART6 loop " + String(i)))
    {
      Serial.println("UART6 to UART2 communication failed");
      test_result = false;
    }
    if(I2CUART3.readStringUntil('\n') != String("UART6 loop " + String(i)))
    {
      Serial.println("UART6 to UART3 communication failed");
      test_result = false;
    }
    if(I2CUART4.readStringUntil('\n') != String("UART6 loop " + String(i)))
    {
      Serial.println("UART6 to UART4 communication failed");
      test_result = false;
    }
    if(I2CUART5.readStringUntil('\n') != String("UART6 loop " + String(i)))
    {
      Serial.println("UART6 to UART5 communication failed");
      test_result = false;
    }
  }



  if(test_result)
  {
    Serial.println("UART Test Passed");
  }
  else
  {
    Serial.println("UART Test Failed==============================================================================!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  }

} 

void InitL9822E(void)
{
  spiL9822.begin(DRV_SCK, DRV_MISO, DRV_MOSI);
  Driver.begin(spiL9822, 1000000); // 1 MHz (до 2 MHz по datasheet)

  const uint16_t values[] = {4250, 2925, 2250, 1837, 1561, 1363, 1214, 1100}; // Example PWM values for channels
  const uint8_t deviation = 7; // 7 percent deviation allowed

  const uint16_t solenoid_power_value = 7465; // 

  const uint16_t ai_switch_up = 3210;
  const uint16_t ai_switch_down = 7677;

  Serial.println("Solenoid output Test Start");

  bool test_passed = true;

  for (size_t i = 0; i < 8; i++)
  {
    Driver.setChannel(0, i, true);
    Driver.write();
    delay(100);

    //PrintAIChannel(4);

    if(!CheckValueDeviation(values[i], ReadAIChanelmV(4), deviation))
    {
      Serial.println("Solenoid output Test Failed at channel " + String(i));
      test_passed = false;
    }
  }

  Driver.setAll(false);
  Driver.write();

  for (size_t i = 0; i < 8; i++)
  {
    Driver.setChannel(1, i, true);
    Driver.write();
    delay(100);

    //PrintAIChannel(0);

    if(!CheckValueDeviation(values[i], ReadAIChanelmV(0), deviation))
    {
      Serial.println("Solenoid output Test Failed at channel " + String(i+8));
      test_passed = false;
    }
  }
  
  Driver.setAll(false);
  Driver.write();

  if(!CheckValueDeviation(solenoid_power_value, ReadAIChanelmV(5), deviation))
  {
    Serial.println("Solenoid output 0-7 Power Test Failed");
    test_passed = false;
  }

  if(!CheckValueDeviation(solenoid_power_value, ReadAIChanelmV(1), deviation))
  {
    Serial.println("Solenoid output 8-15 Power Test Failed");
    test_passed = false;
  }

  if(!CheckValueDeviation(solenoid_power_value, ReadAIChanelmV(7), deviation))
  {
    Serial.println("AI 4-7 24V Power Supply Test Failed");
    test_passed = false;
  }

  if(!CheckValueDeviation(solenoid_power_value, ReadAIChanelmV(3), deviation))
  {
    Serial.println("AI 0-3 24V Power Supply Test Failed");
    test_passed = false;
  }

  if ((!CheckValueDeviation(ai_switch_up, ReadAIChanelmV(2), deviation)||(!CheckValueDeviation(ai_switch_up, ReadAIChanelmV(6), deviation))))
  {
    while ((CheckValueDeviation(ai_switch_down, ReadAIChanelmV(2), deviation)&&(CheckValueDeviation(ai_switch_down, ReadAIChanelmV(6), deviation))))
    {
      Serial.println("Please set the AI switch to UP position, if already set, check for PCB faults");
      delay(1000);
    }

    if ((!CheckValueDeviation(ai_switch_up, ReadAIChanelmV(2), deviation)||(!CheckValueDeviation(ai_switch_up, ReadAIChanelmV(6), deviation))))
    {
      Serial.println("AI Switch UP position Test Failed");
      test_passed = false;
    }
  }

  if ((!CheckValueDeviation(ai_switch_down, ReadAIChanelmV(2), deviation)&&(!CheckValueDeviation(ai_switch_down, ReadAIChanelmV(6), deviation))))
  {
    while ((CheckValueDeviation(ai_switch_up, ReadAIChanelmV(2), deviation)&&(CheckValueDeviation(ai_switch_up, ReadAIChanelmV(6), deviation))))
    {
      Serial.println("Please set the AI switch to DOWN position, if already set, check for PCB faults");
      delay(1000);
    }

    if ((!CheckValueDeviation(ai_switch_down, ReadAIChanelmV(2), deviation)&&(!CheckValueDeviation(ai_switch_down, ReadAIChanelmV(6), deviation))))
    {
      Serial.println("AI Switch DOWN position Test Failed");
      test_passed = false;
    }
  }

  
  if(test_passed)
  {
    Serial.println("Solenoid output Test Passed");
  }
  else
  {
    Serial.println("Solenoid output Test Failed ======================================================================!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
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

void PrintAllAIChannels(void)
{
  Serial.print("AI Voltage: ");
  for(int i = 0; i < 8; i++)
  {
    uint16_t tADCvalue = ReadADCChannel(i);
    Serial.print("AI" + String(i) + ":" + String(adctomv(tADCvalue)) + "mV ");
  }
  Serial.println();
  Serial.flush();
}

void PrintAIChannel(uint8_t channel)
{
  if(channel<8)
  {
    uint16_t tADCvalue = ReadADCChannel(channel);
    Serial.print("AI" + String(channel) + ":" + String(adctomv(tADCvalue)) + "mV ");
    Serial.println();
    Serial.flush();
  }
}

uint16_t ReadAIChanelmV(uint8_t channel)
{
  uint16_t tADCvalue = ReadADCChannel(channel);
  return adctomv(tADCvalue);
}

uint16_t adctomv(uint16_t adc)
{
  return (adc * referenceVoltage / adcResolution) * scaleFactor;
}

bool CheckValueDeviation(uint16_t value1, uint16_t value2, uint8_t percent)
{
  float deviation = ((float)percent) / 100.0;
  float lower_limit = value1 * (1.0 - deviation);
  float upper_limit = value1 * (1.0 + deviation);

  if((value2 >= lower_limit) && (value2 <= upper_limit))
  {
    return true;
  }
  else
  {
    return false;
  }
}

void initDIO(void)
{
  for (int i = 0; i < 16; i++)
  {
    mcp_0.pinMode(i, INPUT);
  }

  pinMode(SSR0_PIN, OUTPUT);
  pinMode(SSR1_PIN, OUTPUT);  
  pinMode(SSR2_PIN, OUTPUT);
  pinMode(SSR3_PIN, OUTPUT);
  digitalWrite(SSR0_PIN, LOW);
  digitalWrite(SSR1_PIN, LOW);  
  digitalWrite(SSR2_PIN, LOW);
  digitalWrite(SSR3_PIN, LOW);  

  Serial.println("DIO/SSR Test Start");

  bool test_passed = true;

  for(int i = 0; i < 16; i++)
  {
    if(mcp_0.digitalRead(i) != HIGH)
    {
      Serial.println("DIO Test Failed at DI" + String(i));
      test_passed = false;
    }
  }

  digitalWrite(SSR0_PIN, HIGH);
  delay(100); 
  //Pint all DI states
  //PrintAllDIChannels();
  if
  ((mcp_0.digitalRead(0) != LOW)||
    (mcp_0.digitalRead(1) != HIGH)||
    (mcp_0.digitalRead(2) != HIGH)||
    (mcp_0.digitalRead(3) != HIGH)||
    (mcp_0.digitalRead(4) != LOW)||
    (mcp_0.digitalRead(5) != HIGH)||
    (mcp_0.digitalRead(6) != HIGH)||
    (mcp_0.digitalRead(7) != HIGH)||
    (mcp_0.digitalRead(8) != LOW)||
    (mcp_0.digitalRead(9) != HIGH)||
    (mcp_0.digitalRead(10) != HIGH)||
    (mcp_0.digitalRead(11) != HIGH)||
    (mcp_0.digitalRead(12) != LOW)||
    (mcp_0.digitalRead(13) != HIGH)||
    (mcp_0.digitalRead(14) != HIGH)||
    (mcp_0.digitalRead(15) != HIGH))
    {
      Serial.println("SSR0 or DIO, DI4, DI8, DI12 Test Failed, DI0, DI4, DI8, DI12 should be LOW when SSR0 is ON");
      test_passed = false;
    }

  digitalWrite(SSR0_PIN, LOW);
  delay(100);
  digitalWrite(SSR1_PIN, HIGH);
  delay(100);
    if
  ((mcp_0.digitalRead(0) != HIGH)||
    (mcp_0.digitalRead(1) != HIGH)||
    (mcp_0.digitalRead(2) != LOW)||
    (mcp_0.digitalRead(3) != HIGH)||
    (mcp_0.digitalRead(4) != HIGH)||
    (mcp_0.digitalRead(5) != HIGH)||
    (mcp_0.digitalRead(6) != LOW)||
    (mcp_0.digitalRead(7) != HIGH)||
    (mcp_0.digitalRead(8) != HIGH)||
    (mcp_0.digitalRead(9) != HIGH)||
    (mcp_0.digitalRead(10) != LOW)||
    (mcp_0.digitalRead(11) != HIGH)||
    (mcp_0.digitalRead(12) != HIGH)||
    (mcp_0.digitalRead(13) != HIGH)||
    (mcp_0.digitalRead(14) != LOW)||
    (mcp_0.digitalRead(15) != HIGH))
    {
      Serial.println("SSR1 or DI2, DI6, DI10, DI14 Test Failed, DI2, DI6, DI10, DI14 should be LOW when SSR1 is ON");
      test_passed = false;
    }
  digitalWrite(SSR1_PIN, LOW);
  delay(100);
  digitalWrite(SSR2_PIN, HIGH);
  delay(100);
    if
  ((mcp_0.digitalRead(0) != HIGH)||
    (mcp_0.digitalRead(1) != LOW)||
    (mcp_0.digitalRead(2) != HIGH)||
    (mcp_0.digitalRead(3) != HIGH)||
    (mcp_0.digitalRead(4) != HIGH)||
    (mcp_0.digitalRead(5) != LOW)||
    (mcp_0.digitalRead(6) != HIGH)||
    (mcp_0.digitalRead(7) != HIGH)||
    (mcp_0.digitalRead(8) != HIGH)||
    (mcp_0.digitalRead(9) != LOW)||
    (mcp_0.digitalRead(10) != HIGH)||
    (mcp_0.digitalRead(11) != HIGH)||
    (mcp_0.digitalRead(12) != HIGH)||
    (mcp_0.digitalRead(13) != LOW)||
    (mcp_0.digitalRead(14) != HIGH)||
    (mcp_0.digitalRead(15) != HIGH))
    {
      Serial.println("SSR2 or DI1, DI5, DI9, DI13 Test Failed, DI1, DI5, DI9, DI13 should be LOW when SSR2 is ON");
      test_passed = false;
    }
  digitalWrite(SSR2_PIN, LOW);
  delay(100);
  digitalWrite(SSR3_PIN, HIGH);
  delay(100);
      if
  ((mcp_0.digitalRead(0) != HIGH)||
    (mcp_0.digitalRead(1) != HIGH)||
    (mcp_0.digitalRead(2) != HIGH)||
    (mcp_0.digitalRead(3) != LOW)||
    (mcp_0.digitalRead(4) != HIGH)||
    (mcp_0.digitalRead(5) != HIGH)||
    (mcp_0.digitalRead(6) != HIGH)||
    (mcp_0.digitalRead(7) != LOW)||
    (mcp_0.digitalRead(8) != HIGH)||
    (mcp_0.digitalRead(9) != HIGH)||
    (mcp_0.digitalRead(10) != HIGH)||
    (mcp_0.digitalRead(11) != LOW)||
    (mcp_0.digitalRead(12) != HIGH)||
    (mcp_0.digitalRead(13) != HIGH)||
    (mcp_0.digitalRead(14) != HIGH)||
    (mcp_0.digitalRead(15) != LOW))
    {
      Serial.println("SSR3 or DI3, DI7, DI11, DI15 Test Failed, DI3, DI7, DI11, DI15 should be LOW when SSR3 is ON");
      test_passed = false;
    }
  digitalWrite(SSR3_PIN, LOW);
  delay(100);


  if(test_passed)
  {
    Serial.println("DIO/SSR Test Passed");
  }
  else
  {
    Serial.println("DIO/SSR Test Failed ======================================================================!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  }
  
}

void PrintAllDIOStates(void)
{
  for (int i = 0; i < 16; i++)
  {
    Serial.print("DI" + String(i) + ":");
    mcp_0.digitalRead(i) == HIGH ? Serial.print("HIGH ") : Serial.print("LOW ");
    delay(1);
  }
  Serial.println();
  Serial.flush();
}

void initButtons(void)
{

  Serial.println("Button Test Start, press buttons to see LED color change");

  mcp_1.pinMode(0, INPUT);
  mcp_1.pinMode(1, INPUT);
  mcp_1.pinMode(2, INPUT);
  mcp_1.pinMode(3, INPUT);
  pinMode(SETUP_BUTTON_PIN, INPUT); // onboard button

  uint8_t Button_states = 0;
  uint8_t prev_Button_states = 0;

  while (true)
  {
    if(mcp_1.digitalRead(0) == LOW)Button_states = 1;
    else if(mcp_1.digitalRead(1) == LOW)Button_states = 2;
    else if(mcp_1.digitalRead(2) == LOW)Button_states = 3;
    else if(mcp_1.digitalRead(3) == LOW)Button_states = 4;
    else if(digitalRead(SETUP_BUTTON_PIN) == LOW)Button_states = 5;
    else Button_states = 0;

    
    if(prev_Button_states != Button_states)
    {
      prev_Button_states = Button_states;
      switch (Button_states)
      {
      case 0:
      {
        for (uint16_t i = 0; i < PixelCount; i++)
        {
          pixel.SetPixelColor(i, RgbColor(0, 0, 0));
        }
        pixel.Show();
        Serial.println("Button released, LEDs OFF");
      }break;

      case 1:
      {
        for (uint16_t i = 0; i < PixelCount; i++)
        {
          pixel.SetPixelColor(i, RgbColor(PixelBrightness, 0, 0));
        }
        pixel.Show();
        Serial.println("Button 4 pressed, LEDs RED");
      }break;

      case 2:
      {
        for (uint16_t i = 0; i < PixelCount; i++)
        {
          pixel.SetPixelColor(i, RgbColor(0, PixelBrightness, 0));
        }
        pixel.Show();
        Serial.println("Button 3 pressed, LEDs GREEN");
      }break;

      case 3:
      {
        for (uint16_t i = 0; i < PixelCount; i++)
        {
          pixel.SetPixelColor(i, RgbColor(0, 0, PixelBrightness));
        }
        pixel.Show();
        Serial.println("Button 2 pressed, LEDs BLUE");
      }break;

      case 4:
      {
        for (uint16_t i = 0; i < PixelCount; i++)
        {
          pixel.SetPixelColor(i, RgbColor(PixelBrightness, PixelBrightness, PixelBrightness));
        }
        pixel.Show();
        Serial.println("Button 1 pressed, LEDs WHITE");
      }break;

      case 5:
      {
        for (uint16_t i = 0; i < PixelCount; i++)
        {
          pixel.SetPixelColor(i, RgbColor(PixelBrightness, PixelBrightness, 0));
        }
        pixel.Show();
        Serial.println("Setup Button pressed, LEDs YELLOW");
      }break;

      
      default:
        break;
      }

    }




  }
  

}

void PrintAllDIChannels(void)
{
  Serial.print("DI States: ");
  for(int i = 0; i < 16; i++)
  {
    mcp_0.digitalRead(i) == HIGH ? Serial.print("DI" + String(i) + ":HIGH ") : Serial.print("DI" + String(i) + ":LOW ");
  }
  Serial.println();
  Serial.flush();
}