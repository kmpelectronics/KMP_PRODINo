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


// Function Prototypes
void createledtask(void);
void InitEthernet(void);
void LoRaInit(void);
void InitDisplay(void);
void initADC(void);
void ds18b20Test(void);
void initUart(void);




void setup() {

  Serial.begin(115200);
  delay(500);
  Serial.println("NCS SCC+ Controller V1 Starting...");

  pixel.Begin();
  pixel.Show();

  Wire.setPins(SDA1, SCL1);
  I2C2.begin(SDA2, SCL2);


  mcp_0.begin_I2C(0x20, &I2C2);
  mcp_1.begin_I2C(0x21, &I2C2);  


  

  // initUart();
  // createledtask();
  // InitEthernet();
  // LoRaInit();
  // initADC();
  // InitDisplay();
  // ds18b20Test();

    
  
  //InitRelay();

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
  //initDIO();
  //initButtons();
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

void LoRaInit(void)
{
   Serial.println("Starting RAK3172");
   Serial.flush();

  if(LoRa.begin(Lora_rx, Lora_tx, Lora_rst))
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

void InitDisplay(void)
{
    
    delay(1);
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
        Serial.println(F("display allocation failed"));
        for (;;); // Don't proceed, loop forever
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

void ds18b20Test(void) {
  byte i;
  byte present = 0;
  byte type_s;
  byte data[9];
  byte addr[8];
  float celsius, fahrenheit;

  while (ds.search(addr))
  {
      Serial.print("ROM =");
      for( i = 0; i < 8; i++) {
      //Serial.write(' ');
      Serial.print(addr[i], HEX);
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
    Serial.print(" T = ");
    Serial.println(celsius);
    // Serial.print(" Celsius, ");
    // Serial.print(fahrenheit);
    // Serial.println(" Fahrenheit");

  }

  Serial.println("No more addresses.");
  Serial.println();
  ds.reset_search();
  
  
  
  // if ( !ds.search(addr)) {
  //   Serial.println("No more addresses.");
  //   Serial.println();
  //   ds.reset_search();
  //   delay(250);
  //   return;
  // }
  
  
}



void initUart(void)
{

  pinMode(UART_RESET, OUTPUT);
  digitalWrite(UART_RESET, LOW);
  delay(100);
  digitalWrite(UART_RESET, HIGH);
  delay(100);

  if (I2CUART1.begin_I2C(I2CUART1_ADD))
  {
    Serial.println("I2C UART1 init success");
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
    Serial.println("I2C UART2 init success");
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
    Serial.println("I2C UART3 init success");
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
    Serial.println("I2C UART4 init success");
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
    Serial.println("I2C UART5 init success");
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
    Serial.println("I2C UART6 init success");
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

  for(uint8_t i=0; i<test_loops; i++)
  {
    I2CUART1.printf("UART1 loop %d\n", i);
    Serial.print("UART2 RX: ");
    while (I2CUART2.available())
      Serial.write(I2CUART2.read());
    Serial.print("UART3 RX: ");
    while (I2CUART3.available())
      Serial.write(I2CUART3.read());
    Serial.print("UART4 RX: ");
    while (I2CUART4.available())
      Serial.write(I2CUART4.read());
    Serial.print("UART5 RX: ");
    while (I2CUART5.available())
      Serial.write(I2CUART5.read());
    Serial.print("UART6 RX: ");
    while (I2CUART6.available())
      Serial.write(I2CUART6.read());
    delay(100);
  }

  for(uint8_t i=0; i<test_loops; i++)
  {
    I2CUART2.printf("UART2 loop %d\n", i);
    Serial.print("UART1 RX: ");
    while (I2CUART1.available())
      Serial.write(I2CUART1.read());
    Serial.print("UART3 RX: ");
    while (I2CUART3.available())
      Serial.write(I2CUART3.read());
    Serial.print("UART4 RX: ");
    while (I2CUART4.available())
      Serial.write(I2CUART4.read());
    Serial.print("UART5 RX: ");
    while (I2CUART5.available())
      Serial.write(I2CUART5.read());
    Serial.print("UART6 RX: ");
    while (I2CUART6.available())
      Serial.write(I2CUART6.read());
    delay(100);
  }

  for(uint8_t i=0; i<test_loops; i++)
  {
    I2CUART3.printf("UART3 loop %d\n", i);
    Serial.print("UART1 RX: ");
    while (I2CUART1.available())
      Serial.write(I2CUART1.read());
    Serial.print("UART2 RX: ");
    while (I2CUART2.available())
      Serial.write(I2CUART2.read());
    Serial.print("UART4 RX: ");
    while (I2CUART4.available())
      Serial.write(I2CUART4.read());
    Serial.print("UART5 RX: ");
    while (I2CUART5.available())
      Serial.write(I2CUART5.read());
    Serial.print("UART6 RX: ");
    while (I2CUART6.available())
      Serial.write(I2CUART6.read());
    delay(100);
  }

  for(uint8_t i=0; i<test_loops; i++)
  {
    I2CUART4.printf("UART4 loop %d\n", i);
    Serial.print("UART1 RX: ");
    while (I2CUART1.available())
      Serial.write(I2CUART1.read());
    Serial.print("UART2 RX: ");
    while (I2CUART2.available())
      Serial.write(I2CUART2.read());
    Serial.print("UART3 RX: ");
    while (I2CUART3.available())
      Serial.write(I2CUART3.read());
    Serial.print("UART5 RX: ");
    while (I2CUART5.available())
      Serial.write(I2CUART5.read());
    Serial.print("UART6 RX: ");
    while (I2CUART6.available())
      Serial.write(I2CUART6.read());
    delay(100);
  }

  for(uint8_t i=0; i<test_loops; i++)
  {
    I2CUART5.printf("UART5 loop %d\n", i);
    Serial.print("UART1 RX: ");
    while (I2CUART1.available())
      Serial.write(I2CUART1.read());
    Serial.print("UART2 RX: ");
    while (I2CUART2.available())
      Serial.write(I2CUART2.read());
    Serial.print("UART3 RX: ");
    while (I2CUART3.available())
      Serial.write(I2CUART3.read());
    Serial.print("UART4 RX: ");
    while (I2CUART4.available())
      Serial.write(I2CUART4.read());
    Serial.print("UART6 RX: ");
    while (I2CUART6.available())
      Serial.write(I2CUART6.read());
    delay(100);
  }

  for(uint8_t i=0; i<test_loops; i++)
  {
    I2CUART6.printf("UART6 loop %d\n", i);
    Serial.print("UART1 RX: ");
    while (I2CUART1.available())
      Serial.write(I2CUART1.read());
    Serial.print("UART2 RX: ");
    while (I2CUART2.available())
      Serial.write(I2CUART2.read());
    Serial.print("UART3 RX: ");
    while (I2CUART3.available())
      Serial.write(I2CUART3.read());
    Serial.print("UART4 RX: ");
    while (I2CUART4.available())
      Serial.write(I2CUART4.read());
    Serial.print("UART5 RX: ");
    while (I2CUART5.available())
      Serial.write(I2CUART5.read());
    delay(100);
  }

} 


