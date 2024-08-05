#include <Arduino.h>

#include "KMPCommon.h"
#include "KMPSmarti8ESP32.h"

#include "ADS1X15.h"

#include "RAK3172.h"

#define DEFAULT_CMD_TIMEOUT 1000

//RAK3172 LoRa(true); //true for debug mode
RAK3172 LoRa(false); //false for normal mode

KMPSmarti8ESP32Class board;

ADS1115 ADS1(0x48);
ADS1115 ADS2(0x49);

#define SDA 14
#define SCL 13

#define AI_COUNT 8 
#define DI_COUNT 8
#define RELAY_COUNT 8

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
void readRemoteADC(uint8_t address);
void readRemoteDI(uint8_t address);
void readRemoteRelay(uint8_t address);
void setRemoteRelay(uint8_t address, uint8_t state);
void readLocalADC(void);
void readLocalDI(void);
void readLocalRelay(void);
void setLocalRelay(uint8_t state);




void setup() 
{
  Serial.begin(115200);

  board.begin(SMARTI8_ESP32_Ethernet);
  board.setStatusLed(green);
  board.setAllRelaysOff();

  delay(1000);

  LoRaInit();
  initADC();

  board.setStatusLed(black);
}

void loop() 
{
  readRemoteADC(SLAVE_ADDRESS);
  readLocalADC();

  readRemoteDI(SLAVE_ADDRESS);
  readLocalDI();

  readRemoteRelay(SLAVE_ADDRESS);
  readLocalRelay();

  setRemoteRelay(SLAVE_ADDRESS, 0x55);
  setLocalRelay(0x55);

  readRemoteRelay(SLAVE_ADDRESS);
  readLocalRelay();

  delay(1000);

  setRemoteRelay(SLAVE_ADDRESS, 0x00);
  setLocalRelay(0x00);

  Serial.println("====================================================");
  delay(2000);
}

void readRemoteADC(uint8_t address)
{
  uint8_t txMessage[]={MASTER_ADDRESS,address,CMD_READ_ADC};

  if(!LoRa.transmitMessage(txMessage, sizeof(txMessage), rxMessage, sizeof(rxMessage), &msglen, DEFAULT_CMD_TIMEOUT))
  {
    Serial.println("Message sent failed");
    Serial.flush();
  }
  else
  {
    Serial.print("Remote AI: ");
    for(int i = 3; i < msglen; i+=2)
    {
      Serial.print("AI" + String((i-3)/2) + ":" + String((rxMessage[i]<<8) | rxMessage[i+1]) + "mV ");
    }
    Serial.println();
    Serial.flush();
  }
}

void readRemoteDI(uint8_t address)
{
  uint8_t txMessage[]={MASTER_ADDRESS,address,CMD_READ_DI};

  if(!LoRa.transmitMessage(txMessage, sizeof(txMessage), rxMessage, sizeof(rxMessage), &msglen, DEFAULT_CMD_TIMEOUT))
  {
    Serial.println("Message sent failed");
    Serial.flush();
  }
  else
  {
    Serial.print("Remote DI: ");
    for(int i = 0; i < DI_COUNT; i++)
    {
      Serial.print("DI" + String(i) + ":" + String(rxMessage[3]&(1<<i) ? 1 : 0) + " ");
    }
    Serial.println();
    Serial.flush();
  }
}

void readRemoteRelay(uint8_t address)
{
  uint8_t txMessage[]={MASTER_ADDRESS,address,CMD_READ_RELAY};

  if(!LoRa.transmitMessage(txMessage, sizeof(txMessage), rxMessage, sizeof(rxMessage), &msglen, DEFAULT_CMD_TIMEOUT))
  {
    Serial.println("Message sent failed");
    Serial.flush();
  }
  else
  {
    Serial.print("Remote RELAY: ");
    for(int i = 0; i < RELAY_COUNT; i++)
    {
      Serial.print("RELAY" + String(i) + ":" + String(rxMessage[3]&(1<<i) ? 1 : 0) + " ");
    }
    Serial.println();
    Serial.flush();
  }
}

void setRemoteRelay(uint8_t address, uint8_t state)
{
  uint8_t txMessage[]={MASTER_ADDRESS,address,CMD_SET_RELAY,state};

  if(!LoRa.transmitMessage(txMessage, sizeof(txMessage), rxMessage, sizeof(rxMessage), &msglen, DEFAULT_CMD_TIMEOUT))
  {
    Serial.println("Message sent failed");
    Serial.flush();
  }
  else
  {
    if(rxMessage[3] == CMD_ERROR)
    {
      Serial.print("Error: ");
      switch(rxMessage[4])
      {
        case ERROR_INVALID_CMD:
          Serial.println("Invalid Command");
          break;
        case ERROR_INVALID_SLAVE_ADDRESS:
          Serial.println("Invalid Slave Address");
          break;
        case ERROR_INVALID_MASTER_ADDRESS:
          Serial.println("Invalid Master Address");
          break;
        default:
          Serial.println("Unknown Error");
          break;
      }
    }
    else if(rxMessage[3] == state)
    {
      Serial.println("Relay set successfully");
    }
    else
    {
      Serial.println("Relay set failed");
    }
    Serial.flush();
  }
}

void readLocalADC(void)
{
  Serial.print("Local AI: ");
  for(int i = 0; i < AI_COUNT; i++)
  {
    Serial.print("AI" + String(i) + ":" + String(ReadADCChannel(i)) + "mV ");
  }
  Serial.println();
  Serial.flush();
}

void readLocalDI(void)
{
  Serial.print("Local DI: ");
  for(int i = 0; i < DI_COUNT; i++)
  {
    Serial.print("DI" + String(i) + ":" + String(board.getOptoInState(i)) + " ");
  }
  Serial.println();
  Serial.flush();
}

void readLocalRelay(void)
{
  Serial.print("Local RELAY: ");
  for(int i = 0; i < RELAY_COUNT; i++)
  {
    Serial.print("RELAY" + String(i) + ":" + String(board.getRelayState(i)) + " ");
  }
  Serial.println();
  Serial.flush();
}


void setLocalRelay(uint8_t state)
{

  for(int i = 0; i < RELAY_COUNT; i++)
  {
    board.setRelayState(i, state&1<<i);
  }
  Serial.println("Local Relays set successfully");
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
  Wire.begin(SDA,SCL);

  if(!ADS1.begin())
  {
    Serial.println("ADC1 not initialized");
  }
  if(!ADS2.begin())
  {
    Serial.println("ADC2 not initialized");
  }

  Serial.println("ADC init OK");
  ADS1.setGain(0);
  ADS2.setGain(0);
}

uint16_t ReadADCChannel(uint8_t channel)
{
  uint16_t measured = 0;
  if(channel<4)
  {
    measured = ADS1.readADC(channel); 
  }
  else
  {
    measured = ADS2.readADC(channel-4);
  }

  return (measured * 6)/10;
}
