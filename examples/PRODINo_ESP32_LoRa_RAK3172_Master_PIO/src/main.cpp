#include <Arduino.h>

#include "KMPCommon.h"
#include "KMPProDinoESP32.h"

#include "RAK3172.h"

#define DEFAULT_CMD_TIMEOUT 1000

//RAK3172 LoRa(true); //true for debug mode
RAK3172 LoRa(false); //false for normal mode

KMPProDinoESP32Class board;

#define DI_COUNT 4
#define RELAY_COUNT 4

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
  CMD_READ_DI = 0x12,
  CMD_READ_RELAY = 0x13,
  CMD_SET_RELAY = 0x14,
  CMD_READ_LORA_VER = 0x15,
  CMD_ERROR = 0xFF
} CommandType;


typedef enum {
  ERROR_INVALID_CMD = 0x01,
  ERROR_INVALID_SLAVE_ADDRESS = 0x02,
  ERROR_INVALID_MASTER_ADDRESS = 0x03
} ErrorCode;
// Function Prototypes
void LoRaInit(void);
void readRemoteDI(uint8_t address);
void readRemoteRelay(uint8_t address);
void setRemoteRelay(uint8_t address, uint8_t state);
void readLocalDI(void);
void readLocalRelay(void);
void setLocalRelay(uint8_t state);

void setup() 
{
  Serial.begin(115200);
  
  
  board.begin(ProDino_ESP32_Ethernet);
  board.setStatusLed(green);
  board.setAllRelaysOff();

  delay(1000);

  LoRaInit();

  board.setStatusLed(black);
}

void loop() 
{
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
  delay(1000);

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


