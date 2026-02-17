#include <Arduino.h>

#include "KMPCommon.h"
#include "KMPProDinoESP32.h"

#include "RAK3172.h"

//RAK3172 LoRa(true);//debug
RAK3172 LoRa(false);//no debug

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
  uint8_t rxMessage[100];

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
void ReturnError(ErrorCode errorCode);
void ReturnDIValue(void);
void ReturnRelayValue(void);
void ReturnLoRaVersion(void);
void SetRelayValue(void);
bool SendMsg(uint8_t* txMessage, size_t txMessageSize);


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

  if(LoRa.receiveMassage(rxMessage, sizeof(rxMessage), &msglen, RAK3172_TIMEOUT_300))
  {
    board.setStatusLed(blue);
    Serial.print("Received: ");
    for(int i = 0; i < msglen; i++)
    {
      Serial.printf("%02X ", rxMessage[i]);
    }
    Serial.println();
    Serial.flush();

    if(MASTER_ADDRESS != rxMessage[0])
    {
      Serial.println("Invalid Master Address");
      Serial.flush();
      //ReturnError(ERROR_INVALID_MASTER_ADDRESS);
    }
    else if(SLAVE_ADDRESS != rxMessage[1])
    {
      Serial.println("Invalid Slave Address");
      Serial.flush();
      //ReturnError(ERROR_INVALID_SLAVE_ADDRESS);
    }
    else
    {
      switch(rxMessage[2])
      {
        case CMD_READ_DI:
          ReturnDIValue();
          break;
        case CMD_READ_RELAY:
          ReturnRelayValue();
          break;
        case CMD_SET_RELAY:
          SetRelayValue();
          break;
        case CMD_READ_LORA_VER:
          ReturnLoRaVersion();
          break;
        default:
          ReturnError(ERROR_INVALID_CMD);
          break;
      }
    }

    board.setStatusLed(black);
  }

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

  if(LoRa.setReceiveMode(RAK3172_CONTINUES_RECEIVE_MODE))
  {
    Serial.println("Receive Mode set to Continues Receive Mode");
    Serial.flush();
  }
  else
  {
    Serial.println("Receive Mode set to Continues Receive Mode failed");
    Serial.flush();
  }
}

bool SendMsg(uint8_t* txMessage, size_t txMessageSize)
{
  bool status = true;
  LoRa.setReceiveMode(RAK3172_TX_MODE);
  if(!LoRa.transmitMessage(txMessage, txMessageSize))status = false;
  LoRa.setReceiveMode(RAK3172_CONTINUES_RECEIVE_MODE);
  return status;
}

void ReturnError(ErrorCode errorCode)
{
  uint8_t errorMessage[]={SLAVE_ADDRESS, MASTER_ADDRESS, CMD_ERROR, errorCode};

  if(SendMsg(errorMessage, sizeof(errorMessage)))
  {
    Serial.println("Error Message Sent");
    Serial.flush();
  }
  else
  {
    Serial.println("Error Message Failed to Send");
    Serial.flush();
  }
}

void ReturnDIValue(void)
{
  uint8_t diValues[]={SLAVE_ADDRESS, MASTER_ADDRESS, CMD_READ_DI, 0x00};

  diValues[3] = board.getOptoInState();

  if(SendMsg(diValues, sizeof(diValues)))
  {
    Serial.println("Read DI Values Sent");
    Serial.flush();
  }
  else
  {
    Serial.println("Read DI Values Failed to Send");
    Serial.flush();
  }

}

void ReturnRelayValue(void)
{
  uint8_t relayValues[]={SLAVE_ADDRESS, MASTER_ADDRESS, CMD_READ_RELAY, 0x00};

  relayValues[3] = board.getRelayState();

  if(SendMsg(relayValues, sizeof(relayValues)))
  {
    Serial.println("Read Relay Values Sent");
    Serial.flush();
  }
  else
  {
    Serial.println("Read Relay Values Failed to Send");
    Serial.flush();
  }

}

void ReturnLoRaVersion(void)
{
  String firmwareVersion = LoRa.getVersion();
  Serial.println("LoRa Version: " + firmwareVersion);

    // Намиране на индекси за разделителите
    int firstUnderscore = firmwareVersion.indexOf('_');
    int firstDot = firmwareVersion.indexOf('.', firstUnderscore);
    int secondDot = firmwareVersion.indexOf('.', firstDot + 1);
    
    // Извличане на числата
    uint8_t major = firmwareVersion.substring(firstUnderscore + 1, firstDot).toInt();
    uint8_t minor = firmwareVersion.substring(firstDot + 1, secondDot).toInt();
    uint8_t patch = firmwareVersion.substring(secondDot + 1, firmwareVersion.indexOf('_', secondDot)).toInt();

    uint8_t versionValues[]={SLAVE_ADDRESS, MASTER_ADDRESS, CMD_READ_LORA_VER, major, minor, patch};

    if(SendMsg(versionValues, sizeof(versionValues)))
    {
      Serial.println("LoRa Version Values Sent");
      Serial.flush();
    }
    else
    {
      Serial.println("LoRa Version Values Failed to Send");
      Serial.flush();
    }
}

void SetRelayValue(void)
{
  for(int i = 0; i < RELAY_COUNT; i++)
  {
    board.setRelayState(i, rxMessage[3]&1<<i);
  }

  uint8_t relayValues[]={SLAVE_ADDRESS, MASTER_ADDRESS, CMD_SET_RELAY, rxMessage[3]};

  if(SendMsg(relayValues, sizeof(relayValues)))
  {
    Serial.println("Set Relay Values Sent");
    Serial.flush();
  }
  else
  {
    Serial.println("Set Relay Values Failed to Send");
    Serial.flush();
  }

}
