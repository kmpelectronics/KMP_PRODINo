// RAK3172_Test_Master - RAK3172 P2P production / functional test, MASTER side.
//
// Purpose:
//   Minimal test that exercises ONLY the on-board RAK3172 LoRa module (no relays,
//   no opto inputs, no ADC). This unit is the fixed REFERENCE station: it is fitted
//   with a known-good RAK3172 and continuously sends PING messages, expecting the
//   slave (the test jig with the module-under-test) to echo them back as PONG.
//
//   Pair this with the RAK3172_Test_Slave example.
//
// Status LED (this board's own RAK3172 + link to the slave):
//   ORANGE - initialising / no valid response from the slave
//   RED    - local RAK3172 fault (begin()/configuration failed)
//   GREEN  - local RAK3172 OK and the slave is answering
//   BLUE   - brief blink on every successful PING/PONG round-trip
//
// Board / wiring: built around KMPProDinoESP32Class. Set BOARD_TYPE to match your
// ProDino ESP32 LoRa variant. The RAK3172 uses the library default pins/UART.

#include <Arduino.h>

#include "KMPCommon.h"
#include "KMPProDinoESP32.h"

#include "RAK3172.h"

// Set this to the ProDino variant you are using (must include the LoRa module).
#define BOARD_TYPE ProDino_ESP32_Ethernet

// RAK3172 LoRa(true) -> print AT traffic on Serial for debugging.
RAK3172 LoRa(false);

KMPProDinoESP32Class board;

// LoRa P2P radio parameters - MUST be identical on master and slave.
uint32_t frequency = 868000000;
uint8_t sf = 7;
uint8_t bw = 0;
uint8_t cr = 0;
uint8_t prlen = 10;
uint8_t pwr = 14;

// Simple test protocol: { MAGIC, command, sequence }
#define MSG_MAGIC 0xA5
typedef enum {
  CMD_PING = 0x01,
  CMD_PONG = 0x02
} TestCmd;

#define PING_TIMEOUT 1000  // ms to wait for the slave's echo
#define PING_PERIOD  1000  // ms between test cycles

size_t rxLen;
uint8_t rxMessage[16];
uint8_t seq = 0;

// Function prototypes
bool loraInit(void);
bool pingSlave(void);

void setup()
{
  Serial.begin(115200);

  board.begin(BOARD_TYPE);
  board.setStatusLed(orange);  // initialising
  delay(1000);

  Serial.println("RAK3172 test - MASTER");
  Serial.flush();

  if (loraInit())
  {
    board.setStatusLed(green);
    Serial.println("MASTER RAK3172 ready.");
  }
  else
  {
    board.setStatusLed(red);   // local module fault - cannot run the test
    Serial.println("MASTER RAK3172 FAILED - check the module!");
  }
  Serial.flush();
}

void loop()
{
  if (pingSlave())
  {
    // Valid echo from the slave -> radio link OK. Blink blue, hold green.
    board.setStatusLed(blue);
    delay(50);
    board.setStatusLed(green);
  }
  else
  {
    // No / invalid response -> the slave (module-under-test) is not answering.
    board.setStatusLed(orange);
  }

  delay(PING_PERIOD);
}

// Send one PING and validate the PONG echoed back by the slave.
bool pingSlave(void)
{
  uint8_t txMessage[] = { MSG_MAGIC, CMD_PING, seq };

  Serial.printf("PING seq=%u ... ", seq);
  Serial.flush();

  // rxLen is left untouched by the driver on a receive timeout, so clear it first.
  rxLen = 0;
  bool ok = LoRa.transmitMessage(txMessage, sizeof(txMessage),
                                 rxMessage, sizeof(rxMessage), &rxLen, PING_TIMEOUT);

  bool valid = ok && rxLen >= 3 &&
               rxMessage[0] == MSG_MAGIC &&
               rxMessage[1] == CMD_PONG &&
               rxMessage[2] == seq;

  if (valid)
  {
    Serial.println("PONG OK");
  }
  else if (ok && rxLen >= 1)
  {
    Serial.print("bad reply: ");
    for (size_t i = 0; i < rxLen; i++) Serial.printf("%02X ", rxMessage[i]);
    Serial.println();
  }
  else
  {
    Serial.println("no response");
  }
  Serial.flush();

  seq++;
  return valid;
}

bool loraInit(void)
{
  Serial.println("Starting RAK3172...");
  Serial.flush();

  if (!LoRa.begin())
  {
    Serial.println("RAK3172 begin() failed - no UART response.");
    Serial.flush();
    return false;
  }
  Serial.println("RAK3172 begin() OK. Version: " + LoRa.getVersion());
  Serial.flush();

  if (!LoRa.setLoRaMode(RAK3172_MODE_LORAP2P))
  {
    Serial.println("Set LoRa P2P mode failed.");
    Serial.flush();
    return false;
  }

  if (!LoRa.setLoRaP2PParameters(frequency, sf, bw, cr, prlen, pwr))
  {
    Serial.println("Set LoRa P2P parameters failed.");
    Serial.flush();
    return false;
  }

  // Master starts in TX mode; transmitMessage() handles RX windows per cycle.
  if (!LoRa.setReceiveMode(RAK3172_TX_MODE))
  {
    Serial.println("Set TX mode failed.");
    Serial.flush();
    return false;
  }

  return true;
}
