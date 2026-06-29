// RAK3172_Test_Slave - RAK3172 P2P production / functional test, SLAVE side.
//
// Purpose:
//   Test jig for RAK3172 modules. A module-under-test is fitted to this ProDino;
//   the firmware initialises it and echoes the master's PING messages back as PONG.
//   It exercises ONLY the RAK3172 (no relays, no opto inputs, no ADC).
//
//   Pair this with the RAK3172_Test_Master example (the reference station that
//   sends the PINGs).
//
// Status LED reports whether the fitted RAK3172 works:
//   RED    - RAK3172 not responding on UART (begin()/config failed) -> hardware fault
//   ORANGE - RAK3172 alive on UART, but no radio link yet (no valid PING received)
//   GREEN  - full pass: RAK3172 answers AND a PING/PONG radio round-trip succeeded
//   BLUE   - brief blink on every successful exchange
//
//   If the module never comes up (RED), the firmware keeps re-initialising it, so a
//   freshly fitted module is picked up automatically without a reset.
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

#define LINK_TIMEOUT 5000  // ms without a valid PING -> radio link considered down
#define RETRY_PERIOD 2000  // ms between re-init attempts while the module is dead

size_t rxLen;
uint8_t rxMessage[16];

bool radioOk = false;            // RAK3172 responds and is configured on UART
unsigned long lastCommTime = 0;  // millis() of the last valid PING/PONG exchange

// Function prototypes
bool loraInit(void);
bool sendMsg(uint8_t* msg, size_t size);
void showStatus(void);

void setup()
{
  Serial.begin(115200);

  board.begin(BOARD_TYPE);
  board.setStatusLed(orange);  // initialising
  delay(1000);

  Serial.println("RAK3172 test - SLAVE (test jig)");
  Serial.flush();

  radioOk = loraInit();
  showStatus();
}

void loop()
{
  // Module not responding (or just replaced) -> keep trying to bring it up.
  if (!radioOk)
  {
    showStatus();  // RED
    delay(RETRY_PERIOD);
    radioOk = loraInit();
    return;
  }

  // Listen for a PING and echo it back as PONG.
  // rxLen is left untouched by the driver on a receive timeout, so clear it first.
  rxLen = 0;
  if (LoRa.receiveMassage(rxMessage, sizeof(rxMessage), &rxLen, RAK3172_TIMEOUT_300) &&
      rxLen >= 3 &&
      rxMessage[0] == MSG_MAGIC &&
      rxMessage[1] == CMD_PING)
  {
    uint8_t seq = rxMessage[2];
    Serial.printf("PING seq=%u received -> PONG\n", seq);
    Serial.flush();

    board.setStatusLed(blue);  // activity blink (held during the TX below)

    uint8_t pong[] = { MSG_MAGIC, CMD_PONG, seq };
    if (sendMsg(pong, sizeof(pong)))
    {
      lastCommTime = millis();  // full radio round-trip succeeded
    }
  }

  showStatus();
}

// Drive the status LED from the current module / link state.
void showStatus(void)
{
  if (!radioOk)
  {
    board.setStatusLed(red);                                  // module fault
  }
  else if (lastCommTime != 0 && (millis() - lastCommTime) < LINK_TIMEOUT)
  {
    board.setStatusLed(green);                                // full pass
  }
  else
  {
    board.setStatusLed(orange);                               // alive, no RF link
  }
}

// Switch to TX, send the message, then return to continuous receive.
bool sendMsg(uint8_t* msg, size_t size)
{
  bool status = true;
  LoRa.setReceiveMode(RAK3172_TX_MODE);
  if (!LoRa.transmitMessage(msg, size)) status = false;
  LoRa.setReceiveMode(RAK3172_CONTINUES_RECEIVE_MODE);
  return status;
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

  // Slave stays in continuous receive, listening for the master's PINGs.
  if (!LoRa.setReceiveMode(RAK3172_CONTINUES_RECEIVE_MODE))
  {
    Serial.println("Set continuous receive mode failed.");
    Serial.flush();
    return false;
  }

  return true;
}
