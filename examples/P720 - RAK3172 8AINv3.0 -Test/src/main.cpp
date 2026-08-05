// P720 - RAK3172 8AINv3.0 - Test
// Based on RAK3172_Test_Slave - functional test jig for a combined
// RAK3172 (LoRa) + 8AIN v3.0 (8 analog inputs) board on SMARTI8.
//
// Two subsystems are tested and reported on the two SMARTI8 status LEDs:
//
//   LED 0 - RAK3172 / radio link (PING/PONG with the RAK3172_Test_Master):
//     RED    - RAK3172 not responding on UART (begin()/config failed)
//     ORANGE - RAK3172 alive, but no radio link yet (no valid PING received)
//     GREEN  - RAK3172 answers AND a PING/PONG radio round-trip succeeded
//     BLUE   - brief blink on every successful exchange
//
//   LED 1 - 8AIN v3.0 analog inputs:
//     RED    - ADC not found, or one or more channels failed
//     ORANGE - ADC up, test not run yet
//     GREEN  - all 8 analog inputs passed
//
// 8AIN test method:
//   SMARTI8 has 8 relays; on this jig relay i feeds 3.3V into analog input i.
//   The test drives one channel at a time and reads ALL inputs: the driven input
//   must land inside a tolerance window around 3.3V, and every other input must
//   stay near 0V. A non-driven input that rises reveals a short between inputs.
//   An all-relays-off baseline read catches stuck-high inputs. See AIN_ON_*/AIN_OFF_*.
//
// Board / wiring: KMPSmarti8ESP32Class. RAK3172 on the library default pins/UART
// (RX=25, TX=27, RST=26 -> J14_8/J14_6/J14_9). 8AIN ADC (2x ADS1015) on I2C
// SDA=14, SCL=13. Dead subsystems are re-initialised automatically.

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

#include "KMPCommon.h"
#include "KMPSmarti8ESP32.h"

#include "RAK3172.h"

// SMARTI8 board variant (non-Ethernet).
#define BOARD_TYPE SMARTI8_ESP32

// RAK3172 LoRa(true) -> print AT traffic on Serial for debugging.
RAK3172 LoRa(false);

KMPSmarti8ESP32Class board;

// ---------------------------------------------------------------------------
// RAK3172 / LoRa P2P
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// 8AIN v3.0 analog inputs (2x ADS1015)
// ---------------------------------------------------------------------------

Adafruit_ADS1015 ADS1;  // channels 1-4 (AIN 0..3)
Adafruit_ADS1015 ADS2;  // channels 5-8 (AIN 4..7)

#define AIN_SDA 14
#define AIN_SCL 13

#define AIN_COUNT       8
#define RELAY_SETTLE_MS 60     // let the relay + input settle before reading
// Analog pass/fail window. Relay ON injects 3.3V; the reading must fall inside a
// tolerance window around the nominal (catches both a low channel - bad contact /
// high impedance - and a high channel - short to a higher rail). Relay OFF must sit
// at/below a small ceiling. Measured on a good board: ON ~3227 mV, OFF ~10 mV.
#define AIN_ON_NOMINAL_MV 3300           // expected reading with the relay ON (3.3V)
#define AIN_ON_TOL_MV     700            // +/- tolerance window around the nominal
#define AIN_ON_MIN_MV     (AIN_ON_NOMINAL_MV - AIN_ON_TOL_MV)  // 2600 mV
#define AIN_ON_MAX_MV     (AIN_ON_NOMINAL_MV + AIN_ON_TOL_MV)  // 4000 mV
#define AIN_OFF_MAX_MV    500            // relay OFF must read at/below this
#define ANALOG_TEST_PERIOD 3000
#define HEARTBEAT_PERIOD   2000  // ms between debug status lines

unsigned long lastHeartbeat = 0;

// ADC calibration - set by initADC() from the detected 8AIN hardware version.
float scaleFactor = 1.0;       // (R1 + R2) / R2 input divider
float referenceVoltage = 2048; // ADC reference in mV
int   adcResolution = 2048;    // ADC counts at reference

bool adcOk = false;              // both ADS1015 responded
bool analogTested = false;       // the 8AIN sweep has run at least once
bool analogTestPass = false;     // all 8 channels passed on the last sweep
unsigned long lastAnalogTest = 0;

// Function prototypes
bool loraInit(void);
bool sendMsg(uint8_t* msg, size_t size);
bool initADC(void);
void i2cScan(void);
int16_t readADCChannel(uint8_t channel);
uint16_t adctomv(int16_t adc);
bool testAnalogInputs(void);
void showStatus(void);
void printHeartbeat(void);

void setup()
{
  Serial.begin(115200);

  board.begin(BOARD_TYPE);
  board.setStatusLed(orange, 0);  // RAK3172 - initialising
  board.setStatusLed(orange, 1);  // 8AIN    - initialising
  board.setAllRelaysOff();
  delay(1000);

  Serial.println();
  Serial.println("========================================");
  Serial.println(" P720 - RAK3172 + 8AIN v3.0 test");
  Serial.printf ("  build: %s %s\n", __DATE__, __TIME__);
  Serial.println("  LED0 = RAK3172/RF,  LED1 = 8AIN");
  Serial.println("========================================");
  Serial.flush();

  Serial.println("[setup] initialising RAK3172...");
  radioOk = loraInit();
  Serial.printf("[setup] RAK3172 init: %s\n", radioOk ? "OK" : "FAIL");

  Serial.println("[setup] initialising 8AIN ADC...");
  adcOk = initADC();
  Serial.printf("[setup] 8AIN ADC init: %s\n", adcOk ? "OK" : "FAIL");
  Serial.flush();

  // Run the analog sweep once at boot so LED 1 shows a result quickly.
  if (adcOk)
  {
    analogTestPass = testAnalogInputs();
    analogTested = true;
    lastAnalogTest = millis();
  }

  showStatus();
}

void loop()
{
  // (Re)initialise dead subsystems so a freshly fitted board is picked up.
  if (!radioOk)
  {
    radioOk = loraInit();
    if (!radioOk) delay(RETRY_PERIOD);  // don't hammer a dead module
  }
  if (!adcOk)
  {
    adcOk = initADC();
  }

  // --- RAK3172 RF test: listen for a PING and echo it back as PONG. ---
  if (radioOk)
  {
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

      board.setStatusLed(blue, 0);  // activity blink (held during the TX below)

      uint8_t pong[] = { MSG_MAGIC, CMD_PONG, seq };
      if (sendMsg(pong, sizeof(pong)))
      {
        lastCommTime = millis();  // full radio round-trip succeeded
      }
    }
  }

  // --- 8AIN test: run periodically. ---
  if (adcOk && (millis() - lastAnalogTest) >= ANALOG_TEST_PERIOD)
  {
    analogTestPass = testAnalogInputs();
    analogTested = true;
    lastAnalogTest = millis();
  }

  // --- Debug heartbeat so problems are visible even when nothing happens. ---
  if ((millis() - lastHeartbeat) >= HEARTBEAT_PERIOD)
  {
    printHeartbeat();
    lastHeartbeat = millis();
  }

  showStatus();
}

// One-line status summary for the terminal.
void printHeartbeat(void)
{
  long linkAge = (lastCommTime == 0) ? -1 : (long)(millis() - lastCommTime);
  Serial.printf("[hb] t=%lus  RAK3172=%s  RFlink=%s(age=%ldms)  ADC=%s  8AIN=%s\n",
                millis() / 1000,
                radioOk ? "OK" : "FAIL",
                (lastCommTime != 0 && linkAge < LINK_TIMEOUT) ? "UP" : "DOWN",
                linkAge,
                adcOk ? "OK" : "FAIL",
                !analogTested ? "n/a" : (analogTestPass ? "PASS" : "FAIL"));
  Serial.flush();
}

// Drive both SMARTI8 status LEDs from the current test state.
void showStatus(void)
{
  // LED 0 - RAK3172 / radio link
  if (!radioOk)
  {
    board.setStatusLed(red, 0);                                    // module fault
  }
  else if (lastCommTime != 0 && (millis() - lastCommTime) < LINK_TIMEOUT)
  {
    board.setStatusLed(green, 0);                                  // link OK
  }
  else
  {
    board.setStatusLed(orange, 0);                                 // alive, no RF link
  }

  // LED 1 - 8AIN analog inputs
  if (!adcOk)
  {
    board.setStatusLed(red, 1);                                    // ADC not found
  }
  else if (!analogTested)
  {
    board.setStatusLed(orange, 1);                                 // not run yet
  }
  else
  {
    board.setStatusLed(analogTestPass ? green : red, 1);           // pass / fail
  }
}

// ---------------------------------------------------------------------------
// 8AIN v3.0 helpers
// ---------------------------------------------------------------------------

// Detect the 8AIN add-on (v2 @ 0x48/0x49 or v3.0 @ 0x4A/0x4B) and set calibration.
bool initADC(void)
{
  Wire.begin(AIN_SDA, AIN_SCL);

  // Dump every device on the bus first - invaluable when the ADC "isn't found".
  i2cScan();

  bool a48 = ADS1.begin(0x48);
  bool a49 = ADS2.begin(0x49);
  Serial.printf("[adc] v2 probe: 0x48=%s 0x49=%s\n", a48 ? "ack" : "-", a49 ? "ack" : "-");
  if (a48 && a49)
  {
    scaleFactor = 32.0 / 10.0;    // (R1 + R2) / R2
    referenceVoltage = 4096.0;    // mV
    adcResolution = 2048;         // 11-bit
    ADS1.setGain(GAIN_ONE);
    ADS2.setGain(GAIN_ONE);
    Serial.println("[adc] 8AIN v2 detected -> gain=ONE ref=4096mV");
    Serial.flush();
    return true;
  }

  bool a4A = ADS1.begin(0x4A);
  bool a4B = ADS2.begin(0x4B);
  Serial.printf("[adc] v3 probe: 0x4A=%s 0x4B=%s\n", a4A ? "ack" : "-", a4B ? "ack" : "-");
  if (a4A && a4B)
  {
    scaleFactor = 50.2 / 10.0;    // (R1 + R2) / R2
    referenceVoltage = 2048.0;    // mV
    adcResolution = 2048;         // 11-bit
    ADS1.setGain(GAIN_TWO);
    ADS2.setGain(GAIN_TWO);
    Serial.println("[adc] 8AIN v3.0 detected -> gain=TWO ref=2048mV");
    Serial.flush();
    return true;
  }

  Serial.println("[adc] 8AIN ADC NOT found (need 0x48+0x49 or 0x4A+0x4B).");
  Serial.flush();
  return false;
}

// Scan the I2C bus and print every responding address (debug aid).
void i2cScan(void)
{
  Serial.print("[i2c] scanning bus...");
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; addr++)
  {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0)
    {
      Serial.printf(" 0x%02X", addr);
      found++;
    }
  }
  Serial.printf("  (%u device%s)\n", found, found == 1 ? "" : "s");
  Serial.flush();
}

// Read one raw single-ended ADC count. Channels 0..3 on ADS1, 4..7 on ADS2.
int16_t readADCChannel(uint8_t channel)
{
  if (channel < 4)
  {
    return ADS1.readADC_SingleEnded(channel);
  }
  return ADS2.readADC_SingleEnded(channel - 4);
}

// Convert a raw ADC count to the input voltage in millivolts.
uint16_t adctomv(int16_t adc)
{
  if (adc < 0) adc = 0;  // clamp noise around 0 V so it never wraps
  return (uint16_t)(((float)adc * referenceVoltage / adcResolution) * scaleFactor);
}

// Sweep all 8 channels: relay i ON -> expect 3.3V on AIN i, OFF -> expect 0V.
bool testAnalogInputs(void)
{
  bool allPass = true;
  Serial.printf("--- 8AIN test: ON window %u..%u mV, OFF/cross-talk <=%u mV ---\n",
                AIN_ON_MIN_MV, AIN_ON_MAX_MV, AIN_OFF_MAX_MV);
  Serial.flush();

  board.setAllRelaysOff();
  delay(RELAY_SETTLE_MS);

  // Baseline: with every relay OFF all inputs must read low.
  // A high reading here means a stuck relay or an input shorted to a live rail.
  for (uint8_t j = 0; j < AIN_COUNT; j++)
  {
    uint16_t mv = adctomv(readADCChannel(j));
    if (mv > AIN_OFF_MAX_MV)
    {
      allPass = false;
      Serial.printf("  baseline AIN%u = %u mV (> %u) -> STUCK HIGH / SHORT to rail\n",
                    j + 1, mv, AIN_OFF_MAX_MV);
      Serial.flush();
    }
  }

  // Drive one channel at a time and read ALL channels. Only the driven channel may
  // be high; if any other input rises, those two inputs are shorted together (or the
  // relay is stuck) - this is how a short *between* analog inputs is detected.
  for (uint8_t i = 0; i < AIN_COUNT; i++)
  {
    board.setRelayState(i, true);          // inject 3.3V on AIN i only
    delay(RELAY_SETTLE_MS);

    uint16_t drivenMv = 0;
    bool windowOk = false;
    int8_t shortCh = -1;                    // first other channel found high
    uint16_t shortMv = 0;

    for (uint8_t j = 0; j < AIN_COUNT; j++)
    {
      uint16_t mv = adctomv(readADCChannel(j));
      if (j == i)
      {
        drivenMv = mv;
        windowOk = (mv >= AIN_ON_MIN_MV) && (mv <= AIN_ON_MAX_MV);
      }
      else if (mv > AIN_OFF_MAX_MV && shortCh < 0)
      {
        shortCh = (int8_t)j;
        shortMv = mv;
      }
    }

    board.setRelayState(i, false);

    bool pass = windowOk && (shortCh < 0);
    if (!pass) allPass = false;

    if (shortCh >= 0)
    {
      Serial.printf("  AIN%u  ON=%4u mV [%s]  -> FAIL: AIN%u also %u mV (SHORT / cross-talk)\n",
                    i + 1, drivenMv, windowOk ? "hi" : "window", shortCh + 1, shortMv);
    }
    else
    {
      Serial.printf("  AIN%u  ON=%4u mV  -> %s\n",
                    i + 1, drivenMv, pass ? "PASS" : "FAIL (out of window)");
    }
    Serial.flush();
  }

  board.setAllRelaysOff();
  Serial.printf("8AIN result: %s\n", allPass ? "ALL PASS" : "FAIL");
  Serial.flush();
  return allPass;
}

// ---------------------------------------------------------------------------
// RAK3172 helpers
// ---------------------------------------------------------------------------

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
