// KMPSmarti8ESP32.cpp
// Company: KMP Electronics Ltd, Bulgaria
// Web: https://kmpelectronics.eu/
// Supported boards: 
//		SMARTI8 ESP32 
//		SMARTI8 ESP32 Ethernet
// Description:
//		Header for KMP SMARTI8 ESP32 boards.
// Version: 0.1.0
// Date: 24.07.2024
// Authors: Dimitar Antonov <d.antonov@kmpelectronics.eu>

#include "KMPSmarti8ESP32.h"

struct BoardConfig_t {
	BoardType Board;
	bool Ethernet;
};

const BoardConfig_t BoardConfig[BOARDS_COUNT] = {
	{SMARTI8_ESP32, false},
	{SMARTI8_ESP32_Ethernet, true},
};

// Relay pins
#define REL1PIN 16
#define REL2PIN 15
#define REL3PIN 14
#define REL4PIN 13
#define REL5PIN 12
#define REL6PIN 11
#define REL7PIN 10
#define REL8PIN 9
/**
 * @brief Relay pins.
 */
const uint8_t RELAY_PINS[RELAY_COUNT] = { REL1PIN, REL2PIN, REL3PIN, REL4PIN, REL5PIN, REL6PIN, REL7PIN, REL8PIN };

// Input pins
#define IN1PIN  1
#define IN2PIN  2
#define IN3PIN  3
#define IN4PIN  4
#define IN5PIN  5
#define IN6PIN  6
#define IN7PIN  7
#define IN8PIN  8
/**
 * @brief Input pins.
 */
const int OPTOIN_PINS[OPTOIN_COUNT] = { IN1PIN, IN2PIN, IN3PIN, IN4PIN, IN5PIN, IN6PIN, IN7PIN, IN8PIN };

// Expander CS pin.
#define MCP23S17CSPin 32  // IO32

MCP Exp1(0x00, MCP23S17CSPin);


// Status RGB LED.
const uint16_t StatusLedPixelCount = 3;
const uint8_t StatusLedPin = 0;
NeoPixelBus<NeoGrbFeature, NeoEsp32BitBang800KbpsMethod> _statusLed(StatusLedPixelCount, StatusLedPin);

// W5500 pins.
#define W5500ResetPin 12  // I12
#define W5500CSPin    33 // IO33

// RS485 pins. Serial1.
#define RS485Pin      2   // IO12
#define RS485RxPin    4   // IO4
#define RS485TxPin    16  // IO16
#define RS485Transmit HIGH
#define RS485Receive  LOW
HardwareSerial RS485Serial(1);

// GSM module pins for Serial2
#define GSMCTSPin   J14_5
#define GSMRTSPin   J14_6
#define GSMRxPin    J14_7
#define GSMTxPin    J14_8
#define GSMDTRPin   J14_9
#define GSMResetPin J14_10

// LoRa module pins for Serial2
#define LoRaRxPin    J14_5
#define LoRaLowPin   J14_6
#define LoRaTxPin    J14_8
#define LoRaBootPin  J14_11
#define LoRaResetPin J14_12
HardwareSerial SerialModem(2);

//KMPSmarti8ESP32Class board;
BoardType _board;

uint32_t _TxFlushDelayuS;

unsigned long _blinkIntervalTimeout[StatusLedPixelCount];
uint8_t _ledState[StatusLedPixelCount];

void KMPSmarti8ESP32Class::begin(BoardType board, bool startEthernet, bool startModem)
{
	_board = board;

	bool isBoardInitialized = false;
	for (size_t i = 0; i < BOARDS_COUNT; i++)
	{
		BoardConfig_t boardConfig = BoardConfig[i];
		if (board == boardConfig.Board)
		{
			isBoardInitialized = true;

			if (boardConfig.Ethernet)
			{
				beginEthernet(startEthernet);
			}

			break;
		}
	}

	if (!isBoardInitialized)
	{
		Serial.println(F("The board is not initialized!"));
		while (1) {}
	}

	// Init expander pins.
	//pinMode(MCP23S17IntetuptPin, INPUT);

	// Set expander pins direction.
	//MCP23S17.init(MCP23S17CSPin);

	Exp1.begin();
	//Exp2.begin();

	for (uint8_t i = 0; i < RELAY_COUNT; i++)
	{
		if (17 > RELAY_PINS[i])Exp1.pinMode(RELAY_PINS[i], OUTPUT);
	}

	for (uint8_t i = 0; i < OPTOIN_COUNT; i++)
	{
		if (17 > OPTOIN_PINS[i])Exp1.pinMode(OPTOIN_PINS[i], INPUT);
	}

	for (uint8_t i = 0; i < OPTOIN_COUNT; i++)
	{
		if (17 > OPTOIN_PINS[i])Exp1.inputInvert(OPTOIN_PINS[i], 1);
	}

	// Status led.

	for (uint8_t i = 0; i < StatusLedPixelCount; i++)
	{
		_blinkIntervalTimeout[i] = 0;
		_ledState[i] = 0;
	}

	_statusLed.Begin();

	// RS485 pin init.
	pinMode(RS485Pin, OUTPUT);
	digitalWrite(RS485Pin, RS485Receive);
}

void KMPSmarti8ESP32Class::beginEthernet(bool startEthernet)
{
	// W5500 pin init.
	pinMode(W5500ResetPin, OUTPUT);

	if (startEthernet)
	{
		restartEthernet();
		Ethernet.init(W5500CSPin);
	}
	else
	{
		digitalWrite(W5500ResetPin, LOW);
	}
}

void KMPSmarti8ESP32Class::restartEthernet()
{
	// RSTn Pull-up Reset (Active low) RESET should be held low at least 500 us for W5500 reset.
	digitalWrite(W5500ResetPin, LOW);
	delay(600);
	digitalWrite(W5500ResetPin, HIGH);
}

RgbColor KMPSmarti8ESP32Class::getStatusLed(uint8_t num)
{
	if (StatusLedPixelCount <= num)return 0;
	return _statusLed.GetPixelColor(num);
}

void KMPSmarti8ESP32Class::setStatusLed(RgbColor color, uint8_t num)
{
	if (StatusLedPixelCount <= num)return;
	_statusLed.SetPixelColor(num, color);
	_statusLed.Show();
}

//void KMPProDinoESP32Class::OnStatusLed()
//{
//	setStatusLed(true);
//}

void KMPSmarti8ESP32Class::offStatusLed()
{
	for (uint8_t i = 0; i < StatusLedPixelCount; i++)
	{
		_statusLed.SetPixelColor(i, 0);
	}

	_statusLed.Show();
}

void KMPSmarti8ESP32Class::processStatusLed(RgbColor color, int blinkInterval, uint8_t num)
{
	if (millis() > _blinkIntervalTimeout[num])
	{
		_ledState[num] = !_ledState[num];

		if (_ledState)
		{
			// Here you can check statuses: is WiFi connected, is there Ethernet connection and other...
			setStatusLed(color, num);
		}
		else
		{
			_statusLed.SetPixelColor(num, 0);
		}

		// Set next time to read data.
		_blinkIntervalTimeout[num] = millis() + blinkInterval;
	}
}

//void KMPSmarti8ESP32Class::NotStatusLed()
//{
//	setStatusLed(!getStatusLed());
//}

/* ----------------------------------------------------------------------- */
/* Relays methods. */
/* ----------------------------------------------------------------------- */

void KMPSmarti8ESP32Class::setRelayState(uint8_t relayNumber, bool state)
{
	//Check if relayNumber is out of range - return.
	if (relayNumber > RELAY_COUNT - 1)
	{
		return;
	}


	if (17 > RELAY_PINS[relayNumber])Exp1.digitalWrite(RELAY_PINS[relayNumber], state);
	//else Exp2.digitalWrite(RELAY_PINS[relayNumber] - 16, state);

}

void KMPSmarti8ESP32Class::togleRelayState(uint8_t relayNumber)
{
	//Check if relayNumber is out of range - return.
	if (relayNumber > RELAY_COUNT - 1)
	{
		return;
	}


	if (17 > RELAY_PINS[relayNumber])Exp1.digitalTogle(RELAY_PINS[relayNumber]);
	//else Exp2.digitalTogle(RELAY_PINS[relayNumber] - 16);
}

void KMPSmarti8ESP32Class::togleRelayState(Relay relay)
{
	togleRelayState((uint8_t)relay);
}

void KMPSmarti8ESP32Class::setRelayState(Relay relay, bool state)
{
	setRelayState((uint8_t)relay, state);
}

void KMPSmarti8ESP32Class::setAllRelaysState(bool state)
{
	for (uint8_t i = 0; i < RELAY_COUNT; i++)
	{
		setRelayState(i, state);
	}
}

void KMPSmarti8ESP32Class::setAllRelaysOn()
{
	setAllRelaysState(true);
}

void KMPSmarti8ESP32Class::setAllRelaysOff()
{
	setAllRelaysState(false);
}

//uint8_t KMPSmarti8ESP32Class::getRelayState(void)
//{
//	uint8_t tState = (MCP23S08.GetPinState() & 0xf0) >> 4;
//	uint8_t tRet = 0;
//	if (tState & (1 << 0))tRet |= 1 << 3;
//	if (tState & (1 << 1))tRet |= 1 << 2;
//	if (tState & (1 << 2))tRet |= 1 << 1;
//	if (tState & (1 << 3))tRet |= 1 << 0;
//	return tRet;
//}
//
bool KMPSmarti8ESP32Class::getRelayState(uint8_t relayNumber)
{
	// Check if relayNumber is out of range - return false.
	if (relayNumber > RELAY_COUNT - 1)
	{
		return false;
	}

	if (17 > RELAY_PINS[relayNumber])return Exp1.digitalRead(RELAY_PINS[relayNumber]);
	//else return Exp2.digitalRead(RELAY_PINS[relayNumber] - 16);

	return false;
}

bool KMPSmarti8ESP32Class::getRelayState(Relay relay)
{
	return getRelayState((uint8_t)relay);
}

/* ----------------------------------------------------------------------- */
/* Opto input methods. */
/* ----------------------------------------------------------------------- */
//uint8_t KMPSmarti8ESP32Class::getOptoInState(void)
//{
//	uint8_t tState = ~MCP23S08.GetPinState();
//	uint8_t tRet = 0;
//	if (tState & (1 << 0))tRet |= 1 << 3;
//	if (tState & (1 << 1))tRet |= 1 << 2;
//	if (tState & (1 << 2))tRet |= 1 << 1;
//	if (tState & (1 << 3))tRet |= 1 << 0;
//	return tRet;
//}

bool KMPSmarti8ESP32Class::getOptoInState(uint8_t optoInNumber)
{
	// Check if optoInNumber is out of range - return false.
	if (optoInNumber > OPTOIN_COUNT - 1)
	{
		return false;
	}

	if (17 > OPTOIN_PINS[optoInNumber])return Exp1.digitalRead(OPTOIN_PINS[optoInNumber]);
	//else return Exp2.digitalRead(OPTOIN_PINS[optoInNumber] - 16);

	return false;
}

bool KMPSmarti8ESP32Class::getOptoInState(OptoIn optoIn)
{
	return getOptoInState((uint8_t)optoIn);
}

/* ----------------------------------------------------------------------- */
/* RS485 methods. */
/* ----------------------------------------------------------------------- */
void KMPSmarti8ESP32Class::rs485Begin(unsigned long baud)
{
	rs485Begin(baud, SERIAL_8N1);
}

void KMPSmarti8ESP32Class::rs485Begin(unsigned long baud, uint32_t config)
{
	RS485Serial.begin(baud, config, RS485RxPin, RS485TxPin);
	_TxFlushDelayuS = (uint32_t)((1000000 / baud) * 15);
}

void KMPSmarti8ESP32Class::rs485End()
{
	RS485Serial.end();
}

/**
* @brief Begin write data to RS485.
*
* @return void
*/
void KMPSmarti8ESP32Class::RS485BeginWrite()
{
	digitalWrite(RS485Pin, RS485Transmit);
	// Allowing pin should delay for 50 nS
	delayMicroseconds(70);
}

/**
* @brief End write data to RS485.
*
* @return void
*/
void KMPSmarti8ESP32Class::RS485EndWrite()
{
	RS485Serial.flush();
	delayMicroseconds(_TxFlushDelayuS);
	digitalWrite(RS485Pin, RS485Receive);
}

size_t KMPSmarti8ESP32Class::rs485Write(const uint8_t data)
{
	RS485BeginWrite();

	size_t result = RS485Serial.write(data);

	RS485EndWrite();

	return result;
}

size_t KMPSmarti8ESP32Class::rs485Write(const uint8_t* data, size_t dataLen)
{
	RS485BeginWrite();

	size_t result = RS485Serial.write(data, dataLen);

	RS485EndWrite();

	return result;
}

int KMPSmarti8ESP32Class::rs485Read()
{
	return rs485Read(10, 10);
}

int KMPSmarti8ESP32Class::rs485Read(unsigned long delayWait, uint8_t repeatTime)
{
	// If the buffer is empty, wait until the data arrives.
	while (!RS485Serial.available())
	{
		delay(delayWait);
		--repeatTime;

		if (repeatTime == 0)
		{
			return -1;
		}
	}

	return RS485Serial.read();
}

void KMPSmarti8ESP32Class::rs485print(const String &s)
{
	RS485Serial.print(s);
}

bool KMPSmarti8ESP32Class::rs485find(const char *target)
{
        return RS485Serial.find(target);
}
