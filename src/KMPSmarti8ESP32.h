// KMPSmarti8ESP32.h
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

#pragma once

#ifndef _KMPSMARTI8ESP32_H
#define _KMPSMARTI8ESP32_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include <NeoPixelBus.h>
#include "MCP23S17.h"
// When the library is fixed to work with ESP32 we will change this reference.
//#include <Ethernet.h>
#include "Ethernet/Ethernet.h"

// Relays count
#define RELAY_COUNT  8
// Inputs count
#define OPTOIN_COUNT 8

// Expander interrupt pin 
#define MCP23S17IntetuptPin 36  // IO36

/**
 * @brief A Grove connector pins
 */
#define	GROVE_D0	22 // IO22
#define	GROVE_D1	21 // IO21

 /**
  * @brief J14 connector
  */
//#define	J14_1	GND
//#define	J14_2	5V
//#define	J14_3	GND
//#define	J14_4	3V3
#define	J14_5	35 // I35
#define	J14_6	27 // IO27
#define	J14_7	34 // I34
#define	J14_8	25 // IO25
#define	J14_9	26 // IO26
#define	J14_10	14 // IO14
#define	J14_11	13 // IO13
#define	J14_12	15 // IO15

 /**
  * @brief J2 connector
  */
#define	J2_1	18 // SCK
#define	J2_2	19 // MISO 
#define	J2_3	23 // MOSI
#define	J2_4	17 // IO17
#define	J2_5	1  // IO5
//#define	J2_6	GND


/**
 * @brief Relays.
 */
enum Relay {
	Relay1 = 0,
	Relay2 = 1,
	Relay3 = 2,
	Relay4 = 3,
	Relay5 = 4,
	Relay6 = 5,
	Relay7 = 6,
	Relay8 = 7
};

/**
 * @brief Inputs.
 */
enum OptoIn {
	OptoIn1 = 0,
	OptoIn2 = 1,
	OptoIn3 = 2,
	OptoIn4 = 3,
	OptoIn5 = 4,
	OptoIn6 = 5,
	OptoIn7 = 6,
	OptoIn8 = 7
};

/**
 * @brief All available bards.
 */
enum BoardType {
	SMARTI8_ESP32,
	SMARTI8_ESP32_Ethernet
};

#define BOARDS_COUNT SMARTI8_ESP32_Ethernet + 1

const char TEXT_HTML[] = "text/html; charset=utf-8";
const char PRODINO_ESP32[] = "SMARTI8 ESP32";
const char URL_KMPELECTRONICS_EU_PRODINO_ESP32[] = "https://kmpelectronics.eu/products/prodino-esp32-v1/";

extern HardwareSerial RS485Serial;

extern HardwareSerial SerialModem;

class KMPSmarti8ESP32Class
{
 public:

	/**
	* @brief Initialize ProDino ESP32 board.
	* @param board Initialize specific bard. Mandatory.
	* @param startEthernet If board has a Ethernet we can stop it if it isn't necessary. If true - starts Ethernet W5500 or false - the Ethernet stays stopped.
	* @param startModem If board has GSM or LoRa module we can stop it if it isn't necessary. If true - starts module or false - the module stays stopped.
	*
	* @return void
	*/
	void begin(BoardType board = SMARTI8_ESP32, bool startEthernet = true, bool startModem = true);

	/**
	* @brief Restarts (Stop & Start) GSM module.
	*
	* @return void
	*/
	void restartGSM();

	/**
	* @brief Restarts (Stop & Start) Ethernet.
	*
	* @return void
	*/
	void restartEthernet();

	/**
	* @brief Get status LED current color.
	*
	* @return RgbColor RGB color.
	*/
	RgbColor getStatusLed( uint8_t num = 0);

	/**
	* @brief Set status LED new color.
	*
	* @param color A new RGB color.
	*
	* @return void
	*/
	void setStatusLed(RgbColor color, uint8_t num = 0);
	
	/**
	* @brief Set status LED to Off.
	*
	* @return void
	*/
	void offStatusLed();

	/**
	* @brief Process status led.
	*
	* @param color A new RGB color.
	* @param blinkInterval blink interval in milliseconds.
	*
	* @return void
	*/
	void processStatusLed(RgbColor color, int blinkInterval, uint8_t num = 0);

	/**
	* @brief Set a relay new state.
	*
	* @param relayNumber Number of relay from 0 to RELAY_COUNT - 1. 0 - Relay1, 1 - Relay2 ...
	* @param state New state of relay, true - On, false = Off.
	*
	* @return void
	*/
	void setRelayState(uint8_t relayNumber, bool state);
	/**
	* @brief Set relay new state.
	*
	* @param relay Relays - Relay1, Relay2 ...
	* @param state New state of relay, true - On, false = Off.
	*
	* @return void
	*/
	void setRelayState(Relay relay, bool state);
	/**
	* @brief Set all relays new state.
	*
	* @param state New state of relay, true - On, false = Off.
	*
	* @return void
	*/
	void togleRelayState(Relay relay);
	/**
	* @brief Togle relay state.
	*
	* @return void
	*/
	void togleRelayState(uint8_t relayNumber);
	/**
	* @brief Togle relay state.
	*
	* @return void
	*/
	void setAllRelaysState(bool state);
	/**
	* @brief Set all relays in ON state.
	*
	* @return void
	*/
	void setAllRelaysOn();
	/**
	* @brief Set all relays in ON state.
	*
	* @return void
	*/
	void setAllRelaysOff();
	/**
	* @brief Get relay state.
	*
	* @param relayNumber Relay number from 0 to RELAY_COUNT - 1
	*
	* @return bool true relay is On, false is Off. If number is out of range - return false.
	*/
	bool getRelayState(uint8_t relayNumber);
	/**
	* @brief Get relay state.
	*
	* @param relay Relay1, Relay2 ...
	*
	* @return bool true relay is On, false is Off. If number is out of range - return false.
	*/
	bool getRelayState(Relay relay);

	uint8_t getRelayState(void);

	/**
	* @brief Get opto in state.
	*
	* @param optoInNumber OptoIn number from 0 to OPTOIN_COUNT - 1
	*
	* @return bool true - opto in is On, false is Off. If number is out of range - return false.
	*/
	bool getOptoInState(uint8_t optoInNumber);
	/**
	* @brief Get opto in state.
	*
	* @param relay OptoIn1, OptoIn2 ...
	*
	* @return bool true - opto in is On, false is Off. If number is out of range - return false.
	*/
	bool getOptoInState(OptoIn optoIn);

	uint8_t getOptoInState(void);

	/**
	* @brief Connect to RS485. With default configuration SERIAL_8N1.
	*
	* @param baud Speed.
	*     Values: 75, 110, 300, 1200, 2400, 4800, 9600, 19200, 38400, 57600 and 115200 bit/s.
	*
	* @return void
	*/
	void rs485Begin(unsigned long baud);
	/**
	* @brief Start connect to RS485.
	*
	* @param baud Speed.
	*             Values: 75, 110, 300, 1200, 2400, 4800, 9600, 19200, 38400, 57600 and 115200 bit/s.
	* @param config Configuration - data bits, parity, stop bits.
	*               Values: SERIAL_5N1, SERIAL_6N1, SERIAL_7N1, SERIAL_8N1, SERIAL_5N2, SERIAL_6N2, SERIAL_7N2, SERIAL_8N2, SERIAL_5E1, SERIAL_6E1, SERIAL_7E1, SERIAL_8E1, SERIAL_5E2,
	SERIAL_6E2, SERIAL_7E2, SERIAL_8E2, SERIAL_5O1, SERIAL_6O1, SERIAL_7O1, SERIAL_8O1, SERIAL_5O2, SERIAL_6O2, SERIAL_7O2, SERIAL_8O2
	*
	* @return void
	*/
	void rs485Begin(unsigned long baud, uint32_t config);
	/**
	* @brief Close connection to RS485.
	*
	* @return void
	*/
	void rs485End();
	/**
	* @brief Transmit one byte data to RS485.
	*
	* @param data Transmit data.
	*
	* @return size_t Count of transmitted - one byte.
	*/
	size_t rs485Write(const uint8_t data);
	/**
	* @brief Transmit one char data to RS485.
	*
	* @param data Transmit data.
	*
	* @return size_t Count of transmitted - one char.
	*/
	size_t rs485Write(const char data) { return rs485Write((uint8_t)data); }
	/**
	* @brief Transmit the text to RS485.
	*
	* @param data Text data to transmit.
	* @param dataLen Array length.
	*
	* @return size_t Count of transmitted chars.
	*/
	size_t rs485Write(const char* data, size_t dataLen) { return rs485Write((const uint8_t*)data, dataLen); }
	/**
	* @brief Transmit the text to RS485.
	*
	* @param data Text data to transmit.
	*
	* @return size_t Count of transmitted chars.
	*/
	size_t rs485Write(const String data) { return rs485Write(data.c_str(), data.length()); }
	/**
	* @brief Send array of bytes to RS485.
	*
	* @param data Array in bytes to be send.
	* @param dataLen Array length.
	*
	* @return size_t Count of transmitted bytes.
	*/
	size_t rs485Write(const uint8_t* data, size_t dataLen);
	/**
	* @brief Read received data from RS485.
	*
	*
	* @return int Received byte.<para></para>
	*   If result = -1 - buffer is empty, no data
	*   if result > -1 - valid byte to read.
	*/
	int rs485Read();
	/**
	* @brief Read received data from RS485. Reading data with delay and repeating the operation while all data to arrive.
	*
	* @param delayWait Wait delay if not available to read byte in milliseconds. Default 10.
	* @param repeatTime Repeat time if not read bytes. Default 10. All time = delayWait * repeatTime.
	*
	* @return int Received byte.
	*   If result = -1 - buffer is empty, no data<para></para>
	*   if result > -1 - valid byte to read.
	*/
	int rs485Read(unsigned long delayWait, uint8_t repeatTime);
	/**
	* @brief End write data to RS485.
	*
	* @return void
	*/
	void RS485EndWrite();
	/**
	* @brief Begin write data to RS485.
	*
	* @return void
	*/
	void RS485BeginWrite();

	private:
		void beginEthernet(bool startEthernet);
};

//extern KMPSmarti8ESP32Class board;

#endif