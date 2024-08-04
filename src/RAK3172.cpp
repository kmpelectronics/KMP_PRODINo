#include "RAK3172.h"

HardwareSerial* _rak_at_serial;
HardwareSerial* _rak_debug_serial;

uint8_t _rak_rst;

uint8_t RAK3172_DEBUG;

RAK3172::RAK3172(bool debug, HardwareSerial* serial, HardwareSerial* debugSerial) {
    _rak_at_serial = serial;
    _rak_debug_serial = debugSerial;
    RAK3172_DEBUG = debug;
}

RAK3172::~RAK3172() {
    // Code for cleaning up the RAK3172 module
}

bool RAK3172::begin(uint8_t rx, uint8_t tx, uint8_t rst) {
    // Code for initializing the RAK3172 module using the provided serial object
    _rak_at_serial->begin(RAK3172_DEFAULT_BAUD, SERIAL_8N1, tx, rx);
    _rak_rst = rst;
    pinMode(_rak_rst, OUTPUT);
    digitalWrite(_rak_rst, HIGH);

    //sendAT("", RAK3172_TIMEOUT_300, RAK3172_DEBUG);
    flushRX(300);

    if(sendAT("AT","OK", RAK3172_TIMEOUT_100, RAK3172_DEBUG))
    {
        _rak_debug_serial->println("LoRa OK. ");
        _rak_debug_serial->flush();
    }
    else
    {
        _rak_debug_serial->println("LoRa modem not responding. ");
        _rak_debug_serial->flush();
        return false;
    }

    return true;
}

void RAK3172::reset()
{
    digitalWrite(_rak_rst, LOW);
    delay(100);
    digitalWrite(_rak_rst, HIGH);
}

String RAK3172::getVersion()
{
    flushRX();
    String response = sendAT("AT+VER=?", RAK3172_TIMEOUT_200, RAK3172_DEBUG);
    int index = response.indexOf("AT+VER=");
    if (index >= 0) {
        return response.substring(index + 7);
    }
    return "";
}

bool RAK3172::getLoRaMode()
{
    flushRX();
    String response = sendAT("AT+NWM=?", RAK3172_TIMEOUT_200, RAK3172_DEBUG);
    if (response.indexOf("AT+NWM=1") >= 0) return true;
    return false;
}

bool RAK3172::setLoRaMode(bool mode)
{
    flushRX();
    if (mode)
    {
        return sendAT("AT+NWM=1", "LoRaWAN", RAK3172_TIMEOUT_300, RAK3172_DEBUG);
    }
    else
    {
        return sendAT("AT+NWM=0", "LoRa P2P", RAK3172_TIMEOUT_300, RAK3172_DEBUG);
    }
}

bool RAK3172::setLoRaP2PParameters(uint32_t frequency, uint8_t sf, uint8_t bw, uint8_t cr, uint8_t prlen, uint8_t pwr)
{
    flushRX();
    String command = "AT+P2P=" + String(frequency) + ":" + String(sf) + ":" + String(bw) + ":" + String(cr) + ":" + String(prlen) + ":" + String(pwr);
    return sendAT(command, "OK", RAK3172_TIMEOUT_300, RAK3172_DEBUG);
}

bool RAK3172::getLoRaP2PParameters(uint32_t *frequency, uint8_t *sf, uint8_t *bw, uint8_t *cr, uint8_t *prlen, uint8_t *pwr)
{
    flushRX();
    String response = sendAT("AT+P2P=?", RAK3172_TIMEOUT_300, RAK3172_DEBUG);
    int index = response.indexOf("AT+P2P=");
    if (index >= 0)
    {
        response = response.substring(index + 7);
        int index1 = response.indexOf(":");
        int index2 = response.indexOf(":", index1 + 1);
        int index3 = response.indexOf(":", index2 + 1);
        int index4 = response.indexOf(":", index3 + 1);
        int index5 = response.indexOf(":", index4 + 1);
        *frequency = response.substring(0, index1).toInt();
        *sf = response.substring(index1 + 1, index2).toInt();
        *bw = response.substring(index2 + 1, index3).toInt();
        *cr = response.substring(index3 + 1, index4).toInt();
        *prlen = response.substring(index4 + 1, index5).toInt();
        *pwr = response.substring(index5 + 1).toInt();
        return true;
    }
    return false;
}

bool RAK3172::setReceiveMode(uint16_t timeout)
{
    flushRX();
    sendAT("AT+PRECV=" + String(timeout), 0, RAK3172_DEBUG);
    if(!getline("OK", RAK3172_TIMEOUT_300, RAK3172_DEBUG))return false;
    delay(5);
    return true;
}

bool RAK3172::receiveMassage(String *massage, uint16_t timeout)
{
    String msg = String("");
    msg = getline(timeout, RAK3172_DEBUG);
    int iTimeout = msg.indexOf("TIMEOUT");
    int iStart = msg.indexOf("RXP2P");
    int iEnd = msg.indexOf("\r");

    if(iTimeout >= 0)
    {
        *massage = String("");
        return true;
    }

    if (iStart >= 0)
    {
        *massage = msg.substring(iStart + 13, iEnd);
        return true;
    }

    *massage = String("");
    return false;
}

bool RAK3172::receiveMassage(uint8_t *message, size_t maxSize, size_t *messageSize, uint16_t timeout)
{
    String msg = String("");
    if(receiveMassage(&msg, timeout))
    {
        size_t hexLength = msg.length();

        if(hexLength == 0)
        {
            // No message received
            messageSize = 0;
            return true;
        }

        if (hexLength % 2 != 0)
        {
            // Invalid hex string length
            return false;
        }

        size_t byteLength = hexLength / 2;
        if (byteLength > maxSize)
        {
            // Output buffer is not large enough
            return false;
        }

        for (size_t i = 0; i < byteLength; i++)
        {
            String byteString = msg.substring(i * 2, i * 2 + 2);
            message[i] = strtol(byteString.c_str(), NULL, 16);
        }

        *messageSize = byteLength;
        return true;
    }

    return false;
}

bool RAK3172::transmitMessage(String& message) 
{
    flushRX();
    sendAT("AT+PSEND=" + message, 0, RAK3172_DEBUG);
    if(!getline("OK", RAK3172_TIMEOUT_5000, RAK3172_DEBUG))return false;
    if(!getline("TXP2P DONE", RAK3172_TIMEOUT_5000, RAK3172_DEBUG))return false;
    return true;
}

bool RAK3172::transmitMessage(uint8_t *message, size_t messageSize) {
    String hexMessage = String("");
    for (size_t i = 0; i < messageSize; i++)
    {
        if (message[i] < 0x10) hexMessage += "0";
        hexMessage += String(message[i],HEX);
    }
    return transmitMessage(hexMessage);
}

bool RAK3172::transmitMessage(uint8_t *txMessage, size_t txMessageSize, uint8_t *rxMessage, size_t maxRxSize, size_t *rxMessageSize, uint16_t timeout)
{
    if(!setReceiveMode(RAK3172_TX_MODE))return false;
    if(!transmitMessage(txMessage, txMessageSize))return false;
    if(!setReceiveMode(timeout))return false;
    if(!receiveMassage(rxMessage, maxRxSize, rxMessageSize, timeout))return false;

    return true;
}


//private

void RAK3172::flushRX()
{
    while (_rak_at_serial->available())
    {
        _rak_at_serial->read();
    }
}

void RAK3172::flushRX(int timeout)
{
    long int time = millis();
    while ((time + timeout) > millis())
    {
        flushRX();
    }
}

String RAK3172::sendAT(String command, const int timeout, bool debug)
{
    String response = String("");

    if (0 < command.length())
    {
        if (debug)
        {
            _rak_debug_serial->println(command);
            _rak_debug_serial->flush();
        }
        _rak_at_serial->println(command);
        _rak_at_serial->flush();
    }

    long int time = millis();
    while ((time + timeout) > millis())
    {
        while (_rak_at_serial->available())
        {
            char c = _rak_at_serial->read();
            response += c;
        }
    }
    if (debug)
    {   
        _rak_debug_serial->print(response);
        _rak_debug_serial->flush();
    }
    return response;
}

bool RAK3172::sendAT(String command, String response, const int timeout, bool debug)
{
    String msg = sendAT(command, timeout, debug);
    if (msg.indexOf(response) >= 0)return true;
    return false;
}

String RAK3172::sendAT(String command, bool debug)
{
    return sendAT(command, RAK3172_DEFAULT_TIMEOUT, debug);
}

String RAK3172::getline(int timeout, bool debug)
{
    String response = String("");
    long int time = millis();
    while ((time + timeout) > millis())
    {
        while (_rak_at_serial->available())
        {
            char c = _rak_at_serial->read();
            response += c;
            if (c == '\n')
            {
                if (debug)
                {   
                    _rak_debug_serial->print(response);
                    _rak_debug_serial->flush();
                }
                return response;
            }
        }
    }
    return response;
}

bool RAK3172::getline(String response, const int timeout, bool debug)
{
    String msg = getline(timeout, debug);
    if (msg.indexOf(response) >= 0)return true;
    return false;
}


    // String msg = String("");
    // msg = sendAT("AT", 100, RAK3172_DEBUG);
    // if (msg.indexOf("OK") >= 0)_rak_debug_serial->println("LoRa OK. ");
    // else {_rak_debug_serial->println("LoRa modem not responding. "); return false;}

    // msg = sendAT("AT+VER=?", 100, RAK3172_DEBUG);
    // if (msg.indexOf("RUI_4.1.1") >= 0)_rak_debug_serial->println("FW v4.1.1");
    // else {_rak_debug_serial->println("LoRa modem FW is not v4.1.1. "); return false;}

    // msg = sendAT("AT+NWM=0", 1000, RAK3172_DEBUG);
    // if (msg.indexOf("Mode: LoRa P2P") >= 0)_rak_debug_serial->println("LoRa P2P mode set OK.");
    // else {_rak_debug_serial->println("LoRa P2P mode set Error."); return false;}

    // msg = sendAT("AT+P2P=868000000:7:125:0:10:14", 100, RAK3172_DEBUG);
    // if (msg.indexOf("OK") >= 0)_rak_debug_serial->println("LoRa P2P parameters set OK.");
    // else {_rak_debug_serial->println("LoRa P2P parameters set Error."); return false;}