#ifndef RAK3172_H
#define RAK3172_H

#include <HardwareSerial.h>
#include <Arduino.h>

#define RAK3172_DEBUG true
#define RAK3172_DEFAULT_DEBUG_SERIAL &Serial

#define RAK3172_DEFAULT_RX 25
#define RAK3172_DEFAULT_TX 27
#define RAK3172_DEFAULT_RST 26
#define RAK3172_DEFAULT_BAUD 115200
#define RAK3172_DEFAULT_SERIAL &Serial2

#define RAK3172_DEFAULT_TIMEOUT 1000
#define RAK3172_TIMEOUT_100 100
#define RAK3172_TIMEOUT_200 200
#define RAK3172_TIMEOUT_300 300
#define RAK3172_TIMEOUT_500 500
#define RAK3172_TIMEOUT_5000 5000

#define RAK3172_CONTINUES_RECEIVE_MODE 65534
#define RAK3172_TX_MODE 0

typedef enum {
    RAK3172_MODE_LORAP2P = 0,
    RAK3172_MODE_LORAWAN = 1
} RAK3172_NETWORK_MODE;



class RAK3172 {
public:
    RAK3172(HardwareSerial *serial=RAK3172_DEFAULT_SERIAL, HardwareSerial *debugSerial = RAK3172_DEFAULT_DEBUG_SERIAL);
    ~RAK3172();

    bool begin(uint8_t rx = RAK3172_DEFAULT_RX, uint8_t tx = RAK3172_DEFAULT_TX, uint8_t rst = RAK3172_DEFAULT_RST);

    String getVersion();

    bool getLoRaMode();

    bool setLoRaMode(bool mode);

    bool getLoRaP2PParameters(uint32_t *frequency, uint8_t *sf, uint8_t *bw, uint8_t *cr, uint8_t *prlen, uint8_t *pwr);

    bool setReceiveMode(uint16_t timeout = RAK3172_CONTINUES_RECEIVE_MODE);

    bool setLoRaP2PParameters(uint32_t frequency, uint8_t sf, uint8_t bw, uint8_t cr, uint8_t prlen, uint8_t pwr);

    bool receiveMassage(String *massage, uint16_t timeout = RAK3172_TIMEOUT_300);

    bool receiveMassage(uint8_t *message, size_t maxSize, size_t *messageSize, uint16_t timeout);

    bool transmitMessage(String &message);

    bool transmitMessage(uint8_t *message, size_t messageSize);

    bool transmitMessage(uint8_t *txMessage, size_t txMessageSize, uint8_t *rxMessage, size_t maxRxSize, size_t *rxMessageSize, uint16_t timeout);

    void reset();

private:
    String sendAT(String command="", const int timeout = RAK3172_DEFAULT_TIMEOUT, bool debug = false);

    bool sendAT(String command, String response, const int timeout, bool debug);

    String sendAT(String command = "", bool debug = false);

    String getline(int timeout=RAK3172_DEFAULT_TIMEOUT, bool debug=false);

    bool getline(String response, const int timeout, bool debug);

    void flushRX();

    void flushRX(int timeout);
};

#endif // RAK3172_H