# KMP_PRODINo

Universal PlatformIO / Arduino library for the KMP Electronics **ProDino ESP32**,
**SMARTI8**, **MADE4HOME** and **MADE8HOME** industrial controllers.

It provides a hardware abstraction layer (relays, opto-isolated inputs, RGB status
LED, RS-485) plus drivers for the on-board communication modules (W5500 Ethernet,
GSM, RAK3172 LoRa, RFM95) and a set of ready-to-build examples.

- Manufacturer: [KMP Electronics](https://kmpelectronics.eu/)
- Product page: <https://kmpelectronics.eu/products/prodino-esp32-v1/>

## Supported boards

| Class | Boards |
|-------|--------|
| `KMPProDinoESP32Class` | ProDino ESP32 V1, and the Ethernet / GSM / LoRa / LoRa-RFM variants and combinations |
| `KMPSmarti8ESP32Class` | SMARTI8 ESP32 series (two RGB status LEDs) |

`begin()` takes a `BoardType` / `SMARTI8_*` enum so the same firmware can target a
specific hardware variant (which modules get powered up at start).

## Installation (PlatformIO)

Add the dependencies to your `platformio.ini`:

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
upload_speed = 921600
lib_deps =
    makuna/NeoPixelBus@^2.8.0
    https://github.com/kmpelectronics/KMP_PRODINo.git
    https://github.com/kmpelectronics/Ethernet.git
```

> The forked `kmpelectronics/Ethernet` library is required for the on-board W5500.
> Individual examples pull in extra dependencies (ADS1X15, SSD1306, TinyGSM, …) —
> see each example's `platformio.ini`.

## Quick start

```cpp
#include <Arduino.h>
#include "KMPCommon.h"          // RgbColor presets: red, green, blue, ...
#include "KMPProDinoESP32.h"

KMPProDinoESP32Class board;

void setup() {
    Serial.begin(115200);
    board.begin(ProDino_ESP32_Ethernet);   // pick your board variant
    board.setStatusLed(green);
    board.setAllRelaysOff();
}

void loop() {
    // Mirror each opto input onto the matching relay
    for (uint8_t i = 0; i < RELAY_COUNT; i++) {
        board.setRelayState(i, board.getOptoInState(i));
    }
    delay(100);
}
```

## API overview (`KMPProDinoESP32Class`)

| Area | Methods |
|------|---------|
| Init | `begin(board, startEthernet = true, startModem = true)`, `restartEthernet()`, `restartGSM()` |
| Relays (4) | `setRelayState(i/Relay, state)`, `setAllRelaysOn()/Off()`, `setAllRelaysState(state)`, `getRelayState(i)` |
| Opto inputs (4) | `getOptoInState(i/OptoIn)`, `getOptoInState()` (all as a bit mask) |
| Status RGB LED | `setStatusLed(RgbColor)`, `offStatusLed()`, `getStatusLed()`, `processStatusLed(color, blinkInterval)` |
| RS-485 | `rs485Begin(baud[, config])`, `rs485End()`, `rs485Write(...)`, `rs485Read([wait, repeat])`, `rs485print()`, `rs485find()` |

Relays and opto inputs are driven through the on-board MCP23S08 SPI expander; RS-485
uses `Serial1` with automatic DE/RE direction switching.

## Modules & drivers

- **Ethernet** — Wiznet W5500, via the forked `Ethernet` library.
- **GSM** — module on `Serial2`; see the `SMARTI8_A7672E` example (TinyGSM).
- **LoRa — RAK3172** — `RAK3172` driver class (AT-command based), supporting both
  LoRa P2P and LoRaWAN modes. See the `*_RAK3172_*` and `RAK3172_P2P_*` examples.
- **LoRa — RFM95** and reset sequencing handled internally by `begin()`.

## Examples

| Example | Board | Demonstrates |
|---------|-------|--------------|
| `Test` | ProDino / SMARTI8 | Relays + RGB LED via a FreeRTOS task (minimal starter) |
| `PRODINo_ESP32_LoRa_RAK3172_Master_PIO` / `..._Slave_PIO` | ProDino LoRa | Addressed LoRa P2P protocol: read DI, read/set relays remotely |
| `RAK3172_Test_Master_PIO` / `..._Slave_PIO` | ProDino LoRa | RAK3172-only PING/PONG test; slave is a test jig and shows module health on the status LED |
| `RAK3172_P2P_Master_PIO` / `..._Slave_PIO` (+ `_TTESTER_`) | RAK3172 | Lower-level RAK3172 P2P messaging |
| `LoRaWAN_Analog_IO_V1.0` | — | ADS1015 ADC + GP8403 DAC analog I/O |
| `Accu-Speed_TTESTER_PIO` | ProDino | ADS1X15 + SSD1306 OLED + MCP23X17 |
| `NCS_SCC_plus_controller_V1_PIO` | ESP32-S3 | Full controller: ADC, OLED, MCP23X17, OneWire, SC16IS7X0, L9822E |
| `PCNT_FTP_Upload_PIO` | SMARTI8 | ESP32 pulse counter + FTP upload over Ethernet |
| `SMARTI8_A7672E` | SMARTI8 | A7672E GSM modem (TinyGSM) + ADS1X15 |
| `RTC_Addon_RV-3028_PIO` | SMARTI8 | External RV-3028 RTC over I2C |

## License / support

© KMP Electronics Ltd, Bulgaria. For questions and support visit
<https://kmpelectronics.eu/>.
