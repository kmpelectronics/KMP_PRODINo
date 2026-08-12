// Stamp-C3U-RS485-Echo
//
// RS485 echo tester on an M5Stamp C3U (ESP32-C3, native USB) with a Stamp PWR485
// RS485 module.
//
// Behaviour:
//   - Receives a message over RS485 (framed by a CR/LF or by an idle gap).
//   - "ping"  -> replies "pong".
//   - anything else -> replies with the string reversed, e.g. "1234" -> "4321".
//   - The on-board WS2812 LED blinks on activity: GREEN while receiving,
//     BLUE while transmitting.
//
// Wiring (Stamp PWR485 -> Stamp C3U):
//   G8  = RS485 RX (RO, receiver output -> ESP32 RX)
//   G10 = RS485 TX (DI, driver input   <- ESP32 TX)
//   The PWR485 is auto-direction (no DE/RE pin), so RS485 is half-duplex: the
//   module echoes our own transmission back on RX, which we discard after each TX.
//
// USB serial (the C3U's native USB) mirrors every exchange for debugging.

#include <Arduino.h>
#include <NeoPixelBus.h>

// ---------------- Config ----------------
#define RS485_RX_PIN 8       // G8  - RO
#define RS485_TX_PIN 10      // G10 - DI
#define RS485_BAUD   9600    // change to match the device under test
#define LED_PIN      2       // on-board WS2812
#define FRAME_GAP_MS 20      // idle gap that ends a message (when no CR/LF)

HardwareSerial RS485(1);     // UART1

NeoPixelBus<NeoGrbFeature, NeoEsp32Rmt0Ws2812xMethod> led(1, LED_PIN);
const RgbColor cGreen(0, 40, 0);
const RgbColor cBlue(0, 0, 40);
const RgbColor cRed(40, 0, 0);
const RgbColor cOff(0);

String rxBuf;
unsigned long lastRxMs = 0;
bool receiving = false;

// ---------------- Helpers ----------------
void ledSet(const RgbColor& c) { led.SetPixelColor(0, c); led.Show(); }

String reversed(const String& s) {
  String r;
  r.reserve(s.length());
  for (int i = s.length() - 1; i >= 0; i--) r += s[i];
  return r;
}

void sendReply(const String& resp) {
  ledSet(cBlue);                          // TX activity
  RS485.print(resp);
  RS485.flush();                          // wait until all bytes are on the wire
  delay(3);                               // let the half-duplex self-echo arrive
  while (RS485.available()) RS485.read(); // ...and discard it
  ledSet(cOff);
}

void processFrame() {
  String msg = rxBuf;
  rxBuf = "";
  receiving = false;
  msg.trim();                             // drop stray spaces / CR / LF
  if (msg.length() == 0) { ledSet(cOff); return; }

  String resp = msg.equals("ping") ? String("pong") : reversed(msg);
  Serial.printf("[rx] \"%s\"  ->  [tx] \"%s\"\n", msg.c_str(), resp.c_str());
  sendReply(resp);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== Stamp-C3U RS485 echo tester ===");
  Serial.printf("RS485 UART1 RX=G%d TX=G%d @ %d 8N1\n", RS485_RX_PIN, RS485_TX_PIN, RS485_BAUD);
  Serial.println("Rules: \"ping\" -> \"pong\";  other -> reversed");

  led.Begin();
  ledSet(cRed);          // booting
  delay(300);
  ledSet(cOff);

  RS485.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
}

void loop() {
  while (RS485.available()) {
    char c = (char)RS485.read();
    if (!receiving) { receiving = true; ledSet(cGreen); }   // RX activity
    lastRxMs = millis();

    if (c == '\r' || c == '\n') {          // terminator ends the frame
      if (rxBuf.length()) processFrame();
    } else {
      rxBuf += c;
    }
  }

  // No terminator? End the frame after an idle gap.
  if (rxBuf.length() && (millis() - lastRxMs) > FRAME_GAP_MS) {
    processFrame();
  }
}
