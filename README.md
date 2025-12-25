                                           - Envinroment Monitor (ESP8266 Based) -

- Wi-Fi enabled CO₂ / temperature / humidity monitor using an SCD30 + ILI9341 TFT.
  
- Logs 5-minute samples to LittleFS (30-day rolling history), serves a modern web dashboard with “Today (live)” + zoomable graphs,
  
- Includes WiFiManager captive portal, mDNS, on-device multi-screen UI with button navigation, and one-tap
  
- PNG export from the web graph.

                              ===================== Pins. (Wemos D1 R2/Mini) ===================== 

#define CO2_TX_PIN  D0  // Senseair S8 UART TX (no interrupt needed)
#define TFT_RST     D1
#define TFT_DC      D2
#define CO2_RX_PIN  D4  // Senseair S8 UART RX (needs interrupt-capable pin; avoid SPI pins)
#define DHT_PIN     D6
#define BUTTON_PIN  -1   // set to -1 to disable button handling
#define BL_PIN      D3
#define TFT_CS      D8


                                 ===================== User Interface. =====================

<img width="863" height="702" alt="image" src="https://github.com/user-attachments/assets/b80e6f18-e9b9-4692-a44c-9d2dd6089b90" />

