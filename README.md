                                           - Envinroment Monitor (ESP8266 Based) -

- Wi-Fi enabled CO₂ / temperature / humidity monitor using an -

- SCD30 + ILI9341 TFT or

- 130*320 TFT + NDIR CO2 Sensor(C8) w/DHT11.
  
- Logs 2-minute samples to LittleFS (30-day rolling history), serves a modern web dashboard with “Today (live)” + zoomable graphs,
  
- Includes WiFiManager captive portal, mDNS, on-device multi-screen UI with button navigation, and one-tap
  
- PNG export from the web graph.

- Button to scroll through Stats/12/24hr graphs on screen. Hold button to set co2 reading to 400ppm.

                              ===================== Pins. (Wemos D1 R2/Mini) ===================== 

- #define CO2_TX_PIN  D0  // Senseair S8 UART TX (no interrupt needed)

- #define TFT_RST     D1

- #define TFT_DC      D2

- #define CO2_RX_PIN  D4  // Senseair S8 UART RX (needs interrupt-capable pin; avoid SPI pins)

- #define DHT_PIN     D6

- #define BUTTON_PIN  -1   // set to -1 to disable button handling

- #define BL_PIN      D3

- #define TFT_CS      D8


                                 ===================== User Interface. =====================
UI

<img width="874" height="702" alt="image" src="https://github.com/user-attachments/assets/3df14214-c6ac-40b7-aee9-07674b1039bd" />

TFT

<img width="650" height="355" alt="image" src="https://github.com/user-attachments/assets/172346d1-8ed0-470f-854a-f34563e7f5d4" />




