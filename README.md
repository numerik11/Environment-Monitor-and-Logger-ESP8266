                                          # Environment Monitor

An ESP8266-based CO₂, temperature and humidity monitor with an on-device TFT display and a Wi-Fi dashboard.

The default build uses an SCD30 sensor. An alternative build supports a Senseair S8 CO₂ sensor with a DHT11. The display can be configured for a 240 × 320 ILI9341 or a 170 × 320 ST7789.

## Features

- Live readings and zoomable history graphs in the web dashboard
- Four-minute samples stored in LittleFS, with a rolling 30-day history
- “Today (live)” view and one-click PNG graph export
- Wi-Fi setup through a WiFiManager captive portal and access at `envmonitor.local`
- Six TFT screens covering current readings, statistics, and 12- and 24-hour graphs
- Button navigation and, in the SCD30 build, confirmed 400 ppm calibration

## Sensor options

- **SCD30 (default):** CO₂, temperature and humidity over I²C
- **Senseair S8 + DHT11:** CO₂ over UART, temperature and humidity from the DHT11. Build with `-DUSE_DHT_S8=1`.



