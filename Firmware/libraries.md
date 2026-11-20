# Firmware dependencies

## Additional Arduino libraries

| Library | Header | Purpose |
|---|---|---|
| WebSockets / arduinoWebSockets | `WebSocketsClient.h` | WSS transport |
| MQTTPubSubClient | `MQTTPubSubClient.h` | MQTT and QoS 1 acknowledgement handling |
| ArduinoJson | `ArduinoJson.h` | Payload and queue parsing |
| BH1750 | `BH1750.h` | Illuminance |
| Adafruit BMP280 Library | `Adafruit_BMP280.h` | Pressure and altitude |
| Adafruit INA219 | `Adafruit_INA219.h` | Electrical monitoring |
| SparkFun SCD4x Arduino Library | `SparkFun_SCD4x_Arduino_Library.h` | SCD40 readings |

Install the dependencies requested by these libraries, such as Adafruit BusIO/Unified Sensor where applicable. `MQTTPubSubClient` is not the similarly named `PubSubClient`, and the SCD4x header here is SparkFun's rather than the Sensirion library listed in earlier documentation.

## ESP32 core components

`WiFi`, `Wire`, `SPI`, `SD`, `FS`, `time`, FreeRTOS, and ESP reset APIs come from the core/toolchain. AS5600 is accessed directly through `Wire`; no additional AS5600 library is used.

ESP32 core 3.3.3 is the recorded baseline build environment. Record the exact board, partition scheme, and selected library versions with every reproducible build; a directory containing several similarly named libraries can select an unintended implementation.
