# Firmware

This folder contains the ESP32 firmware developed for the Miniweather Station.

## Main Controller

- ESP32 DevKit
- Arduino Framework


## Main Program

| File | Description |
|-|-|
| Miniweather_Station.ino | Main firmware program |
| config.h | Network and MQTT configuration |
| calibration.h | Sensor calibration parameters |


## Firmware Function

The firmware performs:

- Sensor initialization
- Environmental data acquisition
- Sensor calibration
- Data processing
- SD card logging
- MQTT communication


## Measurement Interval

The system performs measurement every 60 seconds.

Dynamic sensors:

- Anemometer
- Tipping bucket

are measured continuously during the interval and converted into average values.


## Communication

Data is transmitted using:

- WiFi
- MQTT over WebSocket Secure (WSS)


## Storage

Local backup:

- MicroSD card


## Required Libraries

See:

libraries.md
