# Miniweather Station

An ESP32 weather station for environmental monitoring, local microSD logging, and MQTT over WebSocket Secure transmission to Hyperbase.

The production baseline documented here is **10 November 2026**, identified as **2026.11.10** in the version files. The public sketch contains placeholders for network and service configuration. The separate WiFi/RSSI OTA experiment is not integrated into this production firmware.

## Current operation

- Sensor sampling targets a one-second interval. Results are aggregated into 60-second windows.
- Most parameters use time-weighted means. Wind direction uses a circular mean; rainfall is accumulated; electrical energy is integrated.
- Two configured WiFi profiles support AP selection, roaming, and recovery through an alternative AP.
- MQTT publications use QoS 1. A successful live PUBACK permits at most one backlog attempt for that cycle.
- A GPIO-controlled switch can power-cycle the USB modem when the recovery policy has sufficient evidence of a fault.
- The status LED checks recent live delivery and faults; it is not a database-write confirmation.

## Hardware configuration

| Item | Current information |
|---|---|
| Controller | ESP32 DevKit, 4 MB flash reported by the owner |
| Environmental sensors | SCD40, BMP280, BH1750 |
| Wind and rain | AS5600 vane, pulse anemometer, tipping bucket |
| Water level | JSN-SR04T ultrasonic sensor |
| Electrical monitoring | INA219 |
| Battery | 12 V, 8 Ah, confirmed by the owner |
| Solar charge controller | Maximum rating 10 A; discharge setting reported as 10.7 V; exact model/type still to be recorded |
| Buck converter | MP1584, nominal 5 V / 3 A, confirmed by the owner |
| Connectivity | USB WiFi/4G modem and an alternative WiFi network |
| UART header | Reserved; no UART sensor is installed |

The 3 A converter rating is not a measured transient-current result. The 10.7 V controller setting must be checked against the actual load connection and controller function.

## Documentation

1. [Project Overview](Documentations/01.%20Project%20Overview.md)
2. [System Architecture](Documentations/02.%20System%20Architecture.md)
3. [Hardware Design](Documentations/03.%20Hardware%20Design.md)
4. [Sensor Configuration](Documentations/04.%20Sensor%20Configuration.md)
5. [Firmware Design](Documentations/05.%20Firmware%20Design.md)
6. [Communication System](Documentations/06.%20Communication%20System.md)
7. [Power Management](Documentations/07.%20Power%20Management.md)
8. [Calibration and Testing](Documentations/08.%20Calibration%20and%20Testing.md)
9. [Deployment](Documentations/09.%20Deployment.md)
10. [Future Development](Documentations/10.%20Future%20Development.md)

See the [firmware guide](Firmware/README.md), [hardware handover](Hardware/handover.md), and [change history](CHANGELOG.md).

## Data contract

`current` is in **mA**. The legacy field `power` now represents **energy per 60-second window in mWh**, with `power_unit: "mWh"`. `ketinggian_air_laut` is local water level in **cm**, not geographic elevation. Consumers must use these meanings when ingesting the baseline data.

## Documentation status

The owner updated the system, hardware, and sensor architecture diagrams on 20 November 2026. Chapters 2, 3, and 4 reference those versions. Other figures remain pending a separate owner-led revision, and some deployment image links are unresolved. Use the written baseline and verified unit information when interpreting block diagrams and wiring details.

Historical calibration results are retained with their original context. New firmware behavior and physical hardware still require the versioned acceptance tests described in the testing document.
