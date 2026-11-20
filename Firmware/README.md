# Firmware

## Baseline

`Miniweather_Station.ino` is the public, configuration-sanitized copy of the 10 November 2026 weather-station firmware. The date-based baseline identifier is `2026.11.10`. Calibration and network settings are contained in this one sketch; separate `config.h` and `calibration.h` files are not required.

## Build and configuration

1. Install the ESP32 board package and the dependencies in [libraries.md](libraries.md).
2. Create a local Arduino sketch folder named `Miniweather_Station` and place `Miniweather_Station.ino` inside it. The repository keeps the source under `Firmware`; the local Arduino sketch folder must match the main `.ino` name.
3. Fill both `WIFI_LIST` profiles and the MQTT host, WSS path, topic, and client-ID settings locally. `MODEM_WIFI_SSID` must exactly match the intended modem SSID. Do not publish passwords or sensitive endpoints.
4. Use the actual ESP32 board/flash configuration. The recorded baseline build uses ESP32 core 3.3.3, ESP32 Dev Module, and a default application limit of 1310720 bytes. Record the chosen partition scheme with test evidence.
5. Upload through USB and monitor Serial at 115200. Allow the configured modem boot interval before expecting the first WiFi scan.

The public source was compiled successfully with ESP32 core 3.3.3 and the board configuration above: 1265547 program bytes (96% of the selected application partition) and 53472 static RAM bytes (16%). Rebuild after filling local configuration and inspect the result; static RAM figures do not represent peak runtime TLS heap use.

## Operation

- One-second target sampling; 60-second time-weighted aggregation.
- Circular mean direction; accumulated interval rain; integrated mWh energy.
- Two configured WiFi profiles with AP/BSSID selection and bounded failover.
- WSS MQTT QoS 1. One live ACK permits at most one backlog attempt.
- SD live outbox, append-only backlog/checkpoints, minute CSV, and event CSV.
- GPIO32 HIGH-on modem recovery, TCP-probe evidence, and bounded cycle budget.
- Separate LED supervision of recent live ACK and main-loop freshness.

Unavailable values can be JSON null/CSV nan. Network/SD calls can delay sampling; held-value integration is not a hard real-time guarantee. RAM backlog fallback holds eight records and is lost on power removal.

## Data and calibration

`current` is mA; the legacy MQTT key `power` is mWh energy per minute; CSV names it `energy_mWh`. Local water level is cm under the legacy `ketinggian_air_laut` key. See [calibration.md](calibration.md), [sensor configuration](../Documentations/04.%20Sensor%20Configuration.md), and [communication schema](../Documentations/06.%20Communication%20System.md).

## Maintenance boundaries

PUBACK confirms the broker, not the database. Queue recovery can duplicate records. SD event files do not capture the entire Serial stream. WSS certificate validation needs further work in the current no-CA configuration.

OTA and test-sketch NVS provisioning are not integrated here. See [Future Development](../Documentations/10.%20Future%20Development.md).
