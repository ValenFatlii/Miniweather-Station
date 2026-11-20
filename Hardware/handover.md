# Hardware handover

## Identity and evidence

This handover uses the 10 November 2026 firmware baseline and the owner's corrected 20 October PCB layer package. The earlier, incorrectly supplied PCB PDF is not the hardware reference.

The layer package contains Top/Bottom copper, silkscreen, paste-mask and solder-mask PDFs, BoardOutLine, Multi-Layer, Document, and Hole. Visual features agree with the local Rev D CAD family, but exact netlist/manufacturing equivalence and physical acceptance remain to be recorded.

## Owner-confirmed information

| Item | Confirmed information |
|---|---|
| Battery | 12 V 8 Ah |
| SCC | Maximum 10 A; reported discharge setting 10.7 V |
| Buck | MP1584, nominal 5 V / 3 A |
| Anemometer | Pulse output |
| UART | Reserved, no sensor installed |
| Modem control | GPIO32 HIGH enables the modem |

Record the exact SCC, modem, DevKit, sensor, and TS module identities, physical photographs, harness labels, and measured supply values. A rating is not an acceptance-test result.

## CAD connection reference

| Function | Connection |
|---|---|
| Main I2C | SDA21 / SCL22, 100 kHz |
| Pulse wind / rain | GPIO14 / GPIO13 |
| Ultrasonic | TRIG4; ECHO through U1 channel 1 to IO5 |
| SD | CS15, MOSI23, MISO19, SCK18 in the CAD |
| RGB LED | R25 / G26 / B27 |
| Modem switch | GPIO32 through R4 to TS |
| Reserved UART | TX16, buffered RX17; no sensor installed |

Four-pin I2C1–I2C6 pad order is 1 SDA, 2 SCL, 3 GND, 4 3V3. Five-pin U7–U9 order is 1 3V3, 2 GND, 3 SCL, 4 SDA, with a fifth pad unused by the firmware. These arrangements are different. CAD pad numbering is not an assumed cable-view order; identify pin 1 and verify the actual harness.

RAIN and ANEMO CAD headers use 1 GND, 2 3V3, 3 signal (IO13/IO14). JSN uses 1 GND, 2 JSN_ECHO_5V, 3 IO4, 4 +5V. SD uses 1 3V3, 2 CS, 3 MOSI, 4 CLK, 5 MISO, 6 GND. Check the fitted sensor/module's own requirements before connecting it.

U1 is specified as SN74LVC2G17DBVR supplied from 3V3 with C16 100 nF. CAD channel 1 routes JSN_ECHO_5V to IO5; channel 2 routes reserved WIND_TX_5V to IO17. Do not assume an unbuffered 5 V signal is acceptable at the ESP32.

## Power-path reference

In the CAD, the protected input passes through INA219 before the converter branches. BC1 feeds the logic rail through D3. BC2 feeds the modem through TS. USB return is a separate output net, not automatically the main GND connection; confirm the actual module's switching topology before adding grounded cables or instruments.

Confirm whether the station is supplied from the SCC LOAD output or directly from the battery. The reported 10.7 V discharge function must protect the actual branch, not an unrelated output.

The resistor-label/MPN metadata, capacitor ratings/polarity, footprint compatibility, and manufacturing-file revision require reconciliation before re-ordering or fabrication. Historical zero-DRC records do not certify electrical, thermal, RF, or environmental performance of the installed unit.

## Commissioning record

| Check | Record required |
|---|---|
| Unit identity | PCB/CAD/PDF/Gerber correlation; exact DevKit and fitted modules |
| Unpowered inspection | Polarities, continuity, shorts, connector pin 1, level buffering, harness labels |
| Supply | Source, BC1, rail after D3, BC2, USB, and 3V3 under relevant loads |
| Modem | Startup/transmission voltage, ON/OFF switching, no unintended alternate supply path |
| Sensors | Addresses, known pulse response, vane alignment, tip calibration, ultrasonic reference/range |
| Storage | CSV/outbox/journal/checkpoint behavior and controlled power-interruption evidence |
| Delivery | Live ACK, paired backlog, message ID/timestamp handling, database confirmation |
| Field run | Dates, duration, environment, resets, faults, maintenance, and responsible person |

Follow the identified SCC manufacturer's connection sequence. Allow the firmware's initial 120-second modem boot phase. Use Serial 115200 when available and retain SD event logs. Measurements and queues can exist before the network is ready.

## Maintenance and open items

Back up journal and checkpoints together during controlled maintenance. Check SD capacity, weather sealing, corrosion, mounting, moving wind parts, collector cleanliness, panel contamination, and sensor reference alignment.

Open items include the exact SCC model/type, panel label, complete as-built BOM/harness, manufacturing-file reconciliation, measured voltage/current/transient/thermal results, current installation photos, calibration records, and handover contacts. Owner-confirmed pulse mode and unused UART status are no longer open questions.

OTA, NVS provisioning, stronger WSS certificate verification, and further power/storage improvements are described in [Future Development](../Documentations/10.%20Future%20Development.md).
