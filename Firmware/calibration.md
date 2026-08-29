# Calibration Parameters

## Sensor Calibration Constants

The following calibration parameters are implemented in the ESP32 firmware to correct sensor measurement values based on validation results.

| Parameter | Variable | Value |
|---|---|---|
| Temperature slope | `TEMP_SLOPE` | 0.9976 |
| Temperature offset | `TEMP_OFFSET` | -0.0896 |
| Humidity slope | `HUMIDITY_SLOPE` | 1.0186 |
| Humidity offset | `HUMIDITY_OFFSET` | -0.0879 |
| CO₂ slope | `CO2_SLOPE` | 1.0058 |
| CO₂ offset | `CO2_OFFSET` | -134.3519 |
| Pressure slope | `PRESSURE_SLOPE` | 1.1781 |
| Pressure offset | `PRESSURE_OFFSET` | -178.3727 |
| Altitude slope | `ALTITUDE_SLOPE` | 0.9641 |
| Altitude offset | `ALTITUDE_OFFSET` | 0.2240 |
| Light intensity slope | `LUX_SLOPE` | 1.0021 |
| Light intensity offset | `LUX_OFFSET` | -4.7057 |
| Water level slope | `WATER_LEVEL_SLOPE` | -1.0257 |
| Water level offset | `WATER_LEVEL_OFFSET` | 88.0000 |
| Rainfall slope | `RAIN_SLOPE` | 1.1678 |
| Rainfall offset | `RAIN_OFFSET` | -0.2808 |
| Wind speed slope | `WIND_SPEED_SLOPE` | 0.8858 |
| Wind speed offset | `WIND_SPEED_OFFSET` | 0.1804 |
| Wind direction slope | `WIND_DIR_SLOPE` | 0.9618 |
| Wind direction offset | `WIND_DIR_OFFSET` | -2.0622 |

---

## Calibration Equation

The corrected sensor output is calculated using:

\[
Y_{corrected}=aX_{raw}+b
\]

where:

- `a` = slope parameter
- `b` = offset parameter
- `X_raw` = raw sensor measurement
- `Y_corrected` = calibrated measurement output
