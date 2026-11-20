# Calibration parameters

## Regression convention

The report coefficients describe `sensor_reading = a * reference_value + b`. The firmware estimates the reference using the inverse `corrected = (sensor_reading - b) / a`.

These symbolic coefficients describe the inline `cal_*` functions; they are not separate `TEMP_SLOPE`-style variables or an external calibration header.

| Parameter | a | b | Implemented correction |
|---|---:|---:|---|
| Temperature | 0.9976 | -0.0896 | `(raw + 0.0896) / 0.9976` |
| Humidity | 1.0186 | -0.0879 | `(raw + 0.0879) / 1.0186`, clamp 0–100 |
| CO2 | 1.0058 | -134.3519 | `(raw + 134.3519) / 1.0058` |
| Pressure | 1.1781 | -178.3727 | `(raw + 178.3727) / 1.1781` |
| Altitude | 0.9641 | 0.2240 | `(raw - 0.2240) / 0.9641` |
| Illuminance | 1.0021 | -4.7057 | `(raw + 4.7057) / 1.0021` |
| Wind direction | 0.9618 | -2.0622 | `(sensor_angle + mechanical_offset + 2.0622) / 0.9618`, then wrap |
| Wind speed | 0.8858 | 0.1804 | `(speed_raw - 0.1804) / 0.8858`, clamp non-negative |
| Rainfall | 1.1678 | -0.2808 | Zero stays zero; positive total uses `(raw_total + 0.2808) / 1.1678` |
| Water level | -1.0257 | 88.0000 | `(88.00 - distance_cm) / 1.0257`, clamp 0–85 cm |

Additional non-negative clamping is used where present in the source. Consult the function implementation before changing limits.

## Pulse and angular processing

Anemometer pulse frequency is converted by `-0.0181*f*f + 1.3859*f + 1.4055`, with raw values below 1.5 set to zero before the wind-speed correction.

AS5600 raw angle uses `raw * 360 / 4096`. Corrected direction is wrapped and integrated as sine/cosine for a circular mean. Mechanical north alignment must be recorded.

Rain starts from `0.2 mm * ticks_since_boot`. The corrected total is differenced across reporting boundaries, so the intercept is not added again every minute. The first tick after boot is approximately 0.4117 mm before rounding, and later ticks add approximately 0.1713 mm each before rounding. The boot total is not a persistent daily total.

## Output precision

| Output | Decimal places |
|---|---:|
| Temperature, wind speed, rainfall | 1 |
| Humidity, CO2, pressure, altitude, lux, water level, direction | 0 |
| Voltage | 2 |
| Current | 1 |
| Energy in mWh | 4 |

Round after aggregation/integration, not before. Missing values are not fabricated zeros. Historical regression/error results and the new validation plan are in [Calibration and Testing](../Documentations/08.%20Calibration%20and%20Testing.md).
