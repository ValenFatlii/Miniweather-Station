# Hardware files

## Files currently published

| File | Purpose |
|---|---|
| [pcb_layout.pdf](pcb_layout.pdf) | Archived PCB layout export |
| [gerber.zip](gerber.zip) | Archived manufacturing-file package |
| [handover.md](handover.md) | Hardware identification, wiring reference, commissioning, and open verification items |

The owner supplied a separate 20 October PDF layer package as the current PCB reference. It contains twelve layer PDFs with BC1/BC2, TS, USB, U1, and C16 features consistent with the local Rev D CAD reference. These source/manufacturing assets have not been replaced in this text update; reconcile them before treating the archived public files as the final as-built revision.

## Confirmed configuration

- Battery: 12 V, 8 Ah.
- SCC: maximum 10 A rating, reported discharge setting 10.7 V; exact model/type still to be recorded.
- Buck: MP1584 nominal 5 V / 3 A; loaded/transient capability still to be measured.
- Anemometer: pulse output, GPIO14 in firmware.
- UART: reserved header, no UART sensor installed.
- Modem switch: GPIO32 HIGH-on confirmed by the owner.

The CAD reference includes separate controller/sensor and modem branches, INA219 input-side monitoring, level buffering, and different four-/five-pin I2C header arrangements. Use the [handover](handover.md) and a physical continuity check before wiring a harness.

Schematic/BOM/3D files are not implied to be published merely by listing their roles. Record and publish reconciled source files when available. Existing diagram/image assets remain for the owner's separate update.
