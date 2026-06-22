# firmware/VESC — LispBM scripts that run *on* the VESCs

These scripts run inside the VESC motor controllers (VESC Tool → **Scripting → LispBM**),
not on the ESP32. They are part of the drivetrain control loop and are kept here
so they're easy to find and edit alongside the ESP firmware.

## `flipper_position.lisp`

Closes the **flipper position loop on the VESC itself** (PD + stiction
feedforward, shortest-path error on a wrapped `[0,360)` angle so it can spin past
360° continuously). The ESP32 sends the target through the reliable fake-RPM
carrier (`SET_RPM = target_degrees * 1000`), and derives measured angle from the
VESC STATUS_5 tachometer.

| | |
|---|---|
| Runs on | the **4 flipper VESCs only** (CAN ids `20, 10, 40, 30` — FL, FR, RL, RR per `1.ino`) |
| Traction VESCs (`60`, `50`) | **stock firmware**, driven with `SET_RPM` by the ESP — do **not** flash this |
| Target FW | VESC **6.06** |
| Talks to | `firmware/ESP` — `CANInterface::sendFlipperAngles()` / STATUS_5 tachometer parser |

### Wire protocol (must match `firmware/ESP/include/config.h`)

```
ESP32 -> VESC   ext-id = (CAN_PACKET_SET_RPM << 8) | id   [int32 BE target_degrees * 1000]
VESC  -> ESP32  normal VESC STATUS_5 broadcast            [int32 BE tachometer][int16 BE input_voltage * 10]
```

This intentionally avoids the custom `0x7E`/`0x7F` Lisp CAN event path, which has
been unreliable on the tested VESC firmware build.

### Flashing
1. VESC Tool → connect to a flipper VESC → **App Settings**: confirm its **CAN id**
   and enable STATUS_5 broadcasts.
2. **Scripting → LispBM** → open `flipper_position.lisp` → **Upload** → **Run**.
3. Tick **"Run at startup"** so it survives a power cycle.
4. Repeat for all four flipper VESCs (the file is identical for each).

### Bench-verify before powered motion
- **`deg-per-dist`** — this sets the VESC-side control-loop scale.
- **`FLIPPER_TACH_DEG_PER_COUNT_*`** in `firmware/ESP/include/config.h` — this sets
  the ESP-side telemetry scale from STATUS_5 tachometer counts.
- Start with a low `i-max` until the gains are trusted.
