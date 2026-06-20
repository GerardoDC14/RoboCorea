# firmware/VESC — LispBM scripts that run *on* the VESCs

These scripts run inside the VESC motor controllers (VESC Tool → **Scripting → LispBM**),
not on the ESP32. They are part of the drivetrain control loop and are kept here
so they're easy to find and edit alongside the ESP firmware.

## `flipper_position.lisp`

Closes the **flipper position loop on the VESC itself** (PD + stiction
feedforward, shortest-path error on a wrapped `[0,360)` angle so it can spin past
360° continuously). The ESP32 only sends a target angle and receives the measured
angle back, over custom CAN frames — see the header in the file.

| | |
|---|---|
| Runs on | the **4 flipper VESCs only** (CAN ids `20, 10, 40, 30` — FL, FR, RL, RR per `1.ino`) |
| Traction VESCs (`60`, `50`) | **stock firmware**, driven with `SET_RPM` by the ESP — do **not** flash this |
| Target FW | VESC **6.06** |
| Talks to | `firmware/ESP` — `CANInterface::sendFlipperAngles()` / the flipper report parser |

### Wire protocol (must match `firmware/ESP/include/config.h`)

```
ESP32 → VESC   ext-id = (VESC_CMD_FLIPPER_TARGET << 8) | id   [int32 BE millideg][u8 enable]
VESC  → ESP32  ext-id = (VESC_CMD_FLIPPER_REPORT << 8) | id   [int16 BE deci-deg measured]
```
`VESC_CMD_FLIPPER_TARGET = 0x7E`, `VESC_CMD_FLIPPER_REPORT = 0x7F`.

### Flashing
1. VESC Tool → connect to a flipper VESC → **App Settings**: confirm its **CAN id**
   (the script reads it via `conf-get 'controller-id`).
2. **Scripting → LispBM** → open `flipper_position.lisp` → **Upload** → **Run**.
3. Tick **"Run at startup"** so it survives a power cycle.
4. Repeat for all four flipper VESCs (the file is identical for each).

### Bench-verify before powered motion
- **`deg-per-dist`** — drive the output shaft a known angle by hand and confirm the
  reported angle matches. This single constant sets the whole scale.
- The LispBM API names used here (`event-can-eid`, `can-send-eid`, `bufget-i32`
  with `'big-endian`, `secs-since`, `conf-get 'controller-id`) are correct for
  recent VESC builds but **confirm against your 6.06** — they are the main unknown.
- Confirm command bytes `0x7E`/`0x7F` aren't used by a `CAN_PACKET_*` in your build.
- Start with a low `i-max` until the gains are trusted.
