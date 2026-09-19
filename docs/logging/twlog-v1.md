# TWLG v1 realtime runtime log

TWLG removes BLE from the timing-critical control and identification path.
The ESP32 samples state and actuation in the same 1 kHz control cycle, copies a
fixed binary record into SRAM, and writes buffered batches to a dedicated flash
partition only when flash writes are allowed. BLE is used after the experiment
to download the frozen binary log.

## Timing architecture

```text
1 kHz control task
  sensor update
  state estimator
  control / Vq decision
  32-byte SRAM record copy
          |
          v
    32 KiB RAM buffer
          |
          | flash writes allowed outside critical local windows
          v
   dedicated twlog partition

experiment complete -> BLE binary dump -> PC decoder
```

A near-upright controller must call `setFlashWritesAllowed(false)` before its
critical local window and re-enable writes afterwards. Records continue entering
SRAM while flash programming is paused. Sector erase is performed by `log
prepare` before recording, never by the 1 kHz control task.

## Flash layout

The ESP32-WROOM-32 build uses a 4 MB flash map. The `twlog` raw data partition
starts at `0x190000` and has size `0x270000` (~2.44 MiB).

Within that partition:

```text
0x0000 .. 0x003f   64-byte finalized TWLG header
0x0040 .. 0x0fff   reserved/padding in physical flash
0x1000 ..          32-byte runtime records
```

The BLE dump hides the physical padding. A downloaded `.twlog` file is simply:

```text
[64-byte header][record 0][record 1]...[record N-1]
```

## Header

All fields are little-endian. Header size is 64 bytes.

| Field | Type | Meaning |
| --- | --- | --- |
| magic | u32 | bytes `TWLG` |
| version | u16 | `1` |
| header_size | u16 | `64` |
| record_size | u16 | `32` |
| sample_period_us | u16 | `1000` |
| record_count | u32 | committed records |
| payload_bytes | u32 | `record_count * 32` |
| dropped_records | u32 | records rejected because RAM/capacity was exhausted |
| payload_crc32 | u32 | standard CRC-32 of record payload |
| flags | u32 | bit 0 = finalized/complete |
| reserved[0] | u32 | physical flash payload offset (`4096`) |
| reserved[1] | u32 | flash bytes erased/prepared for this run |
| reserved[2..7] | u32 | reserved |

## 32-byte runtime record

```text
u32   t_us
f32   theta_rad
f32   theta_rate_rad_s
f32   wheel_rate_rad_s
f32   vq_v
f32   accel_weight
u32   fault_mask
u16   flags
u16   raw_count
```

The timestamp, body state, wheel state, and `Vq` are snapshots from the same
firmware control cycle. BLE arrival time is not part of the measurement model.

Record flag assignments:

- bit 0 encoder sample valid
- bit 1 wheel rate valid
- bit 2 IMU sample valid
- bit 3 attitude valid
- bit 4 motor active
- bit 5 FOC mode
- bit 6 open-loop mode
- bit 7 calibration mode
- bit 8 safety fault latched
- bit 9 critical-window marker
- bits 10/11/12 vertex A/B/C
- bit 13 local probe active
- bit 14 swing pump active

The vertex/probe/pump flags are reserved for the firmware-side autonomous swing
state machine; the base logger already preserves their bit assignments.

## Shell workflow

The firmware interface is designed around:

```text
log status
log prepare [seconds]
log start
log critical on|off
log stop
log dump
```

`log prepare` erases the required flash sectors asynchronously. `log start` is
accepted only after state becomes `ready`. `log stop` drains SRAM, writes the
header/CRC, and moves to `complete`. `log dump` is valid only for a complete log
and streams binary after an ASCII length marker.

Host-side download and decoding:

```powershell
python tools/parameter_id/download_log_ble.py `
  -o artifacts/run-01.twlog

python tools/parameter_id/decode_twlog.py `
  artifacts/run-01.twlog `
  -o artifacts/run-01.csv
```

The binary downloader reads the exact length announced by firmware. The decoder
checks magic, format sizes, total length, and payload CRC before writing CSV.
