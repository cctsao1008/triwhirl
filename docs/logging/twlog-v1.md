# TWLG v1 realtime runtime log

TWLG removes BLE from the timing-critical control and identification path.
The ESP32 samples state and actuation in the same nominal 1 kHz control cycle,
copies a fixed binary record into SRAM, and writes buffered batches to a
dedicated flash partition only when flash writes are allowed. BLE is used for
supervisory control and, after the experiment, to download the frozen binary
log.

## Timing architecture

```text
nominal 1 kHz control task
  sensor update
  state estimator
  control / Vq decision
  32-byte SRAM record copy
          |
          v
  32 KiB FreeRTOS stream/ring buffer
          |
          | 256-byte flash batches outside critical local windows
          v
   dedicated twlog partition

experiment complete -> BLE binary dump -> PC decoder
```

The 32 KiB stream buffer is the producer/consumer decoupling layer. A dedicated
ping-pong buffer is not required for the current logger because records are
small, fixed-size, and asynchronous flash draining benefits from the elasticity
of a ring buffer. A double buffer may still be useful later for DMA-style sensor
acquisition or a fixed block-processing pipeline, but it is not part of TWLG v1.

A near-upright controller must call `setFlashWritesAllowed(false)` before its
critical local window and re-enable writes afterwards. Records continue entering
SRAM while flash programming is paused. Sector erase is performed by `log
prepare` before recording, never by the 1 kHz control task.

`sample_period_us=1000` in the header is the nominal control period, not a claim
that every adjacent record is exactly 1000 us apart. Each record carries the
firmware `t_us` timestamp. Host inspection therefore reports both nominal timing
and the measured timestamp-delta distribution; identification and fitting should
use actual timestamps when timing jitter matters.

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
| sample_period_us | u16 | nominal period, currently `1000` |
| record_count | u32 | committed records |
| payload_bytes | u32 | `record_count * 32` |
| dropped_records | u32 | records rejected because prepared capacity or SRAM was exhausted |
| payload_crc32 | u32 | standard CRC-32 of record payload |
| flags | u32 | bit 0 = finalized/complete |
| reserved[0] | u32 | physical flash payload offset (`4096`) |
| reserved[1] | u32 | flash bytes erased/prepared for this run |
| reserved[2..7] | u32 | reserved |

A nonzero `dropped_records` count invalidates the run as a complete
identification capture and should be investigated rather than hidden by a larger
host-side reserve.

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

Signal inspection must honor these validity bits. For example, zero-valued
`theta_rad` fields are not treated as a valid body-angle measurement when the
attitude-valid bit is clear.

## Firmware shell workflow

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

## Host workflow

The preferred host interface is the unified toolbox:

```powershell
python tools/twtool.py log session 5 `
  -o artifacts/run-01.twlog `
  --csv artifacts/run-01.csv

python tools/twtool.py log inspect artifacts/run-01.twlog
```

`log session` keeps one BLE control connection open from prepare through start,
the requested recording interval, and stop/finalization. This prevents BLE scan
and reconnect latency from extending the recording interval and consuming the
prepared record reserve. Binary download reconnects only after recording has
completed.

`log inspect` validates the payload CRC and reports nominal metadata plus actual
`t_us` cadence (`min/mean/median/max`, counts above 1250 us and 2000 us), dropped
records, validity coverage for encoder/wheel/IMU/attitude, and signal ranges only
when the corresponding validity flags are present.

Firmware diagnostics are also available without `idf.py monitor`:

```powershell
python tools/twtool.py diag timing
python tools/twtool.py diag timing-reset
python tools/twtool.py diag timing-test 5
python tools/twtool.py diag imu
```

The binary downloader reads the exact length announced by firmware. The decoder
checks magic, format sizes, total length, and payload CRC before writing CSV.
