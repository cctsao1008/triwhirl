# ESP32 realtime API audit

This note records the ESP32 / ESP-IDF constraints that govern TriWhirl's 1 kHz
firmware-owned control loop. It is intentionally narrower than the general
architecture document: the goal is to prevent timing changes from being driven
by trial-and-error when the platform contract already determines the answer.

## Authority order

For realtime behavior, use this order when sources disagree or an old example is
ambiguous:

1. ESP32 datasheet / technical reference manual.
2. ESP-IDF documentation for the exact major/minor release in use.
3. ESP-IDF driver source for that release.
4. TriWhirl hardware measurements.
5. Older ESP-IDF manuals only as historical background.

TriWhirl currently targets ESP-IDF 6.1 on a dual-core ESP32-WROOM-32 class
module.

## Control ownership

The ESP32 owns sensor acquisition, estimation, safety, control, FOC actuation,
and control-rate logging decisions. Host tools are supervisory only. The host
must never be part of the realtime control loop.

## Core and interrupt affinity

ESP32 external peripheral interrupts are allocated on the CPU core that performs
the allocation. Therefore peripheral creation must be done from a task pinned to
the core which should service that peripheral's ISR.

Current intended mapping:

- Core 0: NimBLE, logger worker, AS5600 worker, I2C0 ISR.
- Core 1: 1 kHz control task, MPU6050 path, I2C1 ISR, GPTimer release ISR.

The two sensors are on independent hardware I2C controllers, so blocking I2C
transactions can overlap in wall-clock time when issued from different cores.

## GPTimer release policy

GPTimer is preferred over FreeRTOS tick delay for the 1 kHz release because the
measured tick-based loop became strongly bimodal once execution time fell below
1 ms.

Important API rules:

- `alarm_count` is an absolute counter target.
- If the running counter has already passed `alarm_count`, the alarm fires
  immediately.
- A one-shot alarm uses `auto_reload_on_alarm = false`; the timer counter keeps
  running after the event.
- The callback runs in ISR context and must remain short and non-blocking.

TriWhirl policy on an overrun is **skip, do not catch up**:

1. Advance the absolute schedule past every missed release.
2. Arm the first future boundary.
3. Block until that boundary.
4. Never launch an immediate catch-up iteration.

This avoids both sub-millisecond catch-up bursts and a periodic timer ISR firing
inside the active MPU/control critical path.

## I2C policy

Use the synchronous ESP-IDF master APIs on the two independent controllers.
The asynchronous master path is still documented as experimental and is not
needed for TriWhirl's current architecture.

The AS5600 and MPU6050 drivers may retain their blocking transactions because
parallelism comes from the two hardware controllers and two cores, not from a
host-side or asynchronous control authority.

The ESP-IDF master transaction APIs are internally serialized per bus, but
TriWhirl still guards AS5600 state that is outside the driver's transaction API
(e.g. persistent register pointer / local timing state) with its own mutex.

## Flash is a hard realtime boundary

This is the most important platform constraint for logging.

On ESP32, internal SPI flash operations on SPI0/1 disable caches. Under the
default flash concurrency model, non-IRAM-safe interrupts are disabled, other
tasks are suspended, and the other core is held in a polling loop while the
flash operation completes.

Consequences:

- Moving only the GPTimer or I2C ISR into IRAM cannot make the normal C++
  control task continue through an internal flash page program.
- `CONFIG_I2C_ISR_IRAM_SAFE` and `CONFIG_GPTIMER_ISR_CACHE_SAFE` can improve ISR
  service while cache is disabled, but they do **not** make the whole control
  task flash-write-safe.
- The existing ~256-byte TWLG page-program cadence therefore represents a real
  architectural source of realtime jitter, not merely a slow logger function.

For hard 1 kHz balance windows, internal flash writes must be disabled. Logging
must use SRAM buffering, decimation, shorter capture windows, or a future
storage/transport architecture which does not stall the ESP32 instruction
cache. Long identification runs may deliberately trade deterministic timing for
continuous flash capture; that is a different operating mode and must not be
confused with the hard-realtime balance mode.

## IRAM policy

Do not put large object files into IRAM merely because they are on a hot path.
Use IRAM in this order:

1. Required ISR/cache-safe dependencies specified by ESP-IDF Kconfig.
2. Small, measured timing-critical functions with proven cache-miss cost.
3. Larger linker placement only after `idf.py size-components` demonstrates
   acceptable IRAM consumption and hardware timing shows a real benefit.

ESP-IDF 6.x places most FreeRTOS functions in flash by default. Therefore an
ISR cannot be declared cache-safe merely because the user callback itself has
`IRAM_ATTR`; its complete transitive call/data graph must also be internal.

## Power-management policy

TriWhirl uses fixed clocks. `CONFIG_PM_ENABLE` is explicitly disabled and the
CPU frequency is fixed at 240 MHz. Dynamic frequency scaling and automatic
light sleep are inappropriate for the control image because ESP-IDF documents
additional interrupt latency when power management is active.

## Current optimization order

1. Stable 1 kHz release semantics: GPTimer one-shot, absolute phase, skip missed
   releases.
2. Preserve dual-controller parallel sensor acquisition and verified interrupt
   affinity.
3. Validate stopped-motor timing with flash writes inactive.
4. Measure active FOC timing.
5. Define a hard-realtime logging mode in which internal flash writes are
   prohibited during balance-critical windows.
6. Only then evaluate QIO/80 MHz, ESP-IDF cache-safe ISR Kconfig, selective IRAM,
   or MPU FIFO if a measured bottleneck remains.

## Items explicitly not treated as solved

- Active-FOC 1 kHz deadline margin has not yet been demonstrated.
- Continuous 1 kHz TWLG writes to internal flash are not compatible with a
  strict uninterrupted 1 kHz task deadline under the default ESP32 flash model.
- Enabling cache-safe ISR options does not remove the task-suspension effect of
  internal flash programming.
