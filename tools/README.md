# TriWhirl host tools

This directory contains programs that run on the development host rather than on the ESP32.

Current tool:

- `logging/capture.py` captures the runtime UART telemetry stream into schema-versioned CSV logs.

Expected responsibilities also include:

- calibration and experiment automation;
- local plant identification;
- model construction and linearization;
- robust H-infinity / LMI synthesis;
- simulation and Monte Carlo validation;
- generation or verification of firmware constants.

Host tools may consume logs and emit configuration or generated data, but they are not part of the real-time ESP32 control loop.

Do not create a separate `host/` application hierarchy unless TriWhirl later gains an actual host-side runtime application.
