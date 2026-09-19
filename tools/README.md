# TriWhirl host tools

This directory is reserved for programs that run on the development host rather than on the ESP32.

Expected responsibilities include:

- telemetry capture and log conversion;
- calibration and experiment automation;
- local plant identification;
- model construction and linearization;
- robust H-infinity / LMI synthesis;
- simulation and Monte Carlo validation;
- generation or verification of firmware constants.

Host tools may consume logs and emit configuration or generated data, but they are not part of the real-time ESP32 control loop.

Do not create a separate `host/` application hierarchy unless TriWhirl later gains an actual host-side runtime application.
