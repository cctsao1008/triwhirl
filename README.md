# TriWhirl

Robust control of a reaction-wheel Reuleaux triangle on ESP32.

## Host toolbox

Use the unified host entry point for logging, identification, fitting, and post-run analysis:

```powershell
python tools/twtool.py --help
```

The realtime control and synchronized 1 kHz TWLG capture remain firmware-owned; BLE is used for configuration and post-run transfer rather than as a realtime timing path.

## Documentation

- [Architecture](docs/architecture.md)
- [Hardware](docs/hardware.md)
- [Development](docs/development.md)
- [Motor bring-up](docs/motor-bringup.md)
- [Host tools](tools/README.md)
- [Parameter identification](tools/parameter_id/README.md)

## License

MIT