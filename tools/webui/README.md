# TriWhirl WebUI

Static Web Bluetooth engineering console for the native ESP-IDF runtime.

## Run locally

From the repository root:

```powershell
python -m http.server 8000 --directory tools/webui
```

Open `http://localhost:8000` in a Web Bluetooth-capable Chromium browser and press **Connect**. `localhost` is an allowed secure development context for Web Bluetooth; normal remote hosting must use HTTPS.

## BLE contract

```text
Device name: TriWhirl
Service:     54f10000-8f4d-4f3a-b691-54524957484c
RX write:    54f10001-8f4d-4f3a-b691-54524957484c
TX notify:   54f10002-8f4d-4f3a-b691-54524957484c
```

The browser and wired UART use the same newline-delimited command protocol and telemetry schema. The UI contains no controller logic; commands remain subject to firmware runtime checks.
