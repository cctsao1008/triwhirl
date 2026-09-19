const SERVICE_UUID = "54f10000-8f4d-4f3a-b691-54524957484c";
const RX_UUID = "54f10001-8f4d-4f3a-b691-54524957484c";
const TX_UUID = "54f10002-8f4d-4f3a-b691-54524957484c";

const TELEMETRY_FIELDS = [
  "t_us", "mode", "vq_v", "e_angle_rad", "e_hz", "status_ok",
  "sample_ok", "mag", "raw", "unwrapped_count", "angle_rad",
  "unwrapped_rad", "vel_rad_s", "vel_inst_rad_s", "vel_valid",
  "read_errors", "imu_ok", "ax", "ay", "az", "gx", "gy", "gz",
  "imu_read_errors", "attitude_ok", "theta_rad", "theta_rate_rad_s",
  "accel_weight", "loop_exec_us", "loop_max_exec_us", "loop_overruns",
];

const ui = {
  state: document.querySelector("#transportState"),
  connect: document.querySelector("#connectButton"),
  disconnect: document.querySelector("#disconnectButton"),
  commandForm: document.querySelector("#commandForm"),
  commandInput: document.querySelector("#commandInput"),
  send: document.querySelector("#sendButton"),
  console: document.querySelector("#console"),
  mode: document.querySelector("#mode"),
  theta: document.querySelector("#theta"),
  thetaRate: document.querySelector("#thetaRate"),
  wheelRate: document.querySelector("#wheelRate"),
  vq: document.querySelector("#vq"),
  loopMax: document.querySelector("#loopMax"),
  record: document.querySelector("#recordButton"),
  save: document.querySelector("#saveButton"),
  clear: document.querySelector("#clearButton"),
};

const decoder = new TextDecoder();
const encoder = new TextEncoder();
let device = null;
let rxCharacteristic = null;
let txCharacteristic = null;
let rxTextBuffer = "";
let recording = false;
let recordedRows = [];

function setConnected(connected) {
  ui.state.textContent = connected ? "connected" : "disconnected";
  ui.state.classList.toggle("connected", connected);
  ui.connect.disabled = connected;
  ui.disconnect.disabled = !connected;
  ui.commandInput.disabled = !connected;
  ui.send.disabled = !connected;
  ui.record.disabled = !connected;
  for (const button of document.querySelectorAll("[data-command]")) {
    button.disabled = !connected;
  }
}

function logLine(line) {
  ui.console.textContent += `${line}\n`;
  const lines = ui.console.textContent.split("\n");
  if (lines.length > 250) {
    ui.console.textContent = lines.slice(-250).join("\n");
  }
  ui.console.scrollTop = ui.console.scrollHeight;
}

function formatNumber(value, digits = 4) {
  const number = Number(value);
  return Number.isFinite(number) ? number.toFixed(digits) : "—";
}

function consumeTelemetry(line) {
  const values = line.split(",").slice(1);
  if (values.length !== TELEMETRY_FIELDS.length) {
    logLine(`protocol error: telemetry field count ${values.length}`);
    return;
  }

  const frame = Object.fromEntries(
    TELEMETRY_FIELDS.map((field, index) => [field, values[index]])
  );

  ui.mode.textContent = frame.mode;
  ui.theta.textContent = formatNumber(frame.theta_rad);
  ui.thetaRate.textContent = formatNumber(frame.theta_rate_rad_s);
  ui.wheelRate.textContent = formatNumber(frame.vel_rad_s);
  ui.vq.textContent = formatNumber(frame.vq_v, 3);
  ui.loopMax.textContent = Number.isFinite(Number(frame.loop_max_exec_us))
    ? String(Number(frame.loop_max_exec_us))
    : "—";

  if (recording) {
    recordedRows.push(values);
    ui.save.disabled = recordedRows.length === 0;
  }
}

function consumeLine(line) {
  if (!line) return;
  if (line.startsWith("telemetry,")) {
    consumeTelemetry(line);
    return;
  }
  if (!line.startsWith("telemetry_fields,")) {
    logLine(line);
  }
}

function onNotification(event) {
  rxTextBuffer += decoder.decode(event.target.value, { stream: true });
  while (true) {
    const newline = rxTextBuffer.indexOf("\n");
    if (newline < 0) break;
    const line = rxTextBuffer.slice(0, newline).replace(/\r$/, "");
    rxTextBuffer = rxTextBuffer.slice(newline + 1);
    consumeLine(line);
  }
}

async function sendCommand(command) {
  if (!rxCharacteristic) throw new Error("not connected");
  const data = encoder.encode(`${command.trim()}\n`);
  if (typeof rxCharacteristic.writeValueWithoutResponse === "function") {
    await rxCharacteristic.writeValueWithoutResponse(data);
  } else {
    await rxCharacteristic.writeValue(data);
  }
}

async function connect() {
  if (!navigator.bluetooth) {
    throw new Error("Web Bluetooth is not available in this browser");
  }

  device = await navigator.bluetooth.requestDevice({
    filters: [{ services: [SERVICE_UUID] }],
  });
  device.addEventListener("gattserverdisconnected", onDisconnected);

  const server = await device.gatt.connect();
  const service = await server.getPrimaryService(SERVICE_UUID);
  rxCharacteristic = await service.getCharacteristic(RX_UUID);
  txCharacteristic = await service.getCharacteristic(TX_UUID);
  await txCharacteristic.startNotifications();
  txCharacteristic.addEventListener("characteristicvaluechanged", onNotification);

  setConnected(true);
  logLine(`connected: ${device.name || "TriWhirl"}`);
  await sendCommand("telemetry on");
}

function onDisconnected() {
  rxCharacteristic = null;
  txCharacteristic = null;
  rxTextBuffer = "";
  setConnected(false);
  logLine("disconnected");
}

async function disconnect() {
  if (device?.gatt?.connected) {
    try {
      await sendCommand("telemetry off");
    } catch (_) {
      // The connection may already be closing.
    }
    device.gatt.disconnect();
  }
  onDisconnected();
}

function csvCell(value) {
  const text = String(value ?? "");
  return /[",\n]/.test(text) ? `"${text.replaceAll('"', '""')}"` : text;
}

function saveCsv() {
  if (recordedRows.length === 0) return;
  const header = ["schema_version", ...TELEMETRY_FIELDS];
  const rows = [
    header,
    ...recordedRows.map((row) => ["1", ...row]),
  ];
  const csv = rows.map((row) => row.map(csvCell).join(",")).join("\r\n") + "\r\n";
  const blob = new Blob([csv], { type: "text/csv;charset=utf-8" });
  const url = URL.createObjectURL(blob);
  const anchor = document.createElement("a");
  anchor.href = url;
  anchor.download = `triwhirl-${new Date().toISOString().replaceAll(":", "-")}.csv`;
  anchor.click();
  URL.revokeObjectURL(url);
}

ui.connect.addEventListener("click", async () => {
  try {
    await connect();
  } catch (error) {
    logLine(`connect error: ${error.message || error}`);
    setConnected(false);
  }
});

ui.disconnect.addEventListener("click", () => disconnect());

ui.commandForm.addEventListener("submit", async (event) => {
  event.preventDefault();
  const command = ui.commandInput.value.trim();
  if (!command) return;
  try {
    await sendCommand(command);
    logLine(`> ${command}`);
    ui.commandInput.value = "";
  } catch (error) {
    logLine(`send error: ${error.message || error}`);
  }
});

for (const button of document.querySelectorAll("[data-command]")) {
  button.addEventListener("click", async () => {
    const command = button.dataset.command;
    try {
      await sendCommand(command);
      logLine(`> ${command}`);
    } catch (error) {
      logLine(`send error: ${error.message || error}`);
    }
  });
}

ui.record.addEventListener("click", () => {
  recording = !recording;
  if (recording) recordedRows = [];
  ui.record.textContent = recording ? "Stop recording" : "Start recording";
  ui.save.disabled = recording || recordedRows.length === 0;
  logLine(recording ? "recording started" : `recording stopped: ${recordedRows.length} rows`);
});

ui.save.addEventListener("click", saveCsv);
ui.clear.addEventListener("click", () => { ui.console.textContent = ""; });

setConnected(false);
