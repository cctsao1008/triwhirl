(() => {
  const $ = (id) => document.getElementById(id);
  const scene = $("scene");
  const sceneCtx = scene.getContext("2d");
  const historyCanvas = $("history");
  const historyCtx = historyCanvas.getContext("2d");
  const profile = $("profile");
  const speed = $("speed");
  const runButton = $("run");
  const stopButton = $("stop");
  const bodyLeft = $("body-left");
  const bodyRight = $("body-right");
  const bodyStrength = $("body-strength");
  const wheelMinus = $("wheel-minus");
  const wheelPlus = $("wheel-plus");
  const wheelStrength = $("wheel-strength");

  let eventSource = null;
  let latest = null;
  let samples = [];
  let meta = null;
  let runComplete = false;
  let disturbances = [];
  let kickFlash = null;

  const number = (value, digits = 3) => {
    const n = Number(value);
    return Number.isFinite(n) ? n.toFixed(digits) : "—";
  };
  const yesNo = (value) => value ? "YES" : "NO";
  const gateName = () => disturbances.length ? "DISTURBANCE" : "BALANCE";

  function fitCanvas(canvas, ctx) {
    const ratio = Math.max(1, window.devicePixelRatio || 1);
    const rect = canvas.getBoundingClientRect();
    const width = Math.max(1, Math.round(rect.width * ratio));
    const height = Math.max(1, Math.round(rect.height * ratio));
    if (canvas.width !== width || canvas.height !== height) {
      canvas.width = width;
      canvas.height = height;
    }
    ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
    return { width: rect.width, height: rect.height };
  }

  function drawReuleaux(ctx) {
    const path = new Path2D(
      "M 0 0 " +
      "A 190 190 0 0 1 -95 -164.5448 " +
      "A 190 190 0 0 1 95 -164.5448 " +
      "A 190 190 0 0 1 0 0 Z"
    );
    ctx.save();
    ctx.fillStyle = "rgba(66, 159, 230, 0.24)";
    ctx.strokeStyle = "#73c4ff";
    ctx.lineWidth = 3;
    ctx.fill(path);
    ctx.stroke(path);
    ctx.restore();
  }

  function drawWheel(ctx, wheelAngle) {
    const radius = 46;
    ctx.save();
    ctx.translate(0, -103);
    ctx.strokeStyle = "#dce8f6";
    ctx.fillStyle = "rgba(220,232,246,0.08)";
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.arc(0, 0, radius, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();

    ctx.rotate(wheelAngle);
    ctx.strokeStyle = "#ffcc66";
    ctx.lineWidth = 2.5;
    for (let i = 0; i < 6; ++i) {
      const angle = i * Math.PI / 3;
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.lineTo(Math.cos(angle) * (radius - 5), Math.sin(angle) * (radius - 5));
      ctx.stroke();
    }
    ctx.fillStyle = "#ffcc66";
    ctx.beginPath();
    ctx.arc(0, 0, 5, 0, Math.PI * 2);
    ctx.fill();
    ctx.restore();
  }

  function drawKickArrow(ctx, scale) {
    if (!kickFlash || performance.now() > kickFlash.until) return;
    const positive = kickFlash.delta > 0;
    const bodyKick = kickFlash.kind === "body";
    ctx.save();
    ctx.scale(1 / scale, 1 / scale);
    ctx.strokeStyle = bodyKick ? "#ff6b75" : "#d58cff";
    ctx.fillStyle = ctx.strokeStyle;
    ctx.lineWidth = 4;
    const y = bodyKick ? -118 * scale : -42 * scale;
    const x0 = positive ? -170 * scale : 170 * scale;
    const x1 = positive ? -85 * scale : 85 * scale;
    ctx.beginPath();
    ctx.moveTo(x0, y);
    ctx.lineTo(x1, y);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(x1, y);
    ctx.lineTo(x1 + (positive ? -14 : 14), y - 9);
    ctx.lineTo(x1 + (positive ? -14 : 14), y + 9);
    ctx.closePath();
    ctx.fill();
    ctx.restore();
  }

  function drawScene() {
    const { width, height } = fitCanvas(scene, sceneCtx);
    sceneCtx.clearRect(0, 0, width, height);

    const groundY = height - 58;
    sceneCtx.strokeStyle = "#52657c";
    sceneCtx.lineWidth = 2;
    sceneCtx.beginPath();
    sceneCtx.moveTo(24, groundY);
    sceneCtx.lineTo(width - 24, groundY);
    sceneCtx.stroke();

    sceneCtx.strokeStyle = "rgba(82,101,124,0.35)";
    sceneCtx.lineWidth = 1;
    for (let x = 30; x < width - 20; x += 32) {
      sceneCtx.beginPath();
      sceneCtx.moveTo(x, groundY + 4);
      sceneCtx.lineTo(x - 12, groundY + 16);
      sceneCtx.stroke();
    }

    const error = latest ? Number(latest.true_error_rad) : 0;
    const wheelAngle = latest ? Number(latest.true_wheel_angle_rad) : 0;
    const scale = Math.min(1.35, Math.max(0.72, width / 850));

    sceneCtx.save();
    sceneCtx.translate(width * 0.50, groundY);
    sceneCtx.scale(scale, scale);
    sceneCtx.strokeStyle = "rgba(101,184,255,0.30)";
    sceneCtx.lineWidth = 1.5;
    sceneCtx.setLineDash([6, 6]);
    sceneCtx.beginPath();
    sceneCtx.moveTo(0, 8);
    sceneCtx.lineTo(0, -230);
    sceneCtx.stroke();
    sceneCtx.setLineDash([]);

    sceneCtx.rotate(error);
    drawReuleaux(sceneCtx);
    drawWheel(sceneCtx, wheelAngle);
    drawKickArrow(sceneCtx, scale);

    sceneCtx.fillStyle = "#65b8ff";
    sceneCtx.beginPath();
    sceneCtx.arc(0, 0, 5, 0, Math.PI * 2);
    sceneCtx.fill();
    sceneCtx.restore();

    sceneCtx.fillStyle = "#8fa3bc";
    sceneCtx.font = "12px ui-monospace, SFMono-Regular, Consolas, monospace";
    sceneCtx.fillText("local upright contact frame", 22, groundY - 10);
    if (!latest) {
      sceneCtx.fillStyle = "#70839c";
      sceneCtx.font = "15px system-ui, sans-serif";
      sceneCtx.textAlign = "center";
      sceneCtx.fillText("Run the 10 s native SITL gate", width / 2, 45);
      sceneCtx.textAlign = "left";
    }
  }

  function drawHistory() {
    const { width, height } = fitCanvas(historyCanvas, historyCtx);
    historyCtx.clearRect(0, 0, width, height);
    historyCtx.fillStyle = "#0d1520";
    historyCtx.fillRect(0, 0, width, height);

    const left = 42, right = 12, top = 12, bottom = 24;
    const plotW = Math.max(10, width - left - right);
    const plotH = Math.max(10, height - top - bottom);
    historyCtx.strokeStyle = "#2b394d";
    historyCtx.lineWidth = 1;
    for (let i = 0; i <= 4; ++i) {
      const y = top + (plotH * i / 4);
      historyCtx.beginPath();
      historyCtx.moveTo(left, y);
      historyCtx.lineTo(left + plotW, y);
      historyCtx.stroke();
    }

    historyCtx.font = "11px ui-monospace, Consolas, monospace";
    historyCtx.fillStyle = "#8fa3bc";
    historyCtx.fillText("+5°", 6, top + 4);
    historyCtx.fillText("0", 20, top + plotH / 2 + 4);
    historyCtx.fillText("−5°", 6, top + plotH + 4);

    if (samples.length < 2) return;
    const endT = Number(samples[samples.length - 1].t_s);
    const startT = Math.max(0, endT - 10);
    const visible = samples.filter((s) => Number(s.t_s) >= startT);
    const span = Math.max(1e-9, endT - startT || 10);
    const xOf = (t) => left + ((t - startT) / span) * plotW;
    const yErr = (v) => top + plotH / 2 - (v / 5) * (plotH / 2);
    const yVq = (v) => top + plotH / 2 - (v / 4) * (plotH / 2);

    disturbances.forEach((event) => {
      if (event.t_s < startT || event.t_s > endT) return;
      const x = xOf(event.t_s);
      historyCtx.strokeStyle = event.kind === "body" ? "#ff6b75" : "#d58cff";
      historyCtx.lineWidth = 1.5;
      historyCtx.setLineDash([4, 3]);
      historyCtx.beginPath();
      historyCtx.moveTo(x, top);
      historyCtx.lineTo(x, top + plotH);
      historyCtx.stroke();
      historyCtx.setLineDash([]);
      historyCtx.fillStyle = historyCtx.strokeStyle;
      historyCtx.fillText(event.kind === "body" ? "B" : "W", x + 3, top + 11);
    });

    const trace = (getter, yMap, color) => {
      historyCtx.strokeStyle = color;
      historyCtx.lineWidth = 2;
      historyCtx.beginPath();
      visible.forEach((sample, index) => {
        const x = xOf(Number(sample.t_s));
        const y = yMap(getter(sample));
        if (index === 0) historyCtx.moveTo(x, y); else historyCtx.lineTo(x, y);
      });
      historyCtx.stroke();
    };
    trace((s) => Number(s.true_error_deg), yErr, "#65b8ff");
    trace((s) => Number(s.vq_applied_v), yVq, "#ffcc66");

    historyCtx.fillStyle = "#65b8ff";
    historyCtx.fillText("θ error", left + 6, height - 6);
    historyCtx.fillStyle = "#ffcc66";
    historyCtx.fillText("Vq", left + 72, height - 6);
  }

  function updateTelemetry(sample) {
    latest = sample;
    $("time-readout").textContent = `t = ${number(sample.t_s, 3)} s`;
    $("phase-readout").textContent = `phase: ${sample.phase}`;
    $("theta").textContent = `${number(sample.true_error_deg, 3)}°`;
    $("theta-dot").textContent = `${number(sample.true_theta_rate_rad_s, 3)} rad/s`;
    $("wheel-rate").textContent = `${number(sample.true_wheel_rate_rad_s, 3)} rad/s`;
    $("wheel-angle").textContent = `${number(sample.true_wheel_angle_rad, 3)} rad`;
    $("phase").textContent = sample.phase;
    $("settling").textContent = yesNo(sample.settling);
    $("stable").textContent = yesNo(sample.stable);
    $("filtered-rate").textContent = `${number(sample.filtered_rate_rad_s, 3)} rad/s`;
    $("target").textContent = `${number(sample.target_velocity_rad_s, 3)} rad/s`;
    $("velocity-error").textContent = `${number(sample.velocity_error_rad_s, 3)} rad/s`;
    $("integral").textContent = `${number(sample.velocity_integral_v, 3)} V`;
    $("vq-unclamped").textContent = `${number(sample.vq_unclamped_v, 3)} V`;
    $("vq").textContent = `${number(sample.vq_applied_v, 3)} V`;
    drawScene();
    drawHistory();
  }

  function updateDisturbanceStatus() {
    $("disturbance-count").textContent = String(disturbances.length);
    $("gate-type").textContent = gateName();
    if (!disturbances.length) {
      $("disturbance-readout").textContent = "No disturbances";
      return;
    }
    const last = disturbances[disturbances.length - 1];
    const label = last.kind === "body" ? "body Δθ̇" : "wheel Δω";
    $("disturbance-readout").textContent =
      `${disturbances.length} injected · last ${label} ${last.delta_rad_s >= 0 ? "+" : ""}${number(last.delta_rad_s, 2)} rad/s @ ${number(last.t_s, 3)} s`;
  }

  function setGate(status, summary = "") {
    const chip = $("gate-chip");
    chip.classList.remove("gate-pass", "gate-blocked", "gate-fail");
    const prefix = disturbances.length ? "SIM DISTURBANCE" : "SIM GATE";
    if (status === "PASS") {
      chip.textContent = `${prefix}: PASS`;
      chip.classList.add("gate-pass");
      $("gate-result").textContent = "PASS";
    } else if (status === "FAIL") {
      chip.textContent = `${prefix}: FAIL`;
      chip.classList.add("gate-fail");
      $("gate-result").textContent = "FAIL";
    } else {
      chip.textContent = `${prefix}: BLOCKED`;
      chip.classList.add("gate-blocked");
      $("gate-result").textContent = status === "RUNNING" ? "RUNNING" : "—";
    }
    $("gate-summary").textContent = summary || "Run the simulation gate.";
    updateDisturbanceStatus();
  }

  function setKickControls(enabled) {
    bodyLeft.disabled = !enabled;
    bodyRight.disabled = !enabled;
    wheelMinus.disabled = !enabled;
    wheelPlus.disabled = !enabled;
  }

  function closeStream() {
    if (eventSource) {
      eventSource.close();
      eventSource = null;
    }
  }

  function stopRun() {
    closeStream();
    runButton.disabled = false;
    stopButton.disabled = true;
    profile.disabled = false;
    setKickControls(false);
    $("stream-chip").textContent = "SIM: disconnected";
  }

  function disturbanceParams(params) {
    disturbances.forEach((event) => {
      const key = event.kind === "body" ? "body_kick" : "wheel_kick";
      params.append(key, `${event.t_s.toFixed(6)}:${event.delta_rad_s.toFixed(6)}`);
    });
  }

  function launchStream(resumeFrom = 0) {
    closeStream();
    runComplete = false;
    runButton.disabled = true;
    stopButton.disabled = false;
    profile.disabled = true;
    setKickControls(false);
    $("stream-chip").textContent = disturbances.length
      ? "SIM: recomputing disturbed native trajectory"
      : "SIM: preparing native run";

    const params = new URLSearchParams({
      profile: profile.value,
      speed: speed.value,
      fps: "60",
      resume_from: String(resumeFrom),
    });
    disturbanceParams(params);

    const source = new EventSource(`/api/live?${params.toString()}`);
    eventSource = source;

    source.addEventListener("meta", (event) => {
      if (eventSource !== source) return;
      meta = JSON.parse(event.data);
      $("model-chip").textContent = `model: ${meta.profile}`;
      $("stream-chip").textContent = disturbances.length
        ? "SIM: disturbed native evidence"
        : "SIM: fresh native evidence";
      $("gate-type").textContent = meta.gate || gateName();
      setKickControls(true);
      setGate("RUNNING", meta.scope || "Fresh native simulation evidence.");
    });

    source.addEventListener("sample", (event) => {
      if (eventSource !== source) return;
      const sample = JSON.parse(event.data);
      samples.push(sample);
      if (samples.length > 4000) samples.splice(0, samples.length - 4000);
      updateTelemetry(sample);
    });

    source.addEventListener("end", (event) => {
      if (eventSource !== source) return;
      const result = JSON.parse(event.data);
      runComplete = true;
      setGate(result.gate_pass ? "PASS" : "FAIL", result.summary || "Simulation finished.");
      $("stream-chip").textContent = "SIM: complete";
      closeStream();
      runButton.disabled = false;
      stopButton.disabled = true;
      profile.disabled = false;
      setKickControls(false);
    });

    source.addEventListener("stream-error", (event) => {
      if (eventSource !== source) return;
      const error = JSON.parse(event.data);
      setGate("FAIL", error.message || "Simulation backend failed.");
      stopRun();
    });

    source.onerror = () => {
      if (eventSource !== source) return;
      if (!runComplete) {
        setGate("FAIL", "SSE connection closed before the native run completed.");
      }
      stopRun();
    };
  }

  function startRun() {
    closeStream();
    samples = [];
    latest = null;
    meta = null;
    disturbances = [];
    kickFlash = null;
    $("kick-readout").textContent = "";
    updateDisturbanceStatus();
    drawScene();
    drawHistory();
    setGate("RUNNING", "Native C++ SITL is generating fresh 10 s evidence.");
    launchStream(0);
  }

  function injectDisturbance(kind, delta) {
    if (!eventSource || !latest) return;
    const currentT = Number(latest.t_s);
    if (!Number.isFinite(currentT)) return;
    const t = Math.max(0, Math.min(10, Math.round(currentT * 1000) / 1000));
    const event = { kind, t_s: t, delta_rad_s: Number(delta) };
    disturbances.push(event);
    disturbances.sort((a, b) => a.t_s - b.t_s);

    // Remove the stale branch from the click time onward. The new SSE stream is
    // a native recomputation from t=0 using every accumulated disturbance, but
    // only resumes display at this point.
    samples = samples.filter((sample) => Number(sample.t_s) < t - 1e-9);
    latest = samples.length ? samples[samples.length - 1] : latest;
    kickFlash = { ...event, until: performance.now() + 650 };
    const label = kind === "body" ? "BODY PUSH" : "WHEEL KICK";
    $("kick-readout").textContent =
      `${label} ${delta >= 0 ? "+" : ""}${number(delta, 2)} rad/s @ ${number(t, 3)} s`;
    updateDisturbanceStatus();
    setGate("RUNNING", "Disturbance accepted; recomputing the native closed-loop trajectory from the same history.");
    drawScene();
    drawHistory();
    launchStream(t);
  }

  runButton.addEventListener("click", startRun);
  stopButton.addEventListener("click", () => {
    setGate("BLOCKED", "Stopped before completing the 10 s simulation gate.");
    stopRun();
  });
  bodyLeft.addEventListener("click", () => injectDisturbance("body", -Number(bodyStrength.value)));
  bodyRight.addEventListener("click", () => injectDisturbance("body", Number(bodyStrength.value)));
  wheelMinus.addEventListener("click", () => injectDisturbance("wheel", -Number(wheelStrength.value)));
  wheelPlus.addEventListener("click", () => injectDisturbance("wheel", Number(wheelStrength.value)));
  window.addEventListener("resize", () => {
    drawScene();
    drawHistory();
  });

  updateDisturbanceStatus();
  drawScene();
  drawHistory();
})();
