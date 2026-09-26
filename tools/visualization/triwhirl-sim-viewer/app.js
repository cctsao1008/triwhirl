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

  let eventSource = null;
  let latest = null;
  let samples = [];
  let meta = null;
  let runComplete = false;

  const number = (value, digits = 3) => {
    const n = Number(value);
    return Number.isFinite(n) ? n.toFixed(digits) : "—";
  };
  const yesNo = (value) => value ? "YES" : "NO";

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
    // Exact three-arc construction from an equilateral triangle with side 190.
    // Local body coordinates put the nominal contact vertex at (0, 0).
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

    // Contact frame / upright reference. Positive simulation angle is rendered
    // positive in canvas coordinates without any visual-only sign reinterpretation.
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
    const xOf = (t) => left + ((t - startT) / Math.max(1e-9, endT - startT || 10)) * plotW;
    const yErr = (v) => top + plotH / 2 - (v / 5) * (plotH / 2);
    const yVq = (v) => top + plotH / 2 - (v / 4) * (plotH / 2);

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

  function setGate(status, summary = "") {
    const chip = $("gate-chip");
    chip.classList.remove("gate-pass", "gate-blocked", "gate-fail");
    if (status === "PASS") {
      chip.textContent = "SIM GATE: PASS";
      chip.classList.add("gate-pass");
      $("gate-result").textContent = "PASS";
    } else if (status === "FAIL") {
      chip.textContent = "SIM GATE: FAIL";
      chip.classList.add("gate-fail");
      $("gate-result").textContent = "FAIL";
    } else {
      chip.textContent = "SIM GATE: BLOCKED";
      chip.classList.add("gate-blocked");
      $("gate-result").textContent = status === "RUNNING" ? "RUNNING" : "—";
    }
    $("gate-summary").textContent = summary || "Run the simulation gate.";
  }

  function stopRun() {
    if (eventSource) {
      eventSource.close();
      eventSource = null;
    }
    runButton.disabled = false;
    stopButton.disabled = true;
    $("stream-chip").textContent = "SIM: disconnected";
  }

  function startRun() {
    stopRun();
    samples = [];
    latest = null;
    meta = null;
    runComplete = false;
    drawScene();
    drawHistory();
    runButton.disabled = true;
    stopButton.disabled = false;
    $("stream-chip").textContent = "SIM: preparing native run";
    setGate("RUNNING", "Native C++ SITL is generating fresh 10 s evidence.");

    const params = new URLSearchParams({
      profile: profile.value,
      speed: speed.value,
      fps: "60",
    });
    eventSource = new EventSource(`/api/live?${params.toString()}`);

    eventSource.addEventListener("meta", (event) => {
      meta = JSON.parse(event.data);
      $("model-chip").textContent = `model: ${meta.profile}`;
      $("stream-chip").textContent = "SIM: replaying fresh native evidence";
      setGate("RUNNING", meta.scope || "Fresh native simulation evidence.");
    });

    eventSource.addEventListener("sample", (event) => {
      const sample = JSON.parse(event.data);
      samples.push(sample);
      if (samples.length > 4000) samples.splice(0, samples.length - 4000);
      updateTelemetry(sample);
    });

    eventSource.addEventListener("end", (event) => {
      const result = JSON.parse(event.data);
      runComplete = true;
      setGate(result.gate_pass ? "PASS" : "FAIL", result.summary || "Simulation finished.");
      $("stream-chip").textContent = "SIM: complete";
      if (eventSource) eventSource.close();
      eventSource = null;
      runButton.disabled = false;
      stopButton.disabled = true;
    });

    eventSource.addEventListener("stream-error", (event) => {
      const error = JSON.parse(event.data);
      setGate("FAIL", error.message || "Simulation backend failed.");
      stopRun();
    });

    eventSource.onerror = () => {
      if (!runComplete && eventSource) {
        setGate("FAIL", "SSE connection closed before the native run completed.");
      }
      stopRun();
    };
  }

  runButton.addEventListener("click", startRun);
  stopButton.addEventListener("click", () => {
    setGate("BLOCKED", "Stopped before completing the 10 s simulation gate.");
    stopRun();
  });
  window.addEventListener("resize", () => {
    drawScene();
    drawHistory();
  });

  drawScene();
  drawHistory();
})();
