(() => {
  const $ = (id) => document.getElementById(id);
  const scene = $("scene");
  const sceneCtx = scene.getContext("2d");
  const historyCanvas = $("history");
  const historyCtx = historyCanvas.getContext("2d");
  const scenario = $("scenario");
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
  let sessionId = null;
  let latest = null;
  let samples = [];
  let disturbances = [];
  let kickFlash = null;
  let stopping = false;
  let currentScenario = scenario.value;

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
    const path = new Path2D(
      "M 0 0 " +
      "A 190 190 0 0 1 -95 -164.5448 " +
      "A 190 190 0 0 1 95 -164.5448 " +
      "A 190 190 0 0 1 0 0 Z"
    );
    ctx.save();
    ctx.fillStyle = "rgba(66,159,230,0.24)";
    ctx.strokeStyle = "#73c4ff";
    ctx.lineWidth = 3;
    ctx.fill(path);
    ctx.stroke(path);
    ctx.restore();
  }

  function buildReuleauxBoundary() {
    const r = 190;
    const h = 164.5448;
    const arcs = [
      { cx: 95, cy: -h, a0: 2 * Math.PI / 3, a1: Math.PI },
      { cx: 0, cy: 0, a0: -2 * Math.PI / 3, a1: -Math.PI / 3 },
      { cx: -95, cy: -h, a0: 0, a1: Math.PI / 3 },
    ];
    const points = [];
    arcs.forEach((arc) => {
      for (let i = 0; i <= 24; ++i) {
        const u = i / 24;
        const a = arc.a0 + (arc.a1 - arc.a0) * u;
        points.push({ x: arc.cx + r * Math.cos(a), y: arc.cy + r * Math.sin(a) });
      }
    });
    return points;
  }

  const reuleauxBoundary = buildReuleauxBoundary();

  function rotatedLowestY(angle) {
    const s = Math.sin(angle);
    const c = Math.cos(angle);
    let maxY = -Infinity;
    reuleauxBoundary.forEach((p) => {
      const y = p.x * s + p.y * c;
      if (y > maxY) maxY = y;
    });
    return Number.isFinite(maxY) ? maxY : 0;
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

    const bodyAngle = latest
      ? Number.isFinite(Number(latest.true_body_angle_rad))
        ? Number(latest.true_body_angle_rad)
        : Number(latest.true_error_rad)
      : 0;
    const wheelAngle = latest ? Number(latest.true_wheel_angle_rad) : 0;
    const scale = Math.min(1.35, Math.max(0.72, width / 850));
    const supportOffset = rotatedLowestY(bodyAngle);
    sceneCtx.save();
    sceneCtx.translate(width * 0.50, groundY - supportOffset * scale);
    sceneCtx.scale(scale, scale);
    sceneCtx.strokeStyle = "rgba(101,184,255,0.30)";
    sceneCtx.lineWidth = 1.5;
    sceneCtx.setLineDash([6, 6]);
    sceneCtx.beginPath();
    sceneCtx.moveTo(0, 40);
    sceneCtx.lineTo(0, -230);
    sceneCtx.stroke();
    sceneCtx.setLineDash([]);
    sceneCtx.rotate(bodyAngle);
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
    sceneCtx.fillText("display contact support follows body orientation", 22, groundY - 10);
    if (!latest) {
      sceneCtx.fillStyle = "#70839c";
      sceneCtx.font = "15px system-ui, sans-serif";
      sceneCtx.textAlign = "center";
      sceneCtx.fillText("Run native SITL — full standup starts from rest and continues until Stop", width / 2, 45);
      sceneCtx.textAlign = "left";
    }
  }

  function drawHistory() {
    const { width, height } = fitCanvas(historyCanvas, historyCtx);
    historyCtx.clearRect(0, 0, width, height);
    historyCtx.fillStyle = "#0d1520";
    historyCtx.fillRect(0, 0, width, height);
    const left = 46, right = 12, top = 12, bottom = 24;
    const plotW = Math.max(10, width - left - right);
    const plotH = Math.max(10, height - top - bottom);
    historyCtx.strokeStyle = "#2b394d";
    historyCtx.lineWidth = 1;
    for (let i = 0; i <= 4; ++i) {
      const y = top + plotH * i / 4;
      historyCtx.beginPath();
      historyCtx.moveTo(left, y);
      historyCtx.lineTo(left + plotW, y);
      historyCtx.stroke();
    }
    const errorRange = currentScenario === "full-standup" ? 60 : 5;
    historyCtx.font = "11px ui-monospace, Consolas, monospace";
    historyCtx.fillStyle = "#8fa3bc";
    historyCtx.fillText(`+${errorRange}°`, 4, top + 4);
    historyCtx.fillText("0", 24, top + plotH / 2 + 4);
    historyCtx.fillText(`−${errorRange}°`, 4, top + plotH + 4);
    if (samples.length < 2) return;

    const endT = Number(samples[samples.length - 1].t_s);
    const startT = Math.max(0, endT - 10);
    const visible = samples.filter((s) => Number(s.t_s) >= startT);
    const span = Math.max(1e-9, endT - startT || 10);
    const xOf = (t) => left + ((t - startT) / span) * plotW;
    const yErr = (v) => top + plotH / 2 - (v / errorRange) * (plotH / 2);
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

  function setLiveStatus(status, summary = "") {
    const chip = $("gate-chip");
    chip.classList.remove("gate-pass", "gate-blocked", "gate-fail");
    if (status === "STABLE") {
      chip.textContent = "LIVE SIM: STABLE";
      chip.classList.add("gate-pass");
    } else if (status === "FAIL") {
      chip.textContent = "LIVE SIM: OUT OF BALANCE";
      chip.classList.add("gate-fail");
    } else if (status === "SWINGING") {
      chip.textContent = "LIVE SIM: SWING-UP";
      chip.classList.add("gate-blocked");
    } else if (status === "CAPTURE") {
      chip.textContent = "LIVE SIM: CAPTURE / SETTLING";
      chip.classList.add("gate-blocked");
    } else {
      chip.textContent = status === "STOPPED" ? "LIVE SIM: STOPPED" : "LIVE SIM: RUNNING";
      chip.classList.add("gate-blocked");
    }
    $("gate-result").textContent = status;
    $("gate-summary").textContent = summary || "Continuous native simulation.";
  }

  function updateTelemetry(sample) {
    latest = sample;
    $("time-readout").textContent = `t = ${number(sample.t_s, 3)} s`;
    $("phase-readout").textContent = `phase: ${sample.phase}`;
    $("theta").textContent = `${number(sample.true_error_deg, 3)}°`;
    $("body-angle").textContent = `${number(sample.true_body_angle_deg, 3)}°`;
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

    if (currentScenario === "full-standup") {
      if (sample.phase === "swing_high" || sample.phase === "swing_low") {
        setLiveStatus("SWINGING", `Energy pumping from resting face · t=${number(sample.t_s, 1)} s`);
      } else if (sample.stable) {
        setLiveStatus("STABLE", `Full standup complete; balancing continuously · t=${number(sample.t_s, 1)} s`);
      } else if (sample.phase === "balance") {
        setLiveStatus("CAPTURE", `${sample.settling ? "Settling after capture" : "First balance approach"} · t=${number(sample.t_s, 1)} s`);
      }
    } else if (sample.phase !== "balance") {
      setLiveStatus("FAIL", `Left Balance at t=${number(sample.t_s, 3)} s; simulation continues until Stop.`);
    } else if (sample.stable) {
      setLiveStatus("STABLE", `Balancing continuously · t=${number(sample.t_s, 1)} s`);
    } else if (sample.settling) {
      setLiveStatus("CAPTURE", `Settling/recovering · t=${number(sample.t_s, 1)} s`);
    }
    drawScene();
    drawHistory();
  }

  function updateDisturbanceStatus() {
    $("disturbance-count").textContent = String(disturbances.length);
    if (!disturbances.length) {
      $("disturbance-readout").textContent = "No disturbances";
      return;
    }
    const last = disturbances[disturbances.length - 1];
    const label = last.kind === "body" ? "body Δθ̇" : "wheel Δω";
    $("disturbance-readout").textContent =
      `${disturbances.length} injected · last ${label} ${last.delta_rad_s >= 0 ? "+" : ""}${number(last.delta_rad_s, 2)} rad/s @ ${number(last.t_s, 3)} s`;
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

  function resetControls() {
    runButton.disabled = false;
    stopButton.disabled = true;
    scenario.disabled = false;
    profile.disabled = false;
    speed.disabled = false;
    setKickControls(false);
  }

  async function postJson(path, payload) {
    const response = await fetch(path, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
    });
    const result = await response.json();
    if (!response.ok || !result.ok) {
      throw new Error(result.error || `${path} failed (${response.status})`);
    }
    return result;
  }

  function startRun() {
    closeStream();
    sessionId = null;
    latest = null;
    samples = [];
    disturbances = [];
    kickFlash = null;
    stopping = false;
    currentScenario = scenario.value;
    updateDisturbanceStatus();
    drawScene();
    drawHistory();
    runButton.disabled = true;
    stopButton.disabled = false;
    scenario.disabled = true;
    profile.disabled = true;
    speed.disabled = true;
    setKickControls(false);
    $("stream-chip").textContent = "SIM: starting continuous native run";
    $("scenario-readout").textContent = currentScenario === "full-standup" ? "Full standup from rest" : "Upright balance only";
    setLiveStatus("RUNNING", "Native fixed-step simulation continues until Stop.");

    const params = new URLSearchParams({
      scenario: currentScenario,
      profile: profile.value,
      speed: speed.value,
      fps: "60",
    });
    const source = new EventSource(`/api/live?${params.toString()}`);
    eventSource = source;

    source.addEventListener("meta", (event) => {
      const meta = JSON.parse(event.data);
      sessionId = meta.session_id;
      currentScenario = meta.scenario || currentScenario;
      $("model-chip").textContent = `model: ${currentScenario} / ${meta.profile}`;
      $("stream-chip").textContent = "SIM: continuous native stream";
      setKickControls(true);
    });

    source.addEventListener("sample", (event) => {
      const sample = JSON.parse(event.data);
      samples.push(sample);
      if (samples.length > 3000) samples.splice(0, samples.length - 3000);
      updateTelemetry(sample);
    });

    source.addEventListener("disturbance", (event) => {
      const injected = JSON.parse(event.data);
      disturbances.push(injected);
      kickFlash = {
        kind: injected.kind,
        delta: Number(injected.delta_rad_s),
        until: performance.now() + 450,
      };
      const label = injected.kind === "body" ? "BODY" : "WHEEL";
      $("kick-readout").textContent = `${label} ${injected.delta_rad_s >= 0 ? "+" : ""}${number(injected.delta_rad_s, 2)} rad/s`;
      setTimeout(() => {
        if (kickFlash && performance.now() > kickFlash.until) {
          $("kick-readout").textContent = "";
          drawScene();
        }
      }, 500);
      updateDisturbanceStatus();
      drawHistory();
    });

    source.addEventListener("end", (event) => {
      const result = JSON.parse(event.data);
      closeStream();
      sessionId = null;
      stopping = false;
      resetControls();
      $("stream-chip").textContent = "SIM: stopped";
      setLiveStatus("STOPPED", `Stopped by user at t=${number(result.t_s, 3)} s.`);
    });

    source.addEventListener("stream-error", (event) => {
      const error = JSON.parse(event.data);
      closeStream();
      sessionId = null;
      stopping = false;
      resetControls();
      $("stream-chip").textContent = "SIM: backend error";
      setLiveStatus("FAIL", error.message || "Native live SITL failed.");
    });

    source.onerror = () => {
      if (!eventSource) return;
      closeStream();
      sessionId = null;
      stopping = false;
      resetControls();
      $("stream-chip").textContent = "SIM: disconnected";
      setLiveStatus("FAIL", "Live SSE connection closed unexpectedly.");
    };
  }

  async function requestStop() {
    if (!sessionId || stopping) return;
    stopping = true;
    stopButton.disabled = true;
    setKickControls(false);
    $("stream-chip").textContent = "SIM: stopping native run";
    try {
      await postJson("/api/stop", { session_id: sessionId });
    } catch (error) {
      closeStream();
      sessionId = null;
      stopping = false;
      resetControls();
      setLiveStatus("FAIL", String(error));
    }
  }

  async function injectDisturbance(kind, delta) {
    if (!sessionId || stopping) return;
    try {
      await postJson("/api/disturbance", {
        session_id: sessionId,
        kind,
        delta_rad_s: delta,
      });
    } catch (error) {
      setLiveStatus("FAIL", `Disturbance injection failed: ${error}`);
    }
  }

  runButton.addEventListener("click", startRun);
  stopButton.addEventListener("click", requestStop);
  bodyLeft.addEventListener("click", () => injectDisturbance("body", -Number(bodyStrength.value)));
  bodyRight.addEventListener("click", () => injectDisturbance("body", Number(bodyStrength.value)));
  wheelMinus.addEventListener("click", () => injectDisturbance("wheel", -Number(wheelStrength.value)));
  wheelPlus.addEventListener("click", () => injectDisturbance("wheel", Number(wheelStrength.value)));
  scenario.addEventListener("change", () => {
    currentScenario = scenario.value;
    $("scenario-readout").textContent = currentScenario === "full-standup" ? "Full standup from rest" : "Upright balance only";
    drawHistory();
  });
  window.addEventListener("resize", () => {
    drawScene();
    drawHistory();
  });
  window.addEventListener("beforeunload", () => {
    if (!sessionId) return;
    const blob = new Blob([JSON.stringify({ session_id: sessionId })], { type: "application/json" });
    navigator.sendBeacon("/api/stop", blob);
  });

  drawScene();
  drawHistory();
  updateDisturbanceStatus();
  setLiveStatus("STOPPED", "Press Run live; full standup starts from the resting orientation.");
})();
