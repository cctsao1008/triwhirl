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

  // Draw an ideal Reuleaux triangle centered on its geometric center.  The
  // browser does not choose the ground contact or center translation; those
  // come from the native rolling geometry in every sample.
  function drawReuleauxMeters(ctx, widthM, pxPerM) {
    const rv = widthM / Math.sqrt(3);
    const v0 = { x: 0, y: -rv };
    const v1 = { x: widthM / 2, y: rv / 2 };
    const v2 = { x: -widthM / 2, y: rv / 2 };

    ctx.save();
    ctx.fillStyle = "rgba(66,159,230,0.24)";
    ctx.strokeStyle = "#73c4ff";
    ctx.lineWidth = 3 / pxPerM;
    ctx.beginPath();
    ctx.moveTo(v1.x, v1.y);
    ctx.arc(v0.x, v0.y, widthM, Math.PI / 3, 2 * Math.PI / 3, false);
    ctx.arc(v1.x, v1.y, widthM, Math.PI, 4 * Math.PI / 3, false);
    ctx.arc(v2.x, v2.y, widthM, 5 * Math.PI / 3, 2 * Math.PI, false);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
    ctx.restore();
  }

  function drawWheelMeters(ctx, wheelAngle, widthM, pxPerM) {
    const radius = widthM * 0.24; // display only; not used by the plant.
    ctx.save();
    ctx.strokeStyle = "#dce8f6";
    ctx.fillStyle = "rgba(220,232,246,0.08)";
    ctx.lineWidth = 3 / pxPerM;
    ctx.beginPath();
    ctx.arc(0, 0, radius, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
    ctx.rotate(wheelAngle);
    ctx.strokeStyle = "#ffcc66";
    ctx.lineWidth = 2.5 / pxPerM;
    for (let i = 0; i < 6; ++i) {
      const angle = i * Math.PI / 3;
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.lineTo(Math.cos(angle) * radius * 0.90,
                 Math.sin(angle) * radius * 0.90);
      ctx.stroke();
    }
    ctx.fillStyle = "#ffcc66";
    ctx.beginPath();
    ctx.arc(0, 0, 4 / pxPerM, 0, Math.PI * 2);
    ctx.fill();
    ctx.restore();
  }

  function drawGround(ctx, width, height, pxPerM, worldOriginX, groundY) {
    ctx.strokeStyle = "#52657c";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(24, groundY);
    ctx.lineTo(width - 24, groundY);
    ctx.stroke();

    ctx.strokeStyle = "rgba(82,101,124,0.35)";
    ctx.lineWidth = 1;
    const spacingM = 0.02;
    const minWorldX = (24 - worldOriginX) / pxPerM;
    const maxWorldX = (width - 24 - worldOriginX) / pxPerM;
    let tick = Math.floor(minWorldX / spacingM) * spacingM;
    for (; tick <= maxWorldX; tick += spacingM) {
      const x = worldOriginX + tick * pxPerM;
      ctx.beginPath();
      ctx.moveTo(x, groundY + 4);
      ctx.lineTo(x - 10, groundY + 14);
      ctx.stroke();
    }
  }

  function drawKickArrow(ctx, centerScreenX, centerScreenY) {
    if (!kickFlash || performance.now() > kickFlash.until) return;
    const positive = kickFlash.delta > 0;
    const bodyKick = kickFlash.kind === "body";
    ctx.save();
    ctx.strokeStyle = bodyKick ? "#ff6b75" : "#d58cff";
    ctx.fillStyle = ctx.strokeStyle;
    ctx.lineWidth = 4;
    const y = centerScreenY - 90;
    const x0 = centerScreenX + (positive ? -125 : 125);
    const x1 = centerScreenX + (positive ? -55 : 55);
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
    const groundY = height - 62;
    const geometryWidthM = latest && Number.isFinite(Number(latest.geometry_width_m))
      ? Number(latest.geometry_width_m)
      : 0.075;
    const pxPerM = Math.min(3200, Math.max(1900, (width * 0.27) / geometryWidthM));
    const worldOriginX = width * 0.50;

    drawGround(sceneCtx, width, height, pxPerM, worldOriginX, groundY);

    if (!latest) {
      sceneCtx.fillStyle = "#70839c";
      sceneCtx.font = "15px system-ui, sans-serif";
      sceneCtx.textAlign = "center";
      sceneCtx.fillText(
        "Run native SITL — full mode starts at the Reuleaux resting orientation",
        width / 2, 45);
      sceneCtx.textAlign = "left";
      return;
    }

    const bodyAngle = Number(latest.true_body_angle_rad);
    const wheelAngle = Number(latest.true_wheel_angle_rad);
    const centerX = Number(latest.true_body_center_x_m);
    const centerY = Number(latest.true_body_center_y_m);
    const contactX = Number(latest.true_contact_x_m);
    const contactY = Number(latest.true_contact_y_m);

    const centerScreenX = worldOriginX + centerX * pxPerM;
    const centerScreenY = groundY - centerY * pxPerM;
    const contactScreenX = worldOriginX + contactX * pxPerM;
    const contactScreenY = groundY - contactY * pxPerM;

    // Native no-slip pose: translate by native center x/y, then rotate body.
    sceneCtx.save();
    sceneCtx.translate(centerScreenX, centerScreenY);
    sceneCtx.scale(pxPerM, -pxPerM);
    sceneCtx.rotate(bodyAngle);
    drawReuleauxMeters(sceneCtx, geometryWidthM, pxPerM);
    drawWheelMeters(sceneCtx, wheelAngle, geometryWidthM, pxPerM);
    sceneCtx.restore();

    // Draw native COM/contact evidence in world coordinates.
    sceneCtx.strokeStyle = "rgba(101,184,255,0.42)";
    sceneCtx.lineWidth = 1.5;
    sceneCtx.setLineDash([5, 4]);
    sceneCtx.beginPath();
    sceneCtx.moveTo(centerScreenX, centerScreenY);
    sceneCtx.lineTo(contactScreenX, contactScreenY);
    sceneCtx.stroke();
    sceneCtx.setLineDash([]);

    sceneCtx.fillStyle = "#65b8ff";
    sceneCtx.beginPath();
    sceneCtx.arc(centerScreenX, centerScreenY, 5, 0, Math.PI * 2);
    sceneCtx.fill();

    sceneCtx.fillStyle = "#ff6b75";
    sceneCtx.beginPath();
    sceneCtx.arc(contactScreenX, contactScreenY, 5, 0, Math.PI * 2);
    sceneCtx.fill();

    drawKickArrow(sceneCtx, centerScreenX, centerScreenY);

    sceneCtx.fillStyle = "#8fa3bc";
    sceneCtx.font = "12px ui-monospace, SFMono-Regular, Consolas, monospace";
    sceneCtx.fillText(
      `native rolling pose  COM x=${number(centerX * 1000, 1)} mm  ` +
      `h=${number(centerY * 1000, 1)} mm  contact x=${number(contactX * 1000, 1)} mm`,
      22, groundY - 12);
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
      chip.textContent = "LIVE SIM: OUT OF LOCAL ENVELOPE";
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
    if ($("body-x")) $("body-x").textContent = `${number(Number(sample.true_body_center_x_m) * 1000, 2)} mm`;
    if ($("body-y")) $("body-y").textContent = `${number(Number(sample.true_body_center_y_m) * 1000, 2)} mm`;
    if ($("contact-x")) $("contact-x").textContent = `${number(Number(sample.true_contact_x_m) * 1000, 2)} mm`;
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
        setLiveStatus(
          "SWINGING",
          `Geometry-derived exploratory swing-up · no full-standup PASS authority · t=${number(sample.t_s, 1)} s`);
      } else if (sample.stable) {
        setLiveStatus(
          "STABLE",
          `Controller reached Stable in the exploratory geometry model · t=${number(sample.t_s, 1)} s`);
      } else if (sample.phase === "balance") {
        setLiveStatus(
          "CAPTURE",
          `${sample.settling ? "Settling after capture" : "First balance approach"} · t=${number(sample.t_s, 1)} s`);
      }
    } else if (sample.phase !== "balance") {
      setLiveStatus("FAIL", `Left Balance at t=${number(sample.t_s, 3)} s; simulation continues until Stop.`);
    } else if (sample.stable) {
      setLiveStatus("STABLE", `Local identified-model balance · t=${number(sample.t_s, 1)} s`);
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
    $("scenario-readout").textContent = currentScenario === "full-standup"
      ? "Geometry-derived full swing observation"
      : "Upright balance only";
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
    $("scenario-readout").textContent = currentScenario === "full-standup"
      ? "Geometry-derived full swing observation"
      : "Upright balance only";
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
  setLiveStatus(
    "STOPPED",
    "Full mode is exploratory geometry-derived rolling dynamics; local balance remains the validation regression path.");
})();
