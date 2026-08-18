/**
 * Clearpath Jackal J100 • Industrial Autonomy Dashboard
 * Frontend WebSocket Engine & Mission Control Teleoperation
 */

let ws = null;
let currentSpeedLinear = 0.6;
let currentSpeedAngular = 0.9;
let currentLinear = 0.0;
let currentAngular = 0.0;
let teleopTimer = null;
let startTime = Date.now();

// DOM Elements
const connStatus = document.getElementById('conn-status');
const connText = document.getElementById('conn-text');
const headerMode = document.getElementById('header-mode');
const slopeHeaderBadge = document.getElementById('slope-header-badge');
const slopeHeaderText = document.getElementById('slope-header-text');
const slopeCardStatus = document.getElementById('slope-card-status');
const cameraFeedTag = document.getElementById('camera-feed-tag');

const horizonCanvas = document.getElementById('horizon-canvas');
const horizonCtx = horizonCanvas ? horizonCanvas.getContext('2d') : null;

const valTotalSlope = document.getElementById('val-total-slope');
const barSlope = document.getElementById('bar-slope');
const valPitch = document.getElementById('val-pitch');
const valRoll = document.getElementById('val-roll');

const explorStatusBanner = document.getElementById('exploration-status-banner');
const valPose = document.getElementById('val-pose');
const valYaw = document.getElementById('val-yaw');
const valSpeed = document.getElementById('val-speed');

const landmarksTbody = document.getElementById('landmarks-tbody');
const landmarkCount = document.getElementById('landmark-count');
const footerUptime = document.getElementById('footer-uptime');

// 1. Draw Canvas-Based Artificial Horizon Attitude Gauge
function drawHorizon(pitchDeg, rollDeg) {
  if (!horizonCtx || !horizonCanvas) return;
  const w = horizonCanvas.width;
  const h = horizonCanvas.height;
  const cx = w / 2;
  const cy = h / 2;
  const r = (w / 2) - 2;

  horizonCtx.clearRect(0, 0, w, h);
  horizonCtx.save();

  // Circular clip to guarantee nothing ever overflows outside the ring
  horizonCtx.beginPath();
  horizonCtx.arc(cx, cy, r, 0, Math.PI * 2);
  horizonCtx.clip();

  // Rotate & Translate by roll and pitch
  horizonCtx.translate(cx, cy);
  horizonCtx.rotate((-rollDeg * Math.PI) / 180);
  const pitchPx = Math.max(-r, Math.min(r, pitchDeg * 1.5));
  horizonCtx.translate(0, pitchPx);

  // Draw Sky (Solid Navy Blue #0369a1)
  horizonCtx.fillStyle = '#0369a1';
  horizonCtx.fillRect(-w, -h * 2, w * 2, h * 2);

  // Draw Ground (Solid Earth Brown #78350f)
  horizonCtx.fillStyle = '#78350f';
  horizonCtx.fillRect(-w, 0, w * 2, h * 2);

  // Draw Horizon White Line
  horizonCtx.strokeStyle = '#ffffff';
  horizonCtx.lineWidth = 2;
  horizonCtx.beginPath();
  horizonCtx.moveTo(-w, 0);
  horizonCtx.lineTo(w, 0);
  horizonCtx.stroke();

  // Pitch ladder marks
  horizonCtx.strokeStyle = 'rgba(255, 255, 255, 0.4)';
  horizonCtx.lineWidth = 1;
  for (let deg = -30; deg <= 30; deg += 10) {
    if (deg === 0) continue;
    const y = -deg * 1.5;
    const len = deg % 20 === 0 ? 20 : 10;
    horizonCtx.beginPath();
    horizonCtx.moveTo(-len / 2, y);
    horizonCtx.lineTo(len / 2, y);
    horizonCtx.stroke();
  }

  horizonCtx.restore();

  // Fixed Yellow Reticle in center
  horizonCtx.save();
  horizonCtx.strokeStyle = '#f59e0b';
  horizonCtx.lineWidth = 3;
  // Left wing
  horizonCtx.beginPath();
  horizonCtx.moveTo(cx - 24, cy);
  horizonCtx.lineTo(cx - 8, cy);
  horizonCtx.stroke();
  // Right wing
  horizonCtx.beginPath();
  horizonCtx.moveTo(cx + 8, cy);
  horizonCtx.lineTo(cx + 24, cy);
  horizonCtx.stroke();
  // Center dot
  horizonCtx.fillStyle = '#f59e0b';
  horizonCtx.beginPath();
  horizonCtx.arc(cx, cy, 2.5, 0, Math.PI * 2);
  horizonCtx.fill();

  // Outer border ring
  horizonCtx.strokeStyle = '#334155';
  horizonCtx.lineWidth = 2;
  horizonCtx.beginPath();
  horizonCtx.arc(cx, cy, r, 0, Math.PI * 2);
  horizonCtx.stroke();
  horizonCtx.restore();
}

// 2. Initialize WebSocket Connection
function connectWebSocket() {
  const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
  const wsUrl = `${protocol}//${window.location.host}/ws`;

  ws = new WebSocket(wsUrl);

  ws.onopen = () => {
    connStatus.className = 'badge badge-success';
    connText.innerText = 'ROS 2 Humble • Connected';
    console.log('[Jackal Mission Control] Connected to ROS 2 Telemetry Bridge.');
  };

  ws.onmessage = (event) => {
    try {
      const data = JSON.parse(event.data);
      updateTelemetry(data);
    } catch (e) {
      console.error('Error parsing telemetry JSON:', e);
    }
  };

  ws.onclose = () => {
    connStatus.className = 'badge badge-terrain danger';
    connText.innerText = 'Offline (Reconnecting...)';
    setTimeout(connectWebSocket, 2000);
  };

  ws.onerror = () => {
    ws.close();
  };
}

// 3. Update Live Telemetry
function updateTelemetry(data) {
  // A. Slope & Inclinometer Telemetry
  if (data.slope) {
    const roll = data.slope.roll || 0.0;
    const pitch = data.slope.pitch || 0.0;
    const total = data.slope.total || 0.0;
    const status = data.slope.status || 'SAFE';

    valTotalSlope.innerText = `${total.toFixed(1)}°`;
    valPitch.innerText = `${pitch.toFixed(1)}°`;
    valRoll.innerText = `${roll.toFixed(1)}°`;

    // Progress bar fill (% of 35 deg max danger)
    const pct = Math.min(100, Math.max(2, (total / 35.0) * 100));
    barSlope.style.width = `${pct}%`;

    // Render Canvas Attitude Horizon
    drawHorizon(pitch, roll);

    // Solid Status Colors
    slopeHeaderBadge.className = `badge badge-terrain ${status.toLowerCase()}`;
    slopeHeaderText.innerText = `Slope: ${total.toFixed(1)}° [${status}]`;

    slopeCardStatus.className = `status-chip chip-${status.toLowerCase()}`;
    slopeCardStatus.innerText = status;

    if (status === 'DANGER') {
      barSlope.style.backgroundColor = '#dc2626'; // Solid red
    } else if (status === 'CAUTION') {
      barSlope.style.backgroundColor = '#d97706'; // Solid amber
    } else {
      barSlope.style.backgroundColor = '#059669'; // Solid emerald green
    }
  }

  // B. Camera Feed Mode
  if (data.has_yolo) {
    cameraFeedTag.innerText = 'YOLOV8 SEMANTIC • ACTIVE';
  } else {
    cameraFeedTag.innerText = 'OPTICAL STREAM';
  }

  // C. Odometry & Exploration Status
  if (data.odom) {
    valPose.innerText = `X: ${data.odom.x.toFixed(2)}m, Y: ${data.odom.y.toFixed(2)}m`;
    valYaw.innerText = `${data.odom.yaw.toFixed(1)}°`;
    valSpeed.innerText = `${data.odom.speed.toFixed(2)} m/s`;
  }

  if (data.exploration) {
    explorStatusBanner.innerText = data.exploration.status || 'Autonomous Navigation Active • Mapping Environment';
  }

  // D. 3D Semantic Landmarks Table
  if (data.landmarks && Array.isArray(data.landmarks)) {
    landmarkCount.innerText = `${data.landmarks.length} Tracked Items`;

    if (data.landmarks.length > 0) {
      landmarksTbody.innerHTML = '';
      data.landmarks.forEach((lm) => {
        const tr = document.createElement('tr');
        tr.innerHTML = `
          <td><strong>#${lm.id}</strong></td>
          <td><strong style="color: #60a5fa;">${lm.label.toUpperCase()}</strong></td>
          <td><span style="color: #34d399; font-weight:600;">${Math.round(lm.score * 100)}%</span></td>
          <td>(${lm.x.toFixed(2)}, ${lm.y.toFixed(2)}, ${lm.z.toFixed(2)})</td>
          <td>${lm.count} hits</td>
        `;
        landmarksTbody.appendChild(tr);
      });
    }
  }
}

// 4. Teleoperation & Remote Controls
function sendCmdVel(linear, angular) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({
      type: 'cmd_vel',
      linear: linear,
      angular: angular
    }));
  }
}

function startContinuousTeleop(linDir, angDir) {
  currentLinear = linDir * currentSpeedLinear;
  currentAngular = angDir * currentSpeedAngular;
  sendCmdVel(currentLinear, currentAngular);

  if (teleopTimer) clearInterval(teleopTimer);
  teleopTimer = setInterval(() => {
    sendCmdVel(currentLinear, currentAngular);
  }, 100);
}

function stopTeleop() {
  currentLinear = 0.0;
  currentAngular = 0.0;
  if (teleopTimer) {
    clearInterval(teleopTimer);
    teleopTimer = null;
  }
  sendCmdVel(0.0, 0.0);
}

// Bind D-Pad Buttons
const btnFwd = document.getElementById('btn-fwd');
const btnRev = document.getElementById('btn-rev');
const btnLeft = document.getElementById('btn-left');
const btnRight = document.getElementById('btn-right');
const btnStop = document.getElementById('btn-stop');

function attachButtonControls(btn, linDir, angDir) {
  btn.addEventListener('mousedown', (e) => { e.preventDefault(); startContinuousTeleop(linDir, angDir); btn.classList.add('active'); });
  btn.addEventListener('mouseup', (e) => { e.preventDefault(); stopTeleop(); btn.classList.remove('active'); });
  btn.addEventListener('mouseleave', () => { stopTeleop(); btn.classList.remove('active'); });
  btn.addEventListener('touchstart', (e) => { e.preventDefault(); startContinuousTeleop(linDir, angDir); btn.classList.add('active'); });
  btn.addEventListener('touchend', (e) => { e.preventDefault(); stopTeleop(); btn.classList.remove('active'); });
}

attachButtonControls(btnFwd, 1.0, 0.0);
attachButtonControls(btnRev, -1.0, 0.0);
attachButtonControls(btnLeft, 0.0, 1.0);
attachButtonControls(btnRight, 0.0, -1.0);
btnStop.addEventListener('click', stopTeleop);

// Speed Buttons
document.querySelectorAll('.speed-btn').forEach(btn => {
  btn.addEventListener('click', () => {
    document.querySelectorAll('.speed-btn').forEach(b => b.classList.remove('active'));
    btn.classList.add('active');
    currentSpeedLinear = parseFloat(btn.getAttribute('data-speed'));
    currentSpeedAngular = currentSpeedLinear * 1.5;
  });
});

// Keyboard bindings (WASD)
window.addEventListener('keydown', (e) => {
  if (e.target.tagName === 'INPUT') return;
  if (e.repeat) return;
  const key = e.key.toLowerCase();
  if (key === 'w' || key === 'arrowup') { startContinuousTeleop(1.0, 0.0); btnFwd.classList.add('active'); }
  else if (key === 's' || key === 'arrowdown') { startContinuousTeleop(-1.0, 0.0); btnRev.classList.add('active'); }
  else if (key === 'a' || key === 'arrowleft') { startContinuousTeleop(0.0, 1.0); btnLeft.classList.add('active'); }
  else if (key === 'd' || key === 'arrowright') { startContinuousTeleop(0.0, -1.0); btnRight.classList.add('active'); }
  else if (key === ' ' || key === 'escape') { stopTeleop(); }
});

window.addEventListener('keyup', (e) => {
  if (e.target.tagName === 'INPUT') return;
  const key = e.key.toLowerCase();
  if (['w', 's', 'a', 'd', 'arrowup', 'arrowdown', 'arrowleft', 'arrowright'].includes(key)) {
    stopTeleop();
    [btnFwd, btnRev, btnLeft, btnRight].forEach(b => b.classList.remove('active'));
  }
});

// Quick Action Buttons
document.getElementById('btn-manual-override').addEventListener('click', () => {
  stopTeleop();
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({ type: 'exploration_cmd', command: 'PAUSE' }));
  }
  headerMode.innerText = 'Manual Override (Autonomy Paused)';
  headerMode.parentElement.className = 'badge badge-terrain caution';
  explorStatusBanner.innerText = '✋ Manual Override Active • Autonomy Paused • Drive with D-Pad or WASD';
});

document.getElementById('btn-explore').addEventListener('click', () => {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({ type: 'exploration_cmd', command: 'START' }));
  }
  headerMode.innerText = 'Autonomous Exploration';
  headerMode.parentElement.className = 'badge badge-blue';
  explorStatusBanner.innerText = '⚡ Autonomous Frontier Exploration Active...';
});

document.getElementById('btn-estop').addEventListener('click', () => {
  stopTeleop();
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({ type: 'estop' }));
  }
  headerMode.innerText = 'EMERGENCY BRAKE ENGAGED';
  headerMode.parentElement.className = 'badge badge-terrain danger';
  explorStatusBanner.innerText = '🛑 EMERGENCY BRAKE ENGAGED • Robot Velocity Locked to 0.0 m/s';
});

document.getElementById('btn-save-map').addEventListener('click', () => {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({ type: 'save_map' }));
    alert('💾 Map Saver Triggered: Saving current SLAM map to /home/holetown/ali/Project-Zero/maps...');
  }
});

// Send Nav Goal
document.getElementById('btn-send-goal').addEventListener('click', () => {
  const gx = parseFloat(document.getElementById('goal-x').value) || 0.0;
  const gy = parseFloat(document.getElementById('goal-y').value) || 0.0;
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({
      type: 'nav_goal',
      x: gx,
      y: gy
    }));
    headerMode.innerText = `Navigating to Goal (${gx.toFixed(1)}, ${gy.toFixed(1)})`;
    headerMode.parentElement.className = 'badge badge-blue';
    explorStatusBanner.innerText = `🎯 Navigating to Custom Waypoint: (${gx.toFixed(2)}, ${gy.toFixed(2)})`;
  }
});

// Session Uptime Counter
setInterval(() => {
  const elapsed = Math.floor((Date.now() - startTime) / 1000);
  const hrs = String(Math.floor(elapsed / 3600)).padStart(2, '0');
  const mins = String(Math.floor((elapsed % 3600) / 60)).padStart(2, '0');
  const secs = String(elapsed % 60).padStart(2, '0');
  footerUptime.innerText = `Session Duration: ${hrs}:${mins}:${secs}`;
}, 1000);

// Initialize on page load
window.addEventListener('DOMContentLoaded', () => {
  drawHorizon(0, 0);
  connectWebSocket();
});
