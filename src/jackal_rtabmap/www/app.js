/**
 * Jackal J100 Advanced Autonomy Mission Control - Frontend Logic
 * Real-time WebSocket Telemetry & Teleoperation Engine
 */

let ws = null;
let currentLinear = 0.0;
let currentAngular = 0.0;
let teleopTimer = null;
let startTime = Date.now();

// DOM Elements
const connStatus = document.getElementById('conn-status');
const connText = document.getElementById('conn-text');
const slopeHeaderPill = document.getElementById('slope-header-pill');
const slopeHeaderStatus = document.getElementById('slope-header-status');
const slopeCardStatus = document.getElementById('slope-card-status');

const horizonSky = document.querySelector('.horizon-sky');
const horizonGround = document.querySelector('.horizon-ground');
const horizonLine = document.querySelector('.horizon-line');

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
const footerTime = document.getElementById('footer-time');

// 1. Initialize WebSocket Connection
function connectWebSocket() {
  const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
  const wsUrl = `${protocol}//${window.location.host}/ws`;

  ws = new WebSocket(wsUrl);

  ws.onopen = () => {
    connStatus.className = 'pill connection-pill';
    connText.innerText = 'ONLINE (FastDDS)';
    console.log('[Jackal Mission Control] WebSocket connected.');
  };

  ws.onmessage = (event) => {
    try {
      const data = JSON.parse(event.data);
      updateTelemetry(data);
    } catch (e) {
      console.error('Error parsing telemetry payload:', e);
    }
  };

  ws.onclose = () => {
    connStatus.className = 'pill connection-pill disconnected';
    connText.innerText = 'OFFLINE (Retrying...)';
    setTimeout(connectWebSocket, 2000);
  };

  ws.onerror = (err) => {
    ws.close();
  };
}

// 2. Update Live Telemetry
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

    // Horizon Artificial Gimbal Animation (Translate Y by pitch, rotate by roll)
    const pitchOffset = Math.max(-40, Math.min(40, pitch * 2));
    const transformStr = `translateY(${pitchOffset}px) rotate(${-roll}deg)`;
    horizonSky.style.transform = transformStr;
    horizonGround.style.transform = transformStr;
    horizonLine.style.transform = `translateY(calc(-50% + ${pitchOffset}px)) rotate(${-roll}deg)`;

    // Colors & Status Badges
    slopeHeaderPill.className = `pill slope-pill ${status.toLowerCase()}`;
    slopeHeaderStatus.innerText = `SLOPE: ${total.toFixed(1)}° [${status}]`;

    slopeCardStatus.className = `status-tag status-${status.toLowerCase()}`;
    slopeCardStatus.innerText = status;

    if (status === 'DANGER') {
      barSlope.style.backgroundColor = '#f85149';
    } else if (status === 'CAUTION') {
      barSlope.style.backgroundColor = '#d29922';
    } else {
      barSlope.style.backgroundColor = '#3fb950';
    }
  }

  // B. Odometry & Exploration Status
  if (data.odom) {
    valPose.innerText = `X: ${data.odom.x.toFixed(2)}m, Y: ${data.odom.y.toFixed(2)}m`;
    valYaw.innerText = `${data.odom.yaw.toFixed(1)}°`;
    valSpeed.innerText = `${data.odom.speed.toFixed(2)} m/s`;
  }

  if (data.exploration) {
    explorStatusBanner.innerText = data.exploration.status || 'AUTONOMOUS NAVIGATION READY';
  }

  // C. 3D Semantic Landmarks Table
  if (data.landmarks && Array.isArray(data.landmarks)) {
    landmarkCount.innerText = `${data.landmarks.length} Objects`;

    if (data.landmarks.length > 0) {
      landmarksTbody.innerHTML = '';
      data.landmarks.forEach((lm) => {
        const tr = document.createElement('tr');
        tr.innerHTML = `
          <td><strong>#${lm.id}</strong></td>
          <td><span style="color: var(--accent-cyan); font-weight:600;">🏷️ ${lm.label}</span></td>
          <td><span style="color: var(--accent-green); font-weight:600;">${Math.round(lm.score * 100)}%</span></td>
          <td>(${lm.x.toFixed(2)}, ${lm.y.toFixed(2)}, ${lm.z.toFixed(2)})</td>
          <td>${lm.count}x hits</td>
        `;
        landmarksTbody.appendChild(tr);
      });
    }
  }
}

// 3. Teleoperation & Remote Joystick Controls
function sendCmdVel(linear, angular) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({
      type: 'cmd_vel',
      linear: linear,
      angular: angular
    }));
  }
}

function startContinuousTeleop(linear, angular) {
  currentLinear = linear;
  currentAngular = angular;
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

function attachButtonControls(btn, lin, ang) {
  btn.addEventListener('mousedown', (e) => { e.preventDefault(); startContinuousTeleop(lin, ang); btn.classList.add('active'); });
  btn.addEventListener('mouseup', (e) => { e.preventDefault(); stopTeleop(); btn.classList.remove('active'); });
  btn.addEventListener('mouseleave', () => { stopTeleop(); btn.classList.remove('active'); });
  btn.addEventListener('touchstart', (e) => { e.preventDefault(); startContinuousTeleop(lin, ang); btn.classList.add('active'); });
  btn.addEventListener('touchend', (e) => { e.preventDefault(); stopTeleop(); btn.classList.remove('active'); });
}

attachButtonControls(btnFwd, 0.6, 0.0);
attachButtonControls(btnRev, -0.6, 0.0);
attachButtonControls(btnLeft, 0.0, 0.8);
attachButtonControls(btnRight, 0.0, -0.8);
btnStop.addEventListener('click', stopTeleop);

// Keyboard bindings (WASD)
window.addEventListener('keydown', (e) => {
  if (e.repeat) return;
  const key = e.key.toLowerCase();
  if (key === 'w' || key === 'arrowup') { startContinuousTeleop(0.6, 0.0); btnFwd.classList.add('active'); }
  else if (key === 's' || key === 'arrowdown') { startContinuousTeleop(-0.6, 0.0); btnRev.classList.add('active'); }
  else if (key === 'a' || key === 'arrowleft') { startContinuousTeleop(0.0, 0.8); btnLeft.classList.add('active'); }
  else if (key === 'd' || key === 'arrowright') { startContinuousTeleop(0.0, -0.8); btnRight.classList.add('active'); }
  else if (key === ' ' || key === 'escape') { stopTeleop(); }
});

window.addEventListener('keyup', (e) => {
  const key = e.key.toLowerCase();
  if (['w', 's', 'a', 'd', 'arrowup', 'arrowdown', 'arrowleft', 'arrowright'].includes(key)) {
    stopTeleop();
    [btnFwd, btnRev, btnLeft, btnRight].forEach(b => b.classList.remove('active'));
  }
});

// Quick Action Buttons
document.getElementById('btn-estop').addEventListener('click', () => {
  stopTeleop();
  alert('🛑 EMERGENCY STOP ACTIVATED: Robot velocity set to 0.0');
});

document.getElementById('btn-explore').addEventListener('click', () => {
  alert('⚡ Autonomous Exploration dispatched!');
});

document.getElementById('btn-save-map').addEventListener('click', () => {
  alert('💾 Saving current 3D RTAB-Map & 2D SLAM Occupancy Grid to /maps...');
});

// Uptime Counter
setInterval(() => {
  const elapsed = Math.floor((Date.now() - startTime) / 1000);
  const hrs = String(Math.floor(elapsed / 3600)).padStart(2, '0');
  const mins = String(Math.floor((elapsed % 3600) / 60)).padStart(2, '0');
  const secs = String(elapsed % 60).padStart(2, '0');
  footerTime.innerText = `Uptime: ${hrs}:${mins}:${secs}`;
}, 1000);

// Initialize on page load
window.addEventListener('DOMContentLoaded', () => {
  connectWebSocket();
});
