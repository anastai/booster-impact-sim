const G0 = 9.80665;
const RE = 6371000;
const GUIDANCE_GAIN = 4.0;

let simData = null;
let chartInstance = null;
let activeTab = 0;

function u(id, out, fmt) {
  document.getElementById(out).textContent = fmt(document.getElementById(id).value);
}

function getAtmos(type, h) {
  if (type === 'none') return { rho: 0 };
  if (type === 'exp')  return { rho: 1.225 * Math.exp(-h / 8500) };
  if (h > 86000) return { rho: 0 };
  if (h > 20000) {
    const T = 216.65, p = 5474.89 * Math.exp(-G0 * (h - 20000) / (287 * T));
    return { rho: p / (287 * T) };
  }
  if (h > 11000) {
    const T = 216.65, p = 22632.1 * Math.exp(-G0 * (h - 11000) / (287 * T));
    return { rho: p / (287 * T) };
  }
  const T = 288.15 - 0.0065 * h;
  const p = 101325 * Math.pow(T / 288.15, 5.2561);
  return { rho: p / (287 * T) };
}

// Approximate time to ground impact (ignoring drag — used for ZEM prediction)
function estimateTTG(h, vh, grav) {
  const disc = vh * vh + 2 * grav * h;
  if (disc < 0 || h <= 0) return 0.1;
  return Math.max((vh + Math.sqrt(disc)) / grav, 0.1);
}

function run() {
  const mProp   = +document.getElementById('mProp').value;
  const mStr    = +document.getElementById('mStr').value;
  const isp     = +document.getElementById('isp').value;
  const thrustN = +document.getElementById('thrust').value * 1000;
  const burnMax = +document.getElementById('burnTime').value;
  const cd      = +document.getElementById('cd').value;
  const cdFall  = +document.getElementById('cdFall').value;
  const cdCtrl  = +document.getElementById('cdCtrl').value;
  const diam    = +document.getElementById('diam').value;
  const atmT    = document.getElementById('atm').value;
  const launchA = +document.getElementById('angle').value * Math.PI / 180;
  const xTarget = +document.getElementById('xTarget').value * 1000; // km → m
  const yTarget = +document.getElementById('yTarget').value * 1000; // km → m
  const aLatMax = +document.getElementById('aLatMax').value * G0;   // g → m/s²

  const Aref     = Math.PI * (diam / 2) ** 2;
  const mdot     = thrustN / (isp * G0);
  const propUsed = Math.min(mProp, mdot * burnMax);

  // 3-D state: position (x, y, h), velocity (vx, vy, vh)
  // x = downrange, y = crossrange, h = altitude
  let mass = mProp + mStr;
  let vx = 0, vy = 0, vh = 0;
  let x  = 0, y  = 0, h  = 0, t = 0;
  const dt = 0.2;
  let prop = propUsed, engineOn = true, burnoutH = 0, burnoutT = 0;
  let maxH = 0, maxV = 0;

  const tA = [], hA = [], xA = [], yA = [], vA = [];
  const thrA = [], dragAeroA = [], dragCtrlA = [], accA = [];
  const aLatMagA = [], aLatXA = [], aLatYA = [];

  for (let i = 0; i < 60000; i++) {
    const atm  = getAtmos(atmT, h);
    const spd  = Math.sqrt(vx*vx + vy*vy + vh*vh);
    const cdNow = engineOn ? cd : cdFall;
    const grav  = G0 * Math.pow(RE / (RE + h), 2);
    const qS    = 0.5 * atm.rho * spd * spd * Aref; // dynamic pressure × ref area

    // Engine
    let thr = 0;
    if (engineOn && prop > 0) {
      thr = thrustN;
      prop -= mdot * dt;
      mass -= mdot * dt;
      if (prop <= 0 || t >= burnMax) {
        engineOn = false;
        burnoutH = h;
        burnoutT = t;
        mass = mStr + (mProp - propUsed);
      }
    }

    // --- Guidance: zero-effort miss (ZEM), all phases, lateral plane only ---
    let aLatX = 0, aLatY = 0, aLatZ = 0;
    if (h > 50 && spd > 1) {
      const ttg   = estimateTTG(h, vh, grav);
      // Predicted miss in horizontal plane
      const missX = xTarget - (x + vx * ttg);
      const missY = yTarget - (y + vy * ttg);
      // Desired horizontal correction acceleration
      const axDes = GUIDANCE_GAIN * missX / (ttg * ttg + 1.0);
      const ayDes = GUIDANCE_GAIN * missY / (ttg * ttg + 1.0);
      // Project onto plane perpendicular to velocity (true lateral acceleration)
      const vhx = vx / spd, vhy = vy / spd, vhz = vh / spd;
      const dot  = axDes * vhx + ayDes * vhy; // azDes = 0
      aLatX = axDes - dot * vhx;
      aLatY = ayDes - dot * vhy;
      aLatZ =       - dot * vhz;
      // Clamp total magnitude to aLatMax
      const mag = Math.sqrt(aLatX*aLatX + aLatY*aLatY + aLatZ*aLatZ);
      if (mag > aLatMax) {
        const s = aLatMax / mag;
        aLatX *= s; aLatY *= s; aLatZ *= s;
      }
    }
    const aLatMag = Math.sqrt(aLatX*aLatX + aLatY*aLatY + aLatZ*aLatZ);

    // --- Drag ---
    const dragAero = qS * cdNow;
    // Actuator drag: control surfaces deflect to produce lateral force → induced drag ∝ aLat²
    const dragCtrl = aLatMax > 0 ? qS * cdCtrl * (aLatMag / aLatMax) ** 2 : 0;
    const dragTot  = dragAero + dragCtrl;

    // Drag components opposing velocity
    const dragX = spd > 0.01 ? dragTot * (vx / spd) : 0;
    const dragY = spd > 0.01 ? dragTot * (vy / spd) : 0;
    const dragZ = spd > 0.01 ? dragTot * (vh / spd) : 0;

    // Thrust along fixed launch direction (x-z plane, no y component)
    const thrX = thr * Math.cos(launchA);
    const thrZ = thr * Math.sin(launchA);

    // Net acceleration
    const ax  = (thrX - dragX) / mass + aLatX;
    const ay  = (     - dragY) / mass + aLatY;
    const az  = (thrZ - dragZ) / mass - grav + aLatZ;
    const acc = Math.sqrt(ax*ax + ay*ay + az*az);

    vx += ax * dt;  vy += ay * dt;  vh += az * dt;
    x  += vx * dt;  y  += vy * dt;  h  += vh * dt;
    t  += dt;

    if (h < 0) { h = 0; break; }
    if (h > maxH) maxH = h;
    if (spd > maxV) maxV = spd;

    if (i % 5 === 0) {
      tA.push(+t.toFixed(1));
      hA.push(+(h / 1000).toFixed(2));
      xA.push(+(x / 1000).toFixed(2));
      yA.push(+(y / 1000).toFixed(2));
      vA.push(+spd.toFixed(1));
      thrA.push(+(thr / 1000).toFixed(2));
      dragAeroA.push(+(dragAero / 1000).toFixed(2));
      dragCtrlA.push(+(dragCtrl / 1000).toFixed(2));
      accA.push(+(acc / G0).toFixed(3));
      aLatMagA.push(+(aLatMag / G0).toFixed(4));
      aLatXA.push(+(aLatX / G0).toFixed(4));
      aLatYA.push(+(aLatY / G0).toFixed(4));
    }
  }

  const impactX = (x / 1000).toFixed(1);
  const impactY = (y / 1000).toFixed(1);
  const missM   = Math.sqrt((x - xTarget)**2 + (y - yTarget)**2).toFixed(0);

  document.getElementById('mAlt').textContent     = (maxH / 1000).toFixed(1);
  document.getElementById('mVel').textContent     = maxV.toFixed(0);
  document.getElementById('mBurnH').textContent   = (burnoutH / 1000).toFixed(1);
  document.getElementById('mImpactX').textContent = impactX;
  document.getElementById('mImpactY').textContent = impactY;
  document.getElementById('mMiss').textContent    = missM;

  const box = document.getElementById('impactBox');
  box.classList.add('show');
  document.getElementById('impactDetail').textContent =
    `Impact (${impactX}, ${impactY}) km  ·  target (${(xTarget/1000).toFixed(1)}, ${(yTarget/1000).toFixed(1)}) km  ·  miss ${missM} m  ·  burnout ${(burnoutH/1000).toFixed(1)} km alt, t = ${burnoutT.toFixed(0)} s`;

  simData = {
    t: tA, h: hA, x: xA, y: yA, v: vA,
    thr: thrA, dragAero: dragAeroA, dragCtrl: dragCtrlA, acc: accA,
    aLat: aLatMagA, aLatX: aLatXA, aLatY: aLatYA,
    aLatMax: aLatMax / G0,
    xTargetKm: xTarget / 1000,
    yTargetKm: yTarget / 1000
  };
  sc(activeTab);
}

function sc(idx) {
  activeTab = idx;
  document.querySelectorAll('.tab').forEach((t, i) => t.classList.toggle('active', i === idx));
  if (!simData) return;
  if (chartInstance) { chartInstance.destroy(); chartInstance = null; }

  const gc   = 'rgba(0,0,0,0.07)';
  const tc   = '#666';
  const cols = ['#378ADD', '#D85A30', '#1D9E75', '#D4537E', '#7F77DD', '#E8A020'];
  const canvas = document.getElementById('ch');

  // 0 — Vertical profile (x vs altitude)
  if (idx === 0) {
    chartInstance = new Chart(canvas, {
      type: 'scatter',
      data: { datasets: [
        { label: 'Trajectory', data: simData.x.map((x, i) => ({ x, y: simData.h[i] })),
          borderColor: cols[0], pointRadius: 1.5, showLine: true, borderWidth: 2 },
        { label: 'Target', data: [{ x: simData.xTargetKm, y: 0 }],
          borderColor: cols[1], backgroundColor: cols[1], pointRadius: 7, pointStyle: 'crossRot', showLine: false }
      ]},
      options: { responsive: true, maintainAspectRatio: false,
        plugins: { legend: { display: true, labels: { color: tc, boxWidth: 10 } } },
        scales: {
          x: { title: { display: true, text: 'Downrange X (km)', color: tc }, ticks: { color: tc }, grid: { color: gc } },
          y: { title: { display: true, text: 'Altitude (km)',     color: tc }, ticks: { color: tc }, grid: { color: gc } }
        }}
    });
    return;
  }

  // 1 — Top-down view (x vs y ground track)
  if (idx === 1) {
    chartInstance = new Chart(canvas, {
      type: 'scatter',
      data: { datasets: [
        { label: 'Ground track', data: simData.x.map((x, i) => ({ x, y: simData.y[i] })),
          borderColor: cols[2], pointRadius: 1.5, showLine: true, borderWidth: 2 },
        { label: 'Target', data: [{ x: simData.xTargetKm, y: simData.yTargetKm }],
          borderColor: cols[1], backgroundColor: cols[1], pointRadius: 7, pointStyle: 'crossRot', showLine: false }
      ]},
      options: { responsive: true, maintainAspectRatio: false,
        plugins: { legend: { display: true, labels: { color: tc, boxWidth: 10 } } },
        scales: {
          x: { title: { display: true, text: 'Downrange X (km)',  color: tc }, ticks: { color: tc }, grid: { color: gc } },
          y: { title: { display: true, text: 'Crossrange Y (km)', color: tc }, ticks: { color: tc }, grid: { color: gc } }
        }}
    });
    return;
  }

  // 4 — Drag vs thrust (with control drag breakdown)
  if (idx === 4) {
    chartInstance = new Chart(canvas, {
      type: 'line',
      data: { labels: simData.t, datasets: [
        { label: 'Thrust (kN)',       data: simData.thr,      borderColor: cols[1], pointRadius: 0, borderWidth: 2 },
        { label: 'Aero drag (kN)',    data: simData.dragAero, borderColor: cols[3], pointRadius: 0, borderWidth: 2, borderDash: [5, 4] },
        { label: 'Actuator drag (kN)',data: simData.dragCtrl, borderColor: cols[5], pointRadius: 0, borderWidth: 2, borderDash: [2, 3] }
      ]},
      options: { responsive: true, maintainAspectRatio: false,
        plugins: { legend: { display: true, labels: { color: tc, boxWidth: 10 } } },
        scales: {
          x: { title: { display: true, text: 'Time (s)', color: tc }, ticks: { color: tc, maxTicksLimit: 10 }, grid: { color: gc } },
          y: { title: { display: true, text: 'kN',       color: tc }, ticks: { color: tc }, grid: { color: gc } }
        }}
    });
    return;
  }

  // 6 — Control effort (lateral acceleration components + limit lines)
  if (idx === 6) {
    const lim = simData.aLatMax;
    chartInstance = new Chart(canvas, {
      type: 'line',
      data: { labels: simData.t, datasets: [
        { label: '|aLat| (g)',   data: simData.aLat,  borderColor: cols[2], pointRadius: 0, borderWidth: 2, fill: true, backgroundColor: cols[2]+'22' },
        { label: 'X component', data: simData.aLatX, borderColor: cols[0], pointRadius: 0, borderWidth: 1.5, borderDash: [3, 2] },
        { label: 'Y component', data: simData.aLatY, borderColor: cols[4], pointRadius: 0, borderWidth: 1.5, borderDash: [3, 2] },
        { label: `+limit (${lim.toFixed(2)} g)`, data: simData.t.map(() =>  lim), borderColor: cols[1], pointRadius: 0, borderWidth: 1.5, borderDash: [6, 4] },
        { label: '−limit',                        data: simData.t.map(() => -lim), borderColor: cols[1], pointRadius: 0, borderWidth: 1.5, borderDash: [6, 4] }
      ]},
      options: { responsive: true, maintainAspectRatio: false,
        plugins: { legend: { display: true, labels: { color: tc, boxWidth: 10 } } },
        scales: {
          x: { title: { display: true, text: 'Time (s)', color: tc }, ticks: { color: tc, maxTicksLimit: 10 }, grid: { color: gc } },
          y: { title: { display: true, text: 'g',        color: tc }, ticks: { color: tc }, grid: { color: gc } }
        }}
    });
    return;
  }

  // Single-series time-domain charts
  const cfgs = [
    null, null,
    { label: 'Altitude (km)',    data: simData.h,   y: 'km',  c: cols[0] },
    { label: 'Velocity (m/s)',   data: simData.v,   y: 'm/s', c: cols[1] },
    null,
    { label: 'Acceleration (g)', data: simData.acc, y: 'g',   c: cols[4] }
  ];
  const cfg = cfgs[idx];
  chartInstance = new Chart(canvas, {
    type: 'line',
    data: { labels: simData.t, datasets: [{
      label: cfg.label, data: cfg.data,
      borderColor: cfg.c, pointRadius: 0, borderWidth: 2,
      fill: true, backgroundColor: cfg.c + '18'
    }]},
    options: { responsive: true, maintainAspectRatio: false,
      plugins: { legend: { display: false } },
      scales: {
        x: { title: { display: true, text: 'Time (s)', color: tc }, ticks: { color: tc, maxTicksLimit: 10 }, grid: { color: gc } },
        y: { title: { display: true, text: cfg.y,      color: tc }, ticks: { color: tc }, grid: { color: gc } }
      }}
  });
}

run();
