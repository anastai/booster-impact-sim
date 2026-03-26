const G0 = 9.80665;
const RE = 6371000;

let simData = null;
let chartInstance = null;
let activeTab = 0;

// Update a slider's display value
function u(id, out, fmt) {
  document.getElementById(out).textContent = fmt(document.getElementById(id).value);
}

// Atmosphere models
function getAtmos(type, h) {
  if (type === 'none') return { rho: 0 };
  if (type === 'exp')  return { rho: 1.225 * Math.exp(-h / 8500) };
  if (h > 86000) return { rho: 0 };
  if (h > 20000) {
    const T = 216.65;
    const p = 5474.89 * Math.exp(-G0 * (h - 20000) / (287 * T));
    return { rho: p / (287 * T) };
  }
  if (h > 11000) {
    const T = 216.65;
    const p = 22632.1 * Math.exp(-G0 * (h - 11000) / (287 * T));
    return { rho: p / (287 * T) };
  }
  const T = 288.15 - 0.0065 * h;
  const p = 101325 * Math.pow(T / 288.15, 5.2561);
  return { rho: p / (287 * T) };
}

// Read all inputs and run the physics simulation
function run() {
  const mPay    = +document.getElementById('mPay').value;
  const mProp   = +document.getElementById('mProp').value;
  const mStr    = +document.getElementById('mStr').value;
  const isp     = +document.getElementById('isp').value;
  const thrustN = +document.getElementById('thrust').value * 1000;
  const burnMax = +document.getElementById('burnTime').value;
  const cd      = +document.getElementById('cd').value;
  const cdFall  = +document.getElementById('cdFall').value;
  const diam    = +document.getElementById('diam').value;
  const atmT    = document.getElementById('atm').value;
  const launchA = +document.getElementById('angle').value * Math.PI / 180;

  const Aref     = Math.PI * (diam / 2) ** 2;
  const mdot     = thrustN / (isp * G0);
  const propUsed = Math.min(mProp, mdot * burnMax);

  let mass = mPay + mProp + mStr;
  let vv = 0, vh = 0, h = 0, x = 0, t = 0;
  const dt = 0.2;
  let prop = propUsed, engineOn = true, burnoutH = 0, burnoutT = 0;
  let maxH = 0, maxV = 0;

  const tA = [], hA = [], vA = [], xA = [], dragA = [], thrA = [], accA = [];

  for (let i = 0; i < 60000; i++) {
    const atm   = getAtmos(atmT, h);
    const spd   = Math.sqrt(vv * vv + vh * vh);
    const cdNow = engineOn ? cd : cdFall;
    const drag  = 0.5 * atm.rho * spd * spd * cdNow * Aref;
    const grav  = G0 * Math.pow(RE / (RE + h), 2);

    let thr = 0;
    if (engineOn && prop > 0) {
      thr = thrustN;
      prop -= mdot * dt;
      mass -= mdot * dt;
      if (prop <= 0 || t >= burnMax) {
        engineOn = false;
        burnoutH = h;
        burnoutT = t;
        mass = mPay + mStr + (mProp - propUsed);
      }
    }

    const thrVV  = thr * Math.sin(launchA);
    const thrVH  = thr * Math.cos(launchA);
    const dragVV = spd > 0.01 ? drag * (vv / spd) : 0;
    const dragVH = spd > 0.01 ? drag * (vh / spd) : 0;
    const aV  = (thrVV - dragVV) / mass - grav;
    const aH  = (thrVH - dragVH) / mass;
    const acc = Math.sqrt(aV * aV + aH * aH);

    vv += aV * dt;
    vh += aH * dt;
    h  += vv * dt;
    x  += vh * dt;
    t  += dt;

    if (h < 0) { h = 0; break; }
    if (h > maxH) maxH = h;
    if (spd > maxV) maxV = spd;

    if (i % 5 === 0) {
      tA.push(+t.toFixed(1));
      hA.push(+(h / 1000).toFixed(2));
      vA.push(+spd.toFixed(1));
      xA.push(+(x / 1000).toFixed(2));
      dragA.push(+(drag / 1000).toFixed(2));
      thrA.push(+(thr / 1000).toFixed(2));
      accA.push(+(acc / G0).toFixed(3));
    }
  }

  const impactKm = (x / 1000).toFixed(1);
  document.getElementById('mAlt').textContent    = (maxH / 1000).toFixed(1);
  document.getElementById('mVel').textContent    = maxV.toFixed(0);
  document.getElementById('mBurnH').textContent  = (burnoutH / 1000).toFixed(1);
  document.getElementById('mImpact').textContent = impactKm;

  const box = document.getElementById('impactBox');
  box.classList.add('show');
  document.getElementById('impactDetail').textContent =
    `${impactKm} km downrange from launch site — burnout at ${(burnoutH / 1000).toFixed(1)} km altitude, t = ${burnoutT.toFixed(0)} s`;

  simData = { t: tA, h: hA, v: vA, x: xA, drag: dragA, thr: thrA, acc: accA };
  sc(activeTab);
}

// Switch chart tab
function sc(idx) {
  activeTab = idx;
  document.querySelectorAll('.tab').forEach((t, i) => t.classList.toggle('active', i === idx));
  if (!simData) return;
  if (chartInstance) { chartInstance.destroy(); chartInstance = null; }

  const gc   = 'rgba(0,0,0,0.07)';
  const tc   = '#666';
  const cols = ['#378ADD', '#D85A30', '#1D9E75', '#D4537E', '#7F77DD'];
  const canvas = document.getElementById('ch');

  if (idx === 0) {
    chartInstance = new Chart(canvas, {
      type: 'scatter',
      data: {
        datasets: [{
          label: 'Trajectory',
          data: simData.x.map((x, i) => ({ x, y: simData.h[i] })),
          borderColor: cols[0],
          pointRadius: 1.5,
          showLine: true,
          borderWidth: 2
        }]
      },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        plugins: { legend: { display: false } },
        scales: {
          x: { title: { display: true, text: 'Downrange distance (km)', color: tc }, ticks: { color: tc }, grid: { color: gc } },
          y: { title: { display: true, text: 'Altitude (km)', color: tc }, ticks: { color: tc }, grid: { color: gc } }
        }
      }
    });
    return;
  }

  if (idx === 3) {
    chartInstance = new Chart(canvas, {
      type: 'line',
      data: {
        labels: simData.t,
        datasets: [
          { label: 'Thrust (kN)', data: simData.thr, borderColor: cols[1], pointRadius: 0, borderWidth: 2 },
          { label: 'Drag (kN)',   data: simData.drag, borderColor: cols[3], pointRadius: 0, borderWidth: 2, borderDash: [5, 4] }
        ]
      },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        plugins: { legend: { display: true, labels: { color: tc, boxWidth: 10 } } },
        scales: {
          x: { title: { display: true, text: 'Time (s)', color: tc }, ticks: { color: tc, maxTicksLimit: 10 }, grid: { color: gc } },
          y: { title: { display: true, text: 'kN', color: tc }, ticks: { color: tc }, grid: { color: gc } }
        }
      }
    });
    return;
  }

  const cfgs = [
    null,
    { label: 'Altitude (km)',      data: simData.h,   y: 'km',  c: cols[0] },
    { label: 'Velocity (m/s)',     data: simData.v,   y: 'm/s', c: cols[1] },
    null,
    { label: 'Acceleration (g)',   data: simData.acc, y: 'g',   c: cols[4] }
  ];
  const cfg = cfgs[idx];
  chartInstance = new Chart(canvas, {
    type: 'line',
    data: {
      labels: simData.t,
      datasets: [{
        label: cfg.label,
        data: cfg.data,
        borderColor: cfg.c,
        pointRadius: 0,
        borderWidth: 2,
        fill: true,
        backgroundColor: cfg.c + '18'
      }]
    },
    options: {
      responsive: true,
      maintainAspectRatio: false,
      plugins: { legend: { display: false } },
      scales: {
        x: { title: { display: true, text: 'Time (s)', color: tc }, ticks: { color: tc, maxTicksLimit: 10 }, grid: { color: gc } },
        y: { title: { display: true, text: cfg.y, color: tc }, ticks: { color: tc }, grid: { color: gc } }
      }
    }
  });
}

// Run on page load
run();
