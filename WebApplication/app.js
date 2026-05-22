/* eslint-disable */
/*
 * HTML-fancy port. Logica identica al port HTML/, solo wiring DOM diverso
 * (toggle switch, segmented control, micro-buttons SET, dark-themed canvases).
 */

// =====================================================================
// FakeSerial
// =====================================================================
class FakeSerial {
  constructor() { this.in_waiting = 1; this.is_open = true; }
  readline() {
    const I1 = randn(), I2 = randn(), Q1 = randn(), Q2 = randn();
    const Delta = randn() * 0.1, V = Math.random();
    return new TextEncoder().encode(`${I1},${I2},${Q1},${Q2},${Delta},${V}\n`);
  }
  write(_d) {} close() { this.is_open = false; }
}
function randn() {
  let u = 0, v = 0;
  while (u === 0) u = Math.random();
  while (v === 0) v = Math.random();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}
function sleep(ms) { return new Promise(r => setTimeout(r, ms)); }

// =====================================================================
// SerialReader
// =====================================================================
class SerialReader {
  constructor(ser, q, onMon, onErr) {
    this.ser = ser; this.queue = q; this.onMonitorBlock = onMon; this.onError = onErr;
    this._running = true; this._task = null; this._buf = '';
  }
  start() { this._task = this._run(); }
  async _run() {
    const block_size = 500;
    let mb = [];
    try {
      while (this._running) {
        const line = await this._readLine();
        if (line === null) break;
        if (line === '') continue;
        if (line.startsWith('M,')) {
          mb.push(line);
          if (mb.length >= block_size) { this.onMonitorBlock(mb.slice()); mb = []; }
        } else {
          if (this.queue.length >= 10000) this.queue.shift();
          this.queue.push(line);
        }
      }
    } catch (e) { this.onError(String(e)); }
  }
  async _readLine() {
    if (this.ser instanceof FakeSerial) {
      await sleep(1);
      return new TextDecoder().decode(this.ser.readline()).trim();
    }
    while (this._running) {
      const nl = this._buf.indexOf('\n');
      if (nl >= 0) {
        const line = this._buf.substring(0, nl).replace(/\r$/, '');
        this._buf = this._buf.substring(nl + 1);
        return line.trim();
      }
      try {
        const { value, done } = await this.ser.reader.read();
        if (done) return null;
        if (value) this._buf += value;
      } catch (e) { this.onError(`Serial read error: ${e}`); return null; }
    }
    return null;
  }
  async stop() {
    this._running = false;
    try { if (this.ser?.reader) await this.ser.reader.cancel(); } catch (e) {}
    if (this._task) try { await this._task; } catch (e) {}
  }
}

// =====================================================================
// MainWindow
// =====================================================================
class MainWindow {
  constructor() {
    this.ser = null; this.reader = null;
    this.lineQueue = [];
    this.latestData = null;
    this.stop_flag = false;
    this.is_monitoring = false;
    this.config_key = 'pid_config';

    this.monitor_buffer_size = 250;
    this.monitor_z = new Float32Array(this.monitor_buffer_size);
    this.monitor_ylim = [0, 1023];

    this._bind();
    this.loadParameters();
  }

  log(txt) {
    const t = new Date().toTimeString().substring(0, 8);
    const s = `[${t}] ${txt}`;
    const ta = document.getElementById('log_area');
    ta.textContent += (ta.textContent ? '\n' : '') + s;
    ta.scrollTop = ta.scrollHeight;
    console.log(s);
  }

  _bind() {
    const $ = id => document.getElementById(id);

    $('connect_btn').addEventListener('click', () => this.connectSerial());
    $('disconnect_btn').addEventListener('click', () => this.disconnectSerial());

    document.querySelectorAll('button.set-btn[data-set]').forEach(b => {
      b.addEventListener('click', () => this.setParam(b.dataset.set));
    });

    // Slider <-> numbox sync (Kp/Ki/Kd/Offset/Amplitude/T)
    const slider_pairs = [
      ['kp_range', 'kp_field'],
      ['ki_range', 'ki_field'],
      ['kd_range', 'kd_field'],
      ['off_range', 'off_field'],
      ['amp_range', 'amp_field'],
      ['t_range',  't_field'],
    ];
    for (const [rId, nId] of slider_pairs) {
      const r = document.getElementById(rId);
      const n = document.getElementById(nId);
      if (!r || !n) continue;
      const updateFill = () => {
        const min = parseFloat(r.min), max = parseFloat(r.max);
        const v = parseFloat(r.value);
        const pct = ((v - min) / (max - min)) * 100;
        r.style.setProperty('--val', pct + '%');
      };
      updateFill();
      r.addEventListener('input', () => {
        n.value = r.value;
        updateFill();
      });
      n.addEventListener('input', () => {
        const v = parseFloat(n.value);
        if (!isNaN(v)) {
          // clamp visivo allo slider; il numbox può uscire dal range per SET T
          const min = parseFloat(r.min), max = parseFloat(r.max);
          r.value = Math.max(min, Math.min(max, v));
          updateFill();
        }
      });
    }

    $('send_pga_btn').addEventListener('click', () => this.sendPga());

    $('print_btn').addEventListener('click', () => this.printOnceTask());
    $('calibrate_btn').addEventListener('click', () => this.calibrateTask());
    $('start_acq_btn').addEventListener('click', () => this.startAcquisitionTask());
    $('stop_acq_btn').addEventListener('click', () => this.stopAcquisitionRequest());
    $('reset_btn').addEventListener('click', () => this.simpleCmd('RESET'));
    $('analyze_btn').addEventListener('click', () => this.runAnalysisNow());

    // PID toggle switch
    $('pid_toggle').addEventListener('change', e => {
      this.togglePid(e.target.checked ? 0 : 1);
    });

    // Mode segmented
    document.querySelectorAll('.segmented[data-control="mode"] .seg').forEach(b => {
      b.addEventListener('click', () => {
        document.querySelectorAll('.segmented[data-control="mode"] .seg').forEach(x => x.classList.remove('active'));
        b.classList.add('active');
        this.changeMode(parseInt(b.dataset.idx));
      });
    });

    $('monitor_btn').addEventListener('click', () => this.toggleMonitor());
    $('monitor_close_btn').addEventListener('click', () => this.hideMonitor());

    $('save_params_btn').addEventListener('click', () => this.saveParameters());
    $('load_params_btn').addEventListener('click', () => this.loadParameters());
    $('send_all_params_btn').addEventListener('click', () => this.sendAllParametersTask());

    $('samples_spin').addEventListener('change', () => this.sendNumSamples());

    $('analysis_close_btn').addEventListener('click', () => {
      document.getElementById('analysis_dialog').classList.add('hidden');
    });

    $('clear_log_btn').addEventListener('click', () => {
      document.getElementById('log_area').textContent = '';
    });
  }

  _setStatus(state, label) {
    const pill = document.getElementById('status_label');
    const dot = pill.querySelector('.dot');
    const txt = pill.querySelector('.status-text');
    dot.className = 'dot dot-' + state;
    txt.textContent = label;
  }

  async connectSerial() {
    const baud = parseInt(document.getElementById('baud_spin').value);
    try {
      if (!('serial' in navigator)) throw new Error('Web Serial API non disponibile');
      const port = await navigator.serial.requestPort();
      await port.open({ baudRate: baud });

      const td = new TextDecoderStream();
      const readableClosed = port.readable.pipeTo(td.writable);
      const reader = td.readable.getReader();
      const te = new TextEncoderStream();
      const writableClosed = te.readable.pipeTo(port.writable);
      const writer = te.writable.getWriter();

      this.ser = {
        port, reader, writer, readableClosed, writableClosed, is_open: true,
        async write(data) {
          if (data instanceof Uint8Array) await this.writer.write(new TextDecoder().decode(data));
          else await this.writer.write(data);
        },
        async close() {
          this.is_open = false;
          try { await this.reader.cancel(); } catch (e) {}
          try { await this.writer.close(); } catch (e) {}
          try { await this.readableClosed.catch(() => {}); } catch (e) {}
          try { await this.writableClosed.catch(() => {}); } catch (e) {}
          try { await this.port.close(); } catch (e) {}
        }
      };

      this.lineQueue.length = 0;
      this.reader = new SerialReader(this.ser, this.lineQueue,
        b => this.handleMonitorBlock(b),
        err => this.log('SerialReader error: ' + err));
      this.reader.start();

      this.log(`Connection success @ ${baud}`);
      this._setStatus('green', `Connected @ ${baud}`);
      document.getElementById('connect_btn').disabled = true;
      document.getElementById('disconnect_btn').disabled = false;
    } catch (e) {
      this.log('⚠️ Arduino non trovato — avvio modalità simulazione');
      this.ser = new FakeSerial();
      this.lineQueue.length = 0;
      this.reader = new SerialReader(this.ser, this.lineQueue,
        b => this.handleMonitorBlock(b),
        err => this.log('SerialReader error: ' + err));
      this.reader.start();
      this._setStatus('amber', 'Connected (FAKE)');
      document.getElementById('connect_btn').disabled = true;
      document.getElementById('disconnect_btn').disabled = false;
    }
  }

  async disconnectSerial() {
    if (this.reader) { try { await this.reader.stop(); } catch (e) {} this.reader = null; }
    if (this.ser) { try { await this.ser.close(); } catch (e) {} }
    this.ser = null;
    document.getElementById('connect_btn').disabled = false;
    document.getElementById('disconnect_btn').disabled = true;
    this._setStatus('red', 'Disconnected');
    this.log('Disconnesso.');
  }

  async simpleCmd(cmd, wait_sec = 0.05) {
    if (!this.ser) { this.log('Error: not connected.'); return; }
    try {
      await this.ser.write(cmd + '\n');
      await sleep(wait_sec * 1000);
      const lines = await this.readNLines(10, 0.5);
      if (lines.length > 0) for (const l of lines) this.log('-> ' + l);
      else this.log(`Sent command: ${cmd}`);
    } catch (e) { this.log(`Error command: ${e}`); }
  }

  async setParam(p) {
    if (!this.ser) { this.log('Error: not connected.'); return; }
    p = p.toUpperCase();
    let val = null;
    if (p === 'P') val = parseFloat(document.getElementById('kp_field').value);
    else if (p === 'I') val = parseFloat(document.getElementById('ki_field').value);
    else if (p === 'D') val = parseFloat(document.getElementById('kd_field').value);
    else if (p === 'O') {
      const def = parseFloat(document.getElementById('off_field').value);
      const inp = window.prompt('Voltage offset (0..1)', def);
      if (inp === null) return;
      const v = parseFloat(inp);
      if (isNaN(v) || v < 0 || v > 1) return;
      val = v;
    }
    else if (p === 'A') val = parseFloat(document.getElementById('amp_field').value);
    else if (p === 'T') val = parseInt(document.getElementById('t_field').value);
    else if (p === 'N') val = parseInt(document.getElementById('samples_spin').value);
    else { this.log('Parameters unknown'); return; }

    if (val !== null) {
      const cmd = `SET ${p} ${val}`;
      this.log(`Sending: ${cmd}`);
      await this.simpleCmd(cmd);
      this.saveParameters();
    }
  }

  getParametersFromGui() {
    return {
      Kp: parseFloat(document.getElementById('kp_field').value),
      Ki: parseFloat(document.getElementById('ki_field').value),
      Kd: parseFloat(document.getElementById('kd_field').value),
      Offset: parseFloat(document.getElementById('off_field').value),
      Amplitude: parseFloat(document.getElementById('amp_field').value),
      SampleInterval: parseInt(document.getElementById('t_field').value),
      NumSamples: parseInt(document.getElementById('samples_spin').value),
    };
  }
  setParametersToGui(p) {
    const s = (id, v) => {
      if (v === undefined) return;
      const el = document.getElementById(id);
      el.value = v;
      el.dispatchEvent(new Event('input')); // resync slider fill
    };
    s('kp_field', p.Kp); s('ki_field', p.Ki); s('kd_field', p.Kd);
    s('off_field', p.Offset); s('amp_field', p.Amplitude);
    s('t_field', p.SampleInterval); s('samples_spin', p.NumSamples);
  }
  saveParameters() {
    try {
      localStorage.setItem(this.config_key, JSON.stringify(this.getParametersFromGui()));
      this.log(`Parameters saved in localStorage[${this.config_key}]`);
    } catch (e) { this.log(`Error during data saving: ${e}`); }
  }
  loadParameters() {
    const raw = localStorage.getItem(this.config_key);
    if (!raw) { this.log('Json file not found. Default parameters used.'); return; }
    try {
      this.setParametersToGui(JSON.parse(raw));
      this.log(`Loaded parameters from localStorage[${this.config_key}]`);
    } catch (e) { this.log(`Error during parameters loading: ${e}`); }
  }
  async sendAllParametersTask() {
    if (!this.ser) { this.log('Error not connected. Parameters not found'); return; }
    this.log('Sending all SET parameters to Arduino...');
    const p = this.getParametersFromGui();
    const cmds = [['P', p.Kp], ['I', p.Ki], ['D', p.Kd], ['O', p.Offset], ['A', p.Amplitude],
                  ['T', p.SampleInterval], ['N', parseInt(document.getElementById('samples_spin').value)]];
    for (const [c, v] of cmds) { this.log(`Inviando: SET ${c} ${v}`); await this.simpleCmd(`SET ${c} ${v}`, 0.1); }
    this.log('All parameters send.');
  }
  async sendNumSamples() {
    if (!this.ser) { this.log('WARNING: Serial not connected'); return; }
    const N = parseInt(document.getElementById('samples_spin').value);
    try { await this.ser.write(`SET N ${N}\n`); this.log(`Inviato: SET N ${N}`); }
    catch (e) { this.log(`Errore SET N: ${e}`); }
  }

  async togglePid(idx) {
    if (idx === 0) { await this.simpleCmd('PID ON');  this.log('PID ON'); }
    else           { await this.simpleCmd('PID OFF'); this.log('PID OFF'); }
  }
  async changeMode(idx) {
    if (idx === 0) { await this.simpleCmd('TTL OFF'); await this.simpleCmd('SET 1 0');   this.log('FreqRef OFF (TTL OFF, SET 1 0)'); }
    else if (idx === 1) { await this.simpleCmd('TTL OFF'); await this.simpleCmd('SET 1 0.5'); this.log('FreqRef ON (TTL OFF, SET 1 0.5)'); }
    else if (idx === 2) { await this.simpleCmd('SET 1 0'); await this.simpleCmd('TTL ON');    this.log('TTL ON (FreqRef OFF, TTL ON)'); }
  }

  async toggleMonitor() {
    const p = document.getElementById('monitor_window');
    if (p.classList.contains('hidden')) {
      p.classList.remove('hidden');
      this.is_monitoring = true;
      try { if (this.ser) await this.ser.write('MONITOR ON\n'); } catch (e) {}
      this.log('Monitoring V+/V- activated.');
    } else this.hideMonitor();
  }
  async hideMonitor() {
    document.getElementById('monitor_window').classList.add('hidden');
    this.is_monitoring = false;
    try { if (this.ser) await this.ser.write('MONITOR OFF\n'); } catch (e) {}
    this.log('Monitoring V+/V- deactivated.');
  }
  handleMonitorBlock(lines) {
    const vp = [], vm = [], z = [];
    for (const ln of lines) {
      const parts = ln.trim().split(',');
      if (parts.length === 4 && parts[0] === 'M') {
        const a = parseFloat(parts[1]), b = parseFloat(parts[2]), c = parseFloat(parts[3]);
        if (!isNaN(a) && !isNaN(b) && !isNaN(c)) { vp.push(a); vm.push(b); z.push(c); }
      }
    }
    if (vp.length) {
      document.getElementById('vplus_field').textContent = vp[vp.length - 1].toFixed(3);
      document.getElementById('vminus_field').textContent = vm[vm.length - 1].toFixed(3);
    }
    if (z.length) {
      const n = z.length, bs = this.monitor_z.length;
      if (n >= bs) for (let i = 0; i < bs; i++) this.monitor_z[i] = z[n - bs + i];
      else {
        for (let i = 0; i < bs - n; i++) this.monitor_z[i] = this.monitor_z[i + n];
        for (let i = 0; i < n; i++) this.monitor_z[bs - n + i] = z[i];
      }
      const mx = Math.max(...this.monitor_z);
      if (mx > this.monitor_ylim[1]) this.monitor_ylim[1] = mx * 1.1;
      plotLines('canvas_osc', [{ y: this.monitor_z, label: 'Z', color: '#f472b6' }], { xlabel: 'Samples', glow: true });
    }
  }

  async sendPga() {
    const cs = document.getElementById('cs_combo').value;
    const preset = document.getElementById('preset_combo').value;
    const chan = parseInt(document.getElementById('chan_combo').value).toString().padStart(2, '0');
    const pol = document.getElementById('pol_combo').value;
    const en = document.getElementById('en_combo').value;
    const cmd = `${cs}${preset}${chan}${pol}${en}`;
    document.getElementById('pga_cmd_label').textContent = cmd;
    this.log(`Invio PGA: ${cmd}`);
    await this.simpleCmd(cmd);
  }

  async readNLines(n, sec = 10.0) {
    const out = [];
    const dl = performance.now() + sec * 1000;
    while (out.length < n && performance.now() < dl) {
      if (this.lineQueue.length > 0) out.push(this.lineQueue.shift());
      else await sleep(2);
    }
    return out;
  }

  async printOnceTask() {
    if (!this.ser) { this.log('Error: not connected.'); return; }
    const N = parseInt(document.getElementById('samples_spin').value);
    this.log(`PRINT command sent. Waiting for ${N} lines...`);
    this.lineQueue.length = 0;
    try { await this.ser.write('PRINT\n'); }
    catch (e) { this.log(`PRINT error: ${e}`); return; }

    const to = Math.max(5.0, 0.02 * N);
    const lines = await this.readNLines(N, to);
    if (lines.length < N) this.log(`Attention: only ${lines.length}/${N} lines received (timeout ${to}s).`);

    const I1 = new Float32Array(N), I2 = new Float32Array(N), Q1 = new Float32Array(N),
          Q2 = new Float32Array(N), Delta = new Float32Array(N), V = new Float32Array(N);
    let idx = 0;
    for (const ln of lines) {
      const parts = ln.replace(/,/g, ' ').split(/\s+/).filter(s => s);
      const vals = []; let ok = true;
      for (const p of parts) { const f = parseFloat(p); if (isNaN(f)) { ok = false; break; } vals.push(f); }
      if (!ok) { this.log(`Non-numeric line ignored: '${ln}'`); continue; }
      if (vals.length >= 6) {
        I1[idx] = vals[0]; I2[idx] = vals[1]; Q1[idx] = vals[2];
        Q2[idx] = vals[3]; Delta[idx] = vals[4]; V[idx] = vals[5];
        idx++;
      } else this.log(`Riga con formato sbagliato ignorata: '${ln}'`);
    }
    if (idx === 0) {
      this.log('No valid data received from PRINT.');
      document.getElementById('analyze_btn').disabled = true;
      return;
    }
    const slc = a => a.slice(0, idx);
    this.latestData = { I1: slc(I1), I2: slc(I2), Q1: slc(Q1), Q2: slc(Q2), Delta: slc(Delta), V: slc(V), NumSamples: idx };
    this.updatePrintPlots(this.latestData.I1, this.latestData.I2, this.latestData.Q1, this.latestData.Q2, this.latestData.Delta, this.latestData.V);
    this.log('PRINT commands received and graphics update signal issued.');
  }

  updatePrintPlots(I1, I2, Q1, Q2, Delta, V) {
    plotLines('canvas_IQ', [
      { y: I1, label: 'X1', color: '#22d3ee' },
      { y: I2, label: 'X2', color: '#a78bfa' },
      { y: Q1, label: 'Y1', color: '#34d399' },
      { y: Q2, label: 'Y2', color: '#fbbf24' },
    ], { xlabel: 'Sample' });

    const Ddeg = new Float32Array(Delta.length);
    for (let i = 0; i < Delta.length; i++) Ddeg[i] = Delta[i] * 180 / Math.PI;
    plotLines('canvas_Delta', [{ y: Ddeg, label: 'δ (deg)', color: '#f472b6', glow: true }], { xlabel: 'Sample' });
    plotLines('canvas_V', [{ y: V, label: 'V offset', color: '#22d3ee', marker: true, glow: true }], { xlabel: 'Sample' });

    this.log('Plots updated.');
    document.getElementById('analyze_btn').disabled = false;
  }

  async calibrateTask() {
    if (!this.ser) { this.log('Errore: non connesso.'); return; }
    this.lineQueue.length = 0;
    this.log('Invio CALIBRATE...');
    try { await this.ser.write('CALIBRATE\n'); } catch (e) { this.log(`Errore invio CALIBRATE: ${e}`); return; }
    const to = 60.0, t0 = performance.now();
    let done = false;
    while ((performance.now() - t0) / 1000 < to) {
      const lines = await this.readNLines(1, 2.0);
      if (!lines.length) continue;
      for (const l of lines) {
        this.log('-> ' + l);
        const lw = l.toLowerCase();
        if (['complete', 'completed', 'done', 'calibration finished', 'cal_done'].some(t => lw.includes(t))) { done = true; break; }
      }
      if (done) break;
    }
    this.log(done ? 'Calibration Completed.' : 'Calibration completed due to timeout (may still be in progress on the Arduino side).');
  }

  async startAcquisitionTask() {
    if (!this.ser) { this.log('Error: not connected.'); return; }
    document.getElementById('start_acq_btn').disabled = true;
    document.getElementById('stop_acq_btn').disabled = false;
    document.getElementById('analyze_btn').disabled = true;

    const times = parseInt(document.getElementById('times_spin').value);
    const N = parseInt(document.getElementById('samples_spin').value);
    const fn = document.getElementById('filename_edit').value.trim();
    const doSave = document.getElementById('save_check').checked;
    const doAna = document.getElementById('analyze_check').checked;

    this.stop_flag = false;
    this.log('PID ON/OFF acquisition started.');
    try { await this.simpleCmd(`SET T ${document.getElementById('t_field').value}`, 0.02); } catch (e) {}

    const sp = Math.round(times / 2);
    const TOT = N * times;
    const I1 = new Float32Array(TOT), I2 = new Float32Array(TOT), Q1 = new Float32Array(TOT),
          Q2 = new Float32Array(TOT), Delta = new Float32Array(TOT), V = new Float32Array(TOT);
    let w = 0;

    for (let j = 1; j <= times; j++) {
      if (this.stop_flag) { this.log('Acquisition interrupted by the user.'); break; }
      if (j > sp) { await this.simpleCmd('PID ON', 0.02);  this.log(`Cycle ${j}/${times}: PID ON`); }
      else        { await this.simpleCmd('PID OFF', 0.02); this.log(`Cycle ${j}/${times}: PID OFF`); }
      await sleep(11000);

      try { this.lineQueue.length = 0; await this.ser.write('PRINT\n'); }
      catch (e) { this.log(`Errore invio PRINT: ${e}`); break; }

      const to = Math.max(5.0, 0.02 * N);
      const lines = await this.readNLines(N, to);
      if (lines.length < N) this.log(`Warning: only ${lines.length}/${N} lines received per cycle ${j}..`);
      for (const ln of lines) {
        const parts = ln.replace(/,/g, ' ').split(/\s+/).filter(s => s);
        const vals = []; let ok = true;
        for (const p of parts) { const f = parseFloat(p); if (isNaN(f)) { ok = false; break; } vals.push(f); }
        if (!ok) continue;
        if (vals.length >= 6 && w < TOT) {
          I1[w] = vals[0]; I2[w] = vals[1]; Q1[w] = vals[2]; Q2[w] = vals[3]; Delta[w] = vals[4]; V[w] = vals[5];
          w++;
        }
      }
      this.log(`Block ${j} completed. Samples written: ${w}`);
    }
    await this.simpleCmd('PID OFF', 0.02);

    let data = null;
    if (w === 0) this.log('No data acquired.');
    else data = { I1: I1.slice(0, w), I2: I2.slice(0, w), Q1: Q1.slice(0, w), Q2: Q2.slice(0, w), Delta: Delta.slice(0, w), V: V.slice(0, w) };
    this.handleAcquisitionFinished(data, doSave, doAna, fn);
  }

  stopAcquisitionRequest() {
    this.stop_flag = true;
    try { this.simpleCmd('STOP'); } catch (e) {}
    this.log('STOP request sent.');
  }

  handleAcquisitionFinished(data, doSave, doAna, fn) {
    document.getElementById('start_acq_btn').disabled = false;
    document.getElementById('stop_acq_btn').disabled = true;
    if (data) {
      this.latestData = data;
      document.getElementById('analyze_btn').disabled = false;
      if (doSave) {
        try {
          const obj = Object.fromEntries(Object.entries(data).map(([k, v]) => [k, Array.from(v)]));
          const blob = new Blob([JSON.stringify(obj)], { type: 'application/json' });
          const url = URL.createObjectURL(blob);
          const a = document.createElement('a');
          a.href = url; a.download = (fn || 'data') + '.json'; a.click();
          URL.revokeObjectURL(url);
          this.log(`Data saved in ${fn}.json`);
        } catch (e) { this.log(`Save error: ${e}`); }
      }
      if (doAna) { this.showAnalysisDialog(this.latestData); this.log('Analysis completed.'); }
    } else document.getElementById('analyze_btn').disabled = true;
  }

  runAnalysisNow() {
    if (!this.latestData) { this.log('No data available for analysis.'); return; }
    this.showAnalysisDialog(this.latestData);
  }
  showAnalysisDialog(D) {
    document.getElementById('analysis_dialog').classList.remove('hidden');
    // X1=I1, Y1=I2, X2=Q1, Y2=Q2 (come nel Python)
    renderAnalysis(D.I1, D.I2, D.Q1, D.Q2, D.Delta, D.V);
  }
}

// =====================================================================
// Analysis rendering
// =====================================================================
function renderAnalysis(X1, Y1, X2, Y2, Delta, V) {
  const idx = [];
  for (let i = 0; i < X1.length; i++) if (X1[i] !== 0) idx.push(i);
  if (idx.length === 0) { drawTextCanvas('canvas_a1', 'No valide input data for the analysis.'); return; }
  const pick = a => idx.map(i => a[i]);
  const X1N = pick(X1), Y1N = pick(Y1), X2N = pick(X2), Y2N = pick(Y2), DN = pick(Delta), VN = pick(V);

  const R1 = X1N.map((_, i) => 2 * Math.sqrt(X1N[i]**2 + Y1N[i]**2));
  const R2 = X1N.map((_, i) => 2 * Math.sqrt(X2N[i]**2 + Y2N[i]**2));
  const T1 = X1N.map((_, i) => Math.atan2(Y1N[i], X1N[i]) * 180 / Math.PI);
  const T2 = X1N.map((_, i) => Math.atan2(Y2N[i], X2N[i]) * 180 / Math.PI);

  plotLines('canvas_a1', [
    { y: X1N, label: 'X1', color: '#22d3ee' },
    { y: X2N, label: 'X2', color: '#a78bfa' },
    { y: Y1N, label: 'Y1', color: '#34d399' },
    { y: Y2N, label: 'Y2', color: '#fbbf24' },
  ], { title: 'X1, X2, Y1, Y2' });

  plotSplit('canvas_a2',
    [{ y: R1, label: 'R1', color: '#22d3ee' }, { y: R2, label: 'R2', color: '#a78bfa' }],
    [{ y: T1, label: 'θ1', color: '#22d3ee' }, { y: T2, label: 'θ2', color: '#a78bfa' }],
    'Amplitude', 'Phase (deg)');

  const Ddeg = DN.map(d => d * 180 / Math.PI);
  plotSplitV('canvas_a3',
    [{ y: Ddeg, label: 'δ', color: '#f472b6', glow: true }],
    [{ y: VN, label: 'V_off', color: '#22d3ee', marker: true }],
    'δ (deg)', 'V offset', 'Collected samples');

  const lenH = Math.floor(DN.length / 2);
  const dt = 0.051;
  const fftOff = fftMag(DN.slice(0, lenH), dt);
  const fftOn = fftMag(DN.slice(lenH), dt);
  const mf = 1 / (2 * dt);
  plotXY('canvas_a4', [
    { x: fftOff.f, y: fftOff.m, label: 'PID OFF', color: '#fbbf24' },
    { x: fftOn.f, y: fftOn.m, label: 'PID ON', color: '#22d3ee' },
  ], { xlabel: 'Frequencies (Hz)', title: 'FFT of δ', xlim: [0, mf / 2] });
}

// =====================================================================
// FFT
// =====================================================================
function fftMag(data, dt) {
  const N = data.length;
  const Np2 = 1 << Math.ceil(Math.log2(Math.max(N, 2)));
  const re = new Float64Array(Np2), im = new Float64Array(Np2);
  for (let i = 0; i < N; i++) re[i] = data[i];
  fft(re, im);
  const half = Math.floor(N / 2);
  const m = new Float64Array(half), f = new Float64Array(half);
  for (let i = 0; i < half; i++) {
    const k = Math.round(i * Np2 / N);
    m[i] = Math.sqrt(re[k] * re[k] + im[k] * im[k]);
    f[i] = i / (N * dt);
  }
  return { f, m };
}
function fft(re, im) {
  const n = re.length;
  let j = 0;
  for (let i = 1; i < n; i++) {
    let bit = n >> 1;
    for (; j & bit; bit >>= 1) j ^= bit;
    j ^= bit;
    if (i < j) { [re[i], re[j]] = [re[j], re[i]]; [im[i], im[j]] = [im[j], im[i]]; }
  }
  for (let len = 2; len <= n; len <<= 1) {
    const ang = -2 * Math.PI / len, wRe = Math.cos(ang), wIm = Math.sin(ang);
    for (let i = 0; i < n; i += len) {
      let cR = 1, cI = 0;
      for (let k = 0; k < len / 2; k++) {
        const tre = cR * re[i + k + len / 2] - cI * im[i + k + len / 2];
        const tim = cR * im[i + k + len / 2] + cI * re[i + k + len / 2];
        re[i + k + len / 2] = re[i + k] - tre;
        im[i + k + len / 2] = im[i + k] - tim;
        re[i + k] += tre; im[i + k] += tim;
        const nR = cR * wRe - cI * wIm, nI = cR * wIm + cI * wRe;
        cR = nR; cI = nI;
      }
    }
  }
}

// =====================================================================
// Plotting (dark theme)
// =====================================================================
const PLOT = {
  axis: 'rgba(154, 166, 196, 0.35)',
  grid: 'rgba(154, 166, 196, 0.10)',
  text: 'rgba(230, 236, 255, 0.85)',
  textDim: 'rgba(154, 166, 196, 0.8)',
};

function _minMax(series, accessor = s => s.y) {
  let mn = Infinity, mx = -Infinity;
  for (const s of series) {
    const a = accessor(s);
    for (let i = 0; i < a.length; i++) { const v = a[i]; if (v < mn) mn = v; if (v > mx) mx = v; }
  }
  if (!isFinite(mn) || !isFinite(mx)) { mn = 0; mx = 1; }
  if (mn === mx) { mn -= 1; mx += 1; }
  return [mn, mx];
}

function _frame(ctx, w, h, padL, padR, padT, padB) {
  ctx.clearRect(0, 0, w, h);
  // gradient bg
  const g = ctx.createLinearGradient(0, 0, 0, h);
  g.addColorStop(0, 'rgba(8, 12, 24, 0.0)');
  g.addColorStop(1, 'rgba(20, 28, 50, 0.0)');
  ctx.fillStyle = g;
  ctx.fillRect(0, 0, w, h);

  ctx.strokeStyle = PLOT.axis;
  ctx.lineWidth = 1;
  ctx.strokeRect(padL, padT, w - padL - padR, h - padT - padB);
}

function _grid(ctx, x0, y0, x1, y1, nx = 5, ny = 5) {
  ctx.strokeStyle = PLOT.grid;
  ctx.lineWidth = 1;
  ctx.beginPath();
  for (let i = 1; i < nx; i++) {
    const x = x0 + (x1 - x0) * i / nx;
    ctx.moveTo(x, y0); ctx.lineTo(x, y1);
  }
  for (let i = 1; i < ny; i++) {
    const y = y0 + (y1 - y0) * i / ny;
    ctx.moveTo(x0, y); ctx.lineTo(x1, y);
  }
  ctx.stroke();
}

function _ticks(ctx, x0, y0, x1, y1, xR, yR) {
  ctx.font = '10px "JetBrains Mono", monospace';
  ctx.fillStyle = PLOT.textDim;
  ctx.strokeStyle = PLOT.axis;
  const n = 5;
  for (let i = 0; i <= n; i++) {
    const xpx = x0 + (x1 - x0) * i / n;
    const xv = xR[0] + (xR[1] - xR[0]) * i / n;
    ctx.beginPath(); ctx.moveTo(xpx, y1); ctx.lineTo(xpx, y1 + 3); ctx.stroke();
    const txt = fmt(xv);
    ctx.fillText(txt, xpx - ctx.measureText(txt).width / 2, y1 + 14);
  }
  for (let i = 0; i <= n; i++) {
    const ypx = y1 - (y1 - y0) * i / n;
    const yv = yR[0] + (yR[1] - yR[0]) * i / n;
    ctx.beginPath(); ctx.moveTo(x0, ypx); ctx.lineTo(x0 - 3, ypx); ctx.stroke();
    const txt = fmt(yv);
    ctx.fillText(txt, x0 - 6 - ctx.measureText(txt).width, ypx + 3);
  }
}

function fmt(v) {
  if (v === 0) return '0';
  const a = Math.abs(v);
  if (a >= 10000 || a < 0.01) return v.toExponential(1);
  return (Math.round(v * 1000) / 1000).toString();
}

function _legend(ctx, items, x, y) {
  ctx.font = '600 10px "Inter", sans-serif';
  let dy = 0;
  for (const it of items) {
    ctx.fillStyle = it.color;
    ctx.fillRect(x, y + dy + 4, 10, 2);
    ctx.fillStyle = PLOT.text;
    ctx.fillText(it.label, x + 14, y + dy + 9);
    dy += 14;
  }
}

function plotLines(canvasId, series, opts = {}) {
  const cv = document.getElementById(canvasId);
  if (!cv) return;
  const ctx = cv.getContext('2d');
  const w = cv.width, h = cv.height;
  const padL = 48, padR = 64, padT = opts.title ? 24 : 10, padB = 28;
  _frame(ctx, w, h, padL, padR, padT, padB);

  if (opts.title) {
    ctx.fillStyle = PLOT.text;
    ctx.font = '600 11px "Inter", sans-serif';
    ctx.fillText(opts.title, padL, 16);
  }

  const N = Math.max(...series.map(s => s.y.length));
  const [mn, mx] = _minMax(series);
  const x0 = padL, x1 = w - padR, y0 = padT, y1 = h - padB;
  _grid(ctx, x0, y0, x1, y1);
  _ticks(ctx, x0, y0, x1, y1, [0, Math.max(1, N - 1)], [mn, mx]);

  for (const s of series) {
    ctx.strokeStyle = s.color;
    ctx.fillStyle = s.color;
    ctx.lineWidth = 1.5;
    if (s.glow) { ctx.shadowColor = s.color; ctx.shadowBlur = 8; }
    ctx.beginPath();
    for (let i = 0; i < s.y.length; i++) {
      const xpx = x0 + (x1 - x0) * (i / Math.max(1, N - 1));
      const ypx = y1 - (y1 - y0) * ((s.y[i] - mn) / (mx - mn));
      if (i === 0) ctx.moveTo(xpx, ypx); else ctx.lineTo(xpx, ypx);
      if (s.marker) ctx.fillRect(xpx - 1, ypx - 1, 2, 2);
    }
    ctx.stroke();
    ctx.shadowBlur = 0;
  }

  _legend(ctx, series, x1 + 6, y0 + 4);
  if (opts.xlabel) {
    ctx.fillStyle = PLOT.textDim;
    ctx.font = '10px "Inter", sans-serif';
    const lab = opts.xlabel;
    ctx.fillText(lab, (x0 + x1) / 2 - ctx.measureText(lab).width / 2, h - 6);
  }
}

function plotXY(canvasId, series, opts = {}) {
  const cv = document.getElementById(canvasId);
  if (!cv) return;
  const ctx = cv.getContext('2d');
  const w = cv.width, h = cv.height;
  const padL = 50, padR = 70, padT = opts.title ? 24 : 10, padB = 28;
  _frame(ctx, w, h, padL, padR, padT, padB);

  if (opts.title) {
    ctx.fillStyle = PLOT.text;
    ctx.font = '600 11px "Inter", sans-serif';
    ctx.fillText(opts.title, padL, 16);
  }

  let xmn, xmx;
  if (opts.xlim) { [xmn, xmx] = opts.xlim; }
  else [xmn, xmx] = _minMax(series, s => s.x);
  const [ymn, ymx] = _minMax(series);
  const x0 = padL, x1 = w - padR, y0 = padT, y1 = h - padB;
  _grid(ctx, x0, y0, x1, y1);
  _ticks(ctx, x0, y0, x1, y1, [xmn, xmx], [ymn, ymx]);

  for (const s of series) {
    ctx.strokeStyle = s.color;
    ctx.lineWidth = 1.5;
    if (s.glow) { ctx.shadowColor = s.color; ctx.shadowBlur = 8; }
    ctx.beginPath();
    let started = false;
    for (let i = 0; i < s.x.length; i++) {
      const xv = s.x[i];
      if (xv < xmn || xv > xmx) { started = false; continue; }
      const xpx = x0 + (x1 - x0) * ((xv - xmn) / (xmx - xmn));
      const ypx = y1 - (y1 - y0) * ((s.y[i] - ymn) / (ymx - ymn));
      if (!started) { ctx.moveTo(xpx, ypx); started = true; }
      else ctx.lineTo(xpx, ypx);
    }
    ctx.stroke();
    ctx.shadowBlur = 0;
  }
  _legend(ctx, series, x1 + 6, y0 + 4);
  if (opts.xlabel) {
    ctx.fillStyle = PLOT.textDim;
    ctx.font = '10px "Inter", sans-serif';
    const lab = opts.xlabel;
    ctx.fillText(lab, (x0 + x1) / 2 - ctx.measureText(lab).width / 2, h - 6);
  }
}

function plotSplit(canvasId, ls, rs, lt, rt) {
  const cv = document.getElementById(canvasId);
  const w = cv.width, h = cv.height;
  const ctx = cv.getContext('2d');
  ctx.clearRect(0, 0, w, h);
  drawSubplot(cv, 0, 0, w / 2, h, ls, { title: lt });
  drawSubplot(cv, w / 2, 0, w / 2, h, rs, { title: rt });
}
function plotSplitV(canvasId, ts, bs, tt, bt, xlab) {
  const cv = document.getElementById(canvasId);
  const w = cv.width, h = cv.height;
  const ctx = cv.getContext('2d');
  ctx.clearRect(0, 0, w, h);
  drawSubplot(cv, 0, 0, w, h / 2, ts, { title: tt });
  drawSubplot(cv, 0, h / 2, w, h / 2, bs, { title: bt, xlabel: xlab });
}
function drawSubplot(cv, x, y, w, h, series, opts) {
  const ctx = cv.getContext('2d');
  ctx.save();
  ctx.beginPath(); ctx.rect(x, y, w, h); ctx.clip();
  ctx.translate(x, y);
  const padL = 48, padR = 48, padT = opts.title ? 20 : 8, padB = 26;
  ctx.strokeStyle = PLOT.axis; ctx.lineWidth = 1;
  ctx.strokeRect(padL, padT, w - padL - padR, h - padT - padB);
  if (opts.title) {
    ctx.fillStyle = PLOT.text;
    ctx.font = '600 10.5px "Inter", sans-serif';
    ctx.fillText(opts.title, padL, 14);
  }
  const N = Math.max(...series.map(s => s.y.length));
  const [mn, mx] = _minMax(series);
  const x0 = padL, x1 = w - padR, y0 = padT, y1 = h - padB;
  _grid(ctx, x0, y0, x1, y1, 4, 4);
  _ticks(ctx, x0, y0, x1, y1, [0, Math.max(1, N - 1)], [mn, mx]);
  for (const s of series) {
    ctx.strokeStyle = s.color; ctx.fillStyle = s.color;
    ctx.lineWidth = 1.5;
    if (s.glow) { ctx.shadowColor = s.color; ctx.shadowBlur = 6; }
    ctx.beginPath();
    for (let i = 0; i < s.y.length; i++) {
      const xpx = x0 + (x1 - x0) * (i / Math.max(1, N - 1));
      const ypx = y1 - (y1 - y0) * ((s.y[i] - mn) / (mx - mn));
      if (i === 0) ctx.moveTo(xpx, ypx); else ctx.lineTo(xpx, ypx);
      if (s.marker) ctx.fillRect(xpx - 1, ypx - 1, 2, 2);
    }
    ctx.stroke();
    ctx.shadowBlur = 0;
  }
  _legend(ctx, series, x1 + 4, y0 + 4);
  if (opts.xlabel) {
    ctx.fillStyle = PLOT.textDim;
    ctx.font = '10px "Inter", sans-serif';
    const lab = opts.xlabel;
    ctx.fillText(lab, (x0 + x1) / 2 - ctx.measureText(lab).width / 2, h - 6);
  }
  ctx.restore();
}

function drawTextCanvas(canvasId, text) {
  const cv = document.getElementById(canvasId);
  const ctx = cv.getContext('2d');
  ctx.clearRect(0, 0, cv.width, cv.height);
  ctx.fillStyle = PLOT.text;
  ctx.font = '13px "Inter", sans-serif';
  ctx.fillText(text, 20, 30);
}

// =====================================================================
// boot
// =====================================================================
window.addEventListener('DOMContentLoaded', () => {
  window.gui = new MainWindow();
});
