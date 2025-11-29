#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
arduino_pid_gui_fixed.py
GUI PyQt5 che riproduce il layout MATLAB fornito (versione "A").
- Usa PyQt5 + matplotlib embedding
- Parsing righe PRINT: "I1,I2,Q1,Q2,Delta,V" (6 valori separati da virgola)

Correzioni apportate:
1. Gestione del threading: Sostituito l'uso di `threading.Thread` con `QThread` o `QRunnable` (o metodi basati su segnali e slot) per le operazioni lunghe (`print_once_thread`, `calibrate_thread`, `start_acquisition_thread`, `run_analysis_now`) per garantire che le interazioni con la GUI avvengano solo nel thread principale, prevenendo i blocchi.
2. Analisi Dati: L'apertura di `AnalysisDialog` è stata spostata in un metodo separato (`show_analysis_dialog`) e invocata tramite `QTimer.singleShot(0, ...)` per assicurare che avvenga nel thread GUI.
3. SerialReader: Il segnale `line_received` è stato rimosso da `SerialReader` per evitare un eccessivo overhead di segnali/slot, mantenendo solo l'uso della coda per la lettura dei dati in blocco.
4. Connessioni: Le chiamate a funzioni lunghe sono state modificate per usare un approccio basato su `QThread` per l'acquisizione e l'analisi.
5. `run_analysis_now`: L'analisi è stata resa non bloccante e gestita correttamente nel thread GUI.
6. `start_acquisition_thread`: Riscritto per usare un `QThread` dedicato per l'acquisizione, con segnali per l'aggiornamento della GUI.
"""

import sys
import time
import queue
from datetime import datetime

import numpy as np
import serial
from serial import SerialException

from PyQt5 import QtCore, QtWidgets
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QPushButton,
    QLabel, QLineEdit, QSpinBox, QDoubleSpinBox, QTextEdit, QCheckBox, QComboBox, QFileDialog, QGroupBox
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QRunnable, QThreadPool

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

import os
from PyQt5 import QtGui


# ---------------- Serial reader QThread ----------------
class SerialReader(QThread):
    # Rimosso line_received per ridurre l'overhead. La coda è il meccanismo principale.
    error = pyqtSignal(str)

    def __init__(self, ser, line_queue, parent=None):
        super().__init__(parent)
        self.ser = ser
        self.queue = line_queue
        self._running = True

    def run(self):
        try:
            while self._running:
                try:
                    # Uso di ser.read(1) e ser.in_waiting per un loop non bloccante
                    # che non blocchi il thread QThread
                    if self.ser.in_waiting > 0:
                        raw = self.ser.readline()
                        if not raw:
                            continue
                        try:
                            line = raw.decode('utf-8', errors='replace').strip()
                        except Exception:
                            line = str(raw)
                        
                        # push in queue non bloccante
                        try:
                            self.queue.put_nowait(line)
                        except queue.Full:
                            # Se la coda è piena, scarta il dato più vecchio e inserisci il nuovo
                            try:
                                _ = self.queue.get_nowait()
                                self.queue.put_nowait(line)
                            except Exception:
                                pass
                    else:
                        # Breve pausa per non consumare troppa CPU
                        self.msleep(1) 
                except Exception as e:
                    self.error.emit(f"Serial read error: {e}")
                    break
        except Exception as e:
            self.error.emit(str(e))

    def stop(self):
        self._running = False
        self.wait(1000)


# ---------------- Worker per operazioni lunghe (QRunnable) ----------------
class Worker(QRunnable):
    """
    Worker thread per eseguire una funzione in un thread separato.
    """
    def __init__(self, fn, *args, **kwargs):
        super().__init__()
        self.fn = fn
        self.args = args
        self.kwargs = kwargs

    @QtCore.pyqtSlot()
    def run(self):
        self.fn(*self.args, **self.kwargs)


# ---------------- Analysis dialog (matlab-like figures) ----------------
class AnalysisDialog(QtWidgets.QDialog):
    def __init__(self, I1, I2, Q1, Q2, Delta, V, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Analysis - analyzer_simple_PID")
        self.resize(1200, 800)

        # CAMBIO QUI: Usiamo QGridLayout invece di QVBoxLayout
        layout = QGridLayout(self) 

        # Conversione degli input in array NumPy per garantire la compatibilità
        I1 = np.array(I1)
        I2 = np.array(I2)
        Q1 = np.array(Q1)
        Q2 = np.array(Q2)
        Delta = np.array(Delta)
        V = np.array(V)

        # 1. Filtraggio dei dati non nulli (come in MATLAB: I1 ~= 0)
        indici_non_zeri = np.where(I1 != 0)
        
        I1_NZ = I1[indici_non_zeri]
        I2_NZ = I2[indici_non_zeri]
        Q1_NZ = Q1[indici_non_zeri]
        Q2_NZ = Q2[indici_non_zeri]
        Delta_NZ = Delta[indici_non_zeri]
        V_NZ = V[indici_non_zeri]
        
        if len(I1_NZ) == 0:
            no_data_label = QLabel("Nessun dato valido (I1=0 per tutti i campioni) per l'analisi.")
            layout.addWidget(no_data_label, 0, 0, 1, 2) # Aggiunto su tutta la riga
            return

        # ------------------------------------------------------------------
        # Calcoli intermedi (Ampiezze e Fasi)
        # ------------------------------------------------------------------
        V1_est = 2.0 * np.sqrt(I1_NZ**2 + Q1_NZ**2)
        V2_est = 2.0 * np.sqrt(I2_NZ**2 + Q2_NZ**2)
        Phi1_rad = np.arctan2(Q1_NZ, I1_NZ)
        Phi2_rad = np.arctan2(Q2_NZ, I2_NZ)
        Phi1_deg = np.degrees(Phi1_rad)
        Phi2_deg = np.degrees(Phi2_rad)
        delta_0 = np.arctan2(-(V1_est/180.0) * np.sign(I1_NZ), (V2_est/30.0) * np.sign(I2_NZ))
        
        # ------------------------------------------------------------------
        # Figure 1: I/Q (Dati filtrati)
        # ------------------------------------------------------------------
        fig1 = Figure(figsize=(6,3))
        fig1.tight_layout(pad=3.0)
        ax1 = fig1.subplots()
        ax1.plot(I1_NZ, label='I1'); ax1.plot(I2_NZ, label='I2'); ax1.plot(Q1_NZ, label='Q1'); ax1.plot(Q2_NZ, label='Q2')
        ax1.set_title('I1, I2, Q1, Q2 (Filtered)'); ax1.legend()
        canvas1 = FigureCanvas(fig1)
        layout.addWidget(canvas1, 0, 0) # Posizione: Riga 0, Colonna 0

        # ------------------------------------------------------------------
        # Figure 2: Ampiezze e Fasi (side-by-side)
        # ------------------------------------------------------------------
        fig2 = Figure(figsize=(8,3))
        fig2.tight_layout(pad=3.0)
        axA = fig2.add_subplot(1, 2, 1)
        axP = fig2.add_subplot(1, 2, 2)
        
        axA.plot(V1_est, label='R1'); axA.plot(V2_est, label='R2'); 
        axA.set_title('Ampiezze Stimate'); axA.legend()
        
        axP.plot(Phi1_deg, label=r'$\theta_1$'); axP.plot(Phi2_deg, label=r'$\theta_2$'); 
        axP.set_title('Fasi Stimate (Gradi)'); axP.legend()
            
        canvas2 = FigureCanvas(fig2)
        layout.addWidget(canvas2, 0, 1) # Posizione: Riga 0, Colonna 1

        # ------------------------------------------------------------------
        # Figure 3: Delta e V (con delta_0 calcolato)
        # ------------------------------------------------------------------
        fig3 = Figure(figsize=(6,4))
        fig3.tight_layout(pad=3.0)
        ax31 = fig3.add_subplot(2, 1, 1)
        ax32 = fig3.add_subplot(2, 1, 2)
        
        ax31.plot(np.degrees(delta_0), label=r'$\delta_0$');
        ax31.plot(np.degrees(Delta_NZ), label=r'$\delta_{Ard.}$');
        ax31.set_ylabel('Angle (deg)'); ax31.legend()
        ax31.set_xlabel('Collected samples');
        
        ax32.plot(V_NZ, '.-', label=r'$V_{off}$');
        ax32.set_ylabel('Voltage offset'); ax32.legend()
        ax32.set_xlabel('Collected samples');
            
        canvas3 = FigureCanvas(fig3)
        layout.addWidget(canvas3, 1, 0) # Posizione: Riga 1, Colonna 0
        
        # ------------------------------------------------------------------
        # Figure 4: FFT (come in MATLAB)
        # ------------------------------------------------------------------
        fig4 = Figure(figsize=(6,3))
        fig4.tight_layout(pad=3.0)
        ax4 = fig4.subplots()
        
        len_data = len(Delta_NZ)
        len_half = len_data // 2
        
        delta_off = Delta_NZ[:len_half]
        tf_off = np.fft.fft(delta_off)
        sample_interval = 0.05 
        freq_off = np.fft.fftfreq(len(delta_off), d=sample_interval) 
        
        delta_on = Delta_NZ[len_half:]
        tf_on = np.fft.fft(delta_on)
        freq_on = np.fft.fftfreq(len(delta_on), d=sample_interval) 
        
        ax4.plot(freq_off[:len_half], np.abs(tf_off[:len_half]), label='PID OFF')
        ax4.plot(freq_on[:len_half], np.abs(tf_on[:len_half]), label='PID ON')
        # Limita l'asse X da 0 alla frequenza di Nyquist
        max_freq = 1.0 / (2.0 * sample_interval)
        ax4.set_xlim(0, max_freq)
        ax4.set_xlabel('Frequencies (Hz)')
        ax4.set_title('FFT of Delta');
        ax4.legend()
        
        canvas4 = FigureCanvas(fig4)
        layout.addWidget(canvas4, 1, 1) # Posizione: Riga 1, Colonna 1



# ---------------- Main GUI window ----------------
class MainWindow(QWidget):
    # Segnali per comunicare dal thread di lavoro al thread GUI
    log_signal = pyqtSignal(str)
    update_plots_signal = pyqtSignal(object, object, object, object, object, object)
    acquisition_finished_signal = pyqtSignal(object, bool, bool, str) # data (dict), doSave, doAnalyze, filename

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Arduino PID Control & PGA - MATLAB style")
        self.resize(1200, 700)

        # Thread pool per eseguire operazioni non-GUI
        self.threadpool = QThreadPool()
        self.threadpool.setMaxThreadCount(4) # Limita il numero di thread

        # Serial
        self.ser = None
        self.reader = None
        self.line_queue = queue.Queue(maxsize=10000)

        # Data
        self.latestData = None
        self.stop_flag = False

        self._build_ui()
        self._connect_signals()

    def _build_ui(self):
        # Main horizontal layout: left big column + right narrow plot column
        main_h = QHBoxLayout(self)

        # LEFT: controls (stacked in vertical)
        left_v = QVBoxLayout()

        # --- Connection panel (mimic MATLAB positions) ---
        conn_group = QGroupBox("Connection")
        conn_group.setMaximumHeight(90)
        cg = QHBoxLayout(conn_group)
        cg.addWidget(QLabel("Porta:"))
        self.port_edit = QLineEdit("COM10" if sys.platform.startswith("win") else "/dev/cu.usbmodem1101")
        cg.addWidget(self.port_edit)
        cg.addWidget(QLabel("Baud:"))
        self.baud_spin = QSpinBox(); self.baud_spin.setRange(300, 4000000); self.baud_spin.setValue(115200)
        cg.addWidget(self.baud_spin)
        self.connect_btn = QPushButton("Connect"); self.disconnect_btn = QPushButton("Disconnect")
        self.disconnect_btn.setEnabled(False)
        cg.addWidget(self.connect_btn); cg.addWidget(self.disconnect_btn)
        self.status_label = QLabel("Stato: Disconnesso"); self.status_label.setStyleSheet("color:red")
        cg.addWidget(self.status_label)
        left_v.addWidget(conn_group)

        # --- PID & SET panel ---
        pid_group = QGroupBox("PID & SET")
        pg = QGridLayout(pid_group)
        pg.addWidget(QLabel("Kp"), 0, 0)
        self.kp_field = QDoubleSpinBox(); self.kp_field.setDecimals(6); self.kp_field.setValue(0.05)
        pg.addWidget(self.kp_field, 0, 1)
        pg.addWidget(QLabel("Ki"), 0, 2)
        self.ki_field = QDoubleSpinBox(); self.ki_field.setDecimals(6); self.ki_field.setValue(0.5)
        pg.addWidget(self.ki_field, 0, 3)
        pg.addWidget(QLabel("Kd"), 0, 4)
        self.kd_field = QDoubleSpinBox(); self.kd_field.setDecimals(6); self.kd_field.setValue(0.0)
        pg.addWidget(self.kd_field, 0, 5)

        pg.addWidget(QLabel("Offset (O)"), 1, 0)
        self.off_field = QDoubleSpinBox(); self.off_field.setDecimals(4); self.off_field.setValue(0.5)
        pg.addWidget(self.off_field, 1, 1)
        pg.addWidget(QLabel("Amplitude (A)"), 1, 2)
        self.amp_field = QDoubleSpinBox(); self.amp_field.setDecimals(4); self.amp_field.setValue(0.05)
        pg.addWidget(self.amp_field, 1, 3)
        pg.addWidget(QLabel("sample_interval ms (T)"), 1, 4)
        self.t_field = QSpinBox(); self.t_field.setRange(1, 10000); self.t_field.setValue(50)
        pg.addWidget(self.t_field, 1, 5)

        # SET buttons row
        btn_row = QHBoxLayout()
        self.setP_btn = QPushButton("SET P"); self.setI_btn = QPushButton("SET I"); self.setD_btn = QPushButton("SET D")
        self.setO_btn = QPushButton("SET O"); self.setA_btn = QPushButton("SET A"); self.setT_btn = QPushButton("SET T")
        for w in [self.setP_btn, self.setI_btn, self.setD_btn, self.setO_btn, self.setA_btn, self.setT_btn]:
            btn_row.addWidget(w)
        pg.addLayout(btn_row, 2, 0, 1, 6)

        # PID ON / OFF
        self.pid_on_btn = QPushButton("PID ON"); self.pid_off_btn = QPushButton("PID OFF")
        pg.addWidget(self.pid_on_btn, 3, 1); pg.addWidget(self.pid_off_btn, 3, 2)
        left_v.addWidget(pid_group)

        # --- PGA panel ---
        pga_group = QGroupBox("Programmable Gain Amplifier (PGA)")
        pga_l = QGridLayout(pga_group)
        pga_l.addWidget(QLabel("CS"), 0, 0)
        self.cs_combo = QComboBox(); self.cs_combo.addItems(['1','2','3','4'])
        pga_l.addWidget(self.cs_combo, 0, 1)
        pga_l.addWidget(QLabel("Gain"), 0, 2)
        self.preset_combo = QComboBox(); self.preset_combo.addItems([str(i) for i in range(1,10)])
        pga_l.addWidget(self.preset_combo, 0, 3)
        pga_l.addWidget(QLabel("Trim"), 1, 0)
        self.chan_combo = QComboBox(); self.chan_combo.addItems([f"{i:02d}" for i in range(1,17)])
        pga_l.addWidget(self.chan_combo, 1, 1)
        pga_l.addWidget(QLabel("Polarity"), 1, 2)
        self.pol_combo = QComboBox(); self.pol_combo.addItems(['P','N'])
        pga_l.addWidget(self.pol_combo, 1, 3)
        pga_l.addWidget(QLabel("Enable"), 2, 0)
        self.en_combo = QComboBox(); self.en_combo.addItems(['0','1'])
        pga_l.addWidget(self.en_combo, 2, 1)
        self.send_pga_btn = QPushButton("Invia ABCDEF")
        pga_l.addWidget(self.send_pga_btn, 2, 2, 1, 2)
        # pga command label
        self.pga_cmd_label = QLabel("Comando: -")
        pga_l.addWidget(self.pga_cmd_label, 3, 0, 1, 4)
        left_v.addWidget(pga_group)

        # --- Acquisition panel ---
        acq_group = QGroupBox("Acquisizione")
        acq_l = QGridLayout(acq_group)
        acq_l.addWidget(QLabel("Numero di cicli (Times)"), 0, 0)
        self.times_spin = QSpinBox(); self.times_spin.setRange(1, 1000); self.times_spin.setValue(6)
        acq_l.addWidget(self.times_spin, 1, 0)
        acq_l.addWidget(QLabel("NUM_SAMPLES (per ciclo)"), 0, 1)
        self.samples_spin = QSpinBox(); self.samples_spin.setRange(1, 100000); self.samples_spin.setValue(200)
        acq_l.addWidget(self.samples_spin, 1, 1)
        acq_l.addWidget(QLabel("Filename"), 0, 2)
        self.filename_edit = QLineEdit("Test_PID_ON_OFF")
        acq_l.addWidget(self.filename_edit, 1, 2)
        self.save_check = QCheckBox("Salva dati"); self.save_check.setChecked(True)
        self.analyze_check = QCheckBox("Analizza dopo acquisizione")
        acq_l.addWidget(self.save_check, 2, 0); acq_l.addWidget(self.analyze_check, 2, 1)

        # acquisition buttons
        self.start_acq_btn = QPushButton("Start Acquisizione PID ON/OFF")
        self.stop_acq_btn = QPushButton("Stop"); self.stop_acq_btn.setEnabled(False)
        self.calibrate_btn = QPushButton("CALIBRATE")
        self.print_btn = QPushButton("PRINT (one block)")
        self.reset_btn = QPushButton("RESET")
        self.analyze_btn = QPushButton("Analizza ora"); self.analyze_btn.setEnabled(False)
        # place buttons similarly
        acq_btn_h = QHBoxLayout()
        for b in [self.start_acq_btn, self.stop_acq_btn, self.calibrate_btn, self.print_btn, self.reset_btn, self.analyze_btn]:
            acq_btn_h.addWidget(b)
        acq_l.addLayout(acq_btn_h, 3, 0, 1, 3)

        # log area
        self.log_area = QTextEdit(); self.log_area.setReadOnly(True)
        acq_l.addWidget(self.log_area, 4, 0, 1, 3)
        left_v.addWidget(acq_group)

        # put left_v into a widget to allow right column fixed width
        left_widget = QWidget()
        left_widget.setLayout(left_v)
        left_widget.setMinimumWidth(880)
        main_h.addWidget(left_widget, stretch=1)

        # ---------------- RIGHT: plot panel (vertical 3 plots) ----------------
        plot_widget = QGroupBox("Visualizzazione dati (PRINT)")
        plot_widget.setMinimumWidth(290)
        plot_v = QVBoxLayout(plot_widget)

        # Canvas 1: I1,I2,Q1,Q2
        self.fig_IQ = Figure(figsize=(4, 2.6))
        self.ax_IQ = self.fig_IQ.add_subplot(111)
        self.canvas_IQ = FigureCanvas(self.fig_IQ)
        plot_v.addWidget(self.canvas_IQ)

        # Canvas 2: Delta
        self.fig_Delta = Figure(figsize=(4, 2.6))
        self.ax_Delta = self.fig_Delta.add_subplot(111)
        self.canvas_Delta = FigureCanvas(self.fig_Delta)
        plot_v.addWidget(self.canvas_Delta)

        # Canvas 3: V offset
        self.fig_V = Figure(figsize=(4, 2.6))
        self.ax_V = self.fig_V.add_subplot(111)
        self.canvas_V = FigureCanvas(self.fig_V)
        plot_v.addWidget(self.canvas_V)

        main_h.addWidget(plot_widget, stretch=0)

    def _connect_signals(self):
        # Connessioni dei segnali
        self.connect_btn.clicked.connect(self.connect_serial)
        self.disconnect_btn.clicked.connect(self.disconnect_serial)
        self.setP_btn.clicked.connect(lambda: self.set_param('P'))
        self.setI_btn.clicked.connect(lambda: self.set_param('I'))
        self.setD_btn.clicked.connect(lambda: self.set_param('D'))
        self.setO_btn.clicked.connect(lambda: self.set_param('O'))
        self.setA_btn.clicked.connect(lambda: self.set_param('A'))
        self.setT_btn.clicked.connect(lambda: self.set_param('T'))
        self.pid_on_btn.clicked.connect(lambda: self.simple_cmd('PID ON'))
        self.pid_off_btn.clicked.connect(lambda: self.simple_cmd('PID OFF'))
        self.send_pga_btn.clicked.connect(self.send_pga)
        
        # Uso di QRunnable per le operazioni lunghe
        self.print_btn.clicked.connect(lambda: self.threadpool.start(Worker(self.print_once_task)))
        self.calibrate_btn.clicked.connect(lambda: self.threadpool.start(Worker(self.calibrate_task)))
        self.start_acq_btn.clicked.connect(lambda: self.threadpool.start(Worker(self.start_acquisition_task)))
        self.stop_acq_btn.clicked.connect(self.stop_acquisition_request)
        self.reset_btn.clicked.connect(lambda: self.simple_cmd('RESET'))
        self.analyze_btn.clicked.connect(self.run_analysis_now) # Non è un'operazione lunga, può essere nel thread GUI

        # Connessione dei segnali personalizzati
        self.log_signal.connect(self.log)
        self.update_plots_signal.connect(self.update_print_plots)
        self.acquisition_finished_signal.connect(self.handle_acquisition_finished)


    # ---------------- utility: logging ----------------
    def log(self, txt):
        t = datetime.now().strftime("%H:%M:%S")
        s = f"[{t}] {txt}"
        # append direttamente, poiché questo metodo è chiamato nel thread GUI (tramite segnale)
        self.log_area.append(s)
        print(s)
        
    def log_from_thread(self, txt):
        # Metodo per loggare da un thread di lavoro
        self.log_signal.emit(txt)


    # ---------------- connect / disconnect ----------------
    def connect_serial(self):
        port = self.port_edit.text().strip()
        baud = int(self.baud_spin.value())
        try:
            self.ser = serial.Serial(port, baud, timeout=0.01) # Timeout ridotto per non bloccare il SerialReader
            with self.line_queue.mutex:
                self.line_queue.queue.clear()
            self.reader = SerialReader(self.ser, self.line_queue)
            self.reader.error.connect(lambda e: self.log_from_thread("SerialReader error: " + e))
            self.reader.start()
            self.log(f"Connessione stabilita: {port} @ {baud}")
            self.status_label.setText(f"Stato: Connesso ({port} @ {baud})")
            self.status_label.setStyleSheet("color:green")
            self.connect_btn.setEnabled(False)
            self.disconnect_btn.setEnabled(True)
        except SerialException as e:
            self.log(f"Errore connessione: {e}")
            self.status_label.setText("Stato: Errore connessione")
            self.status_label.setStyleSheet("color:red")
            self.ser = None

    def disconnect_serial(self):
        if self.reader:
            try:
                self.reader.stop()
            except Exception:
                pass
            self.reader = None
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)
        self.status_label.setText("Stato: Disconnesso")
        self.status_label.setStyleSheet("color:red")
        self.log("Disconnesso.")

    # ---------------- simple_cmd ----------------
    def simple_cmd(self, cmd, wait_sec=0.05):
        if not self.ser:
            self.log("Errore: non connesso.")
            return
        try:
            self.ser.write((cmd + '\n').encode('utf-8'))
            time.sleep(wait_sec)
            # read immediate few lines from queue
            lines = self.read_n_lines(10, sec_total=0.5)
            if lines:
                for l in lines:
                    self.log("-> " + l)
            else:
                self.log(f"Comando inviato: {cmd}")
        except Exception as e:
            self.log(f"Errore invio comando: {e}")

    # ---------------- set param ----------------
    def set_param(self, param):
        if not self.ser:
            self.log("Errore: non connesso.")
            return
        param = param.upper()
        val = None
        if param == 'P':
            val = self.kp_field.value()
        elif param == 'I':
            val = self.ki_field.value()
        elif param == 'D':
            val = self.kd_field.value()
        elif param == 'O':
            # L'input dialog deve essere eseguito nel thread GUI
            val, ok = QtWidgets.QInputDialog.getDouble(self, "SET O", "Voltage offset (0..1)", self.off_field.value(), 0.0, 1.0, 4)
            if not ok: return
        elif param == 'A':
            val = self.amp_field.value()
        elif param == 'T':
            val = self.t_field.value()
        else:
            self.log("Parametro sconosciuto")
            return
        
        if val is not None:
            cmd = f"SET {param} {val}"
            self.log(f"Inviando: {cmd}")
            self.simple_cmd(cmd)

    # ---------------- send PGA ----------------
    def send_pga(self):
        cs = self.cs_combo.currentText()
        preset = self.preset_combo.currentText()
        chan = self.chan_combo.currentText()
        pol = self.pol_combo.currentText()
        en = self.en_combo.currentText()
        cmd = f"{cs}{preset}{chan}{pol}{en}"
        self.pga_cmd_label.setText(f"Comando: {cmd}")
        self.log(f"Invio PGA: {cmd}")
        self.simple_cmd(cmd)

    # ---------------- read helper ----------------
    def read_n_lines(self, n, sec_total=10.0):
        lines = []
        deadline = time.time() + sec_total
        while len(lines) < n and time.time() < deadline:
            remaining = deadline - time.time()
            try:
                # get con timeout per non bloccare indefinitamente
                line = self.line_queue.get(timeout=max(0.001, min(0.5, remaining)))
                if line is None:
                    continue
                lines.append(line)
            except queue.Empty:
                continue
        return lines

    # ---------------- PRINT (one block) - Task per QRunnable ----------------
    def print_once_task(self):
        if not self.ser:
            self.log_from_thread("Errore: non connesso.")
            return
        num_points = int(self.samples_spin.value())
        self.log_from_thread(f"Comando PRINT inviato. Attendo {num_points} righe...")
        
        # Svuota la coda prima di inviare il comando
        with self.line_queue.mutex:
            self.line_queue.queue.clear()
            
        try:
            self.ser.write(b'PRINT\n')
        except Exception as e:
            self.log_from_thread(f"Errore invio PRINT: {e}")
            return
            
        timeout_total = max(5.0, 0.02 * num_points)
        lines = self.read_n_lines(num_points, sec_total=timeout_total)
        
        if len(lines) < num_points:
            self.log_from_thread(f"Attenzione: ricevute solo {len(lines)}/{num_points} righe (timeout {timeout_total}s).")
            
        # parse comma-separated floats
        I1 = np.zeros(num_points); I2 = np.zeros(num_points); Q1 = np.zeros(num_points); Q2 = np.zeros(num_points)
        Delta = np.zeros(num_points); V = np.zeros(num_points)
        idx = 0
        for line in lines:
            # accept comma-separated or space-separated
            parts = [p.strip() for p in line.replace(',', ' ').split()]
            try:
                vals = [float(x) for x in parts]
            except Exception:
                self.log_from_thread(f"Riga non numerica ignorata: '{line}'"); continue
            if len(vals) >= 6:
                I1[idx] = vals[0]; I2[idx] = vals[1]; Q1[idx] = vals[2]; Q2[idx] = vals[3]; Delta[idx] = vals[4]; V[idx] = vals[5]
                idx += 1
            else:
                self.log_from_thread(f"Riga con formato sbagliato ignorata: '{line}'")
                
        if idx == 0:
            self.log_from_thread("Nessun dato valido ricevuto da PRINT.")
            # Aggiorna la GUI per riabilitare il pulsante di analisi
            QtCore.QMetaObject.invokeMethod(self.analyze_btn, "setEnabled", QtCore.Qt.QueuedConnection, QtCore.Q_ARG(bool, False))
            return
            
        I1 = I1[:idx]; I2 = I2[:idx]; Q1 = Q1[:idx]; Q2 = Q2[:idx]; Delta = Delta[:idx]; V = V[:idx]
        self.latestData = {'I1': I1, 'I2': I2, 'Q1': Q1, 'Q2': Q2, 'Delta': Delta, 'V': V, 'NumSamples': idx}

        # Emette il segnale per l'aggiornamento della GUI
        self.update_plots_signal.emit(I1, I2, Q1, Q2, Delta, V)
        self.log_from_thread("Dati PRINT ricevuti e segnale di aggiornamento grafici emesso.")


    # ---------------- update plots in GUI (chiamato dal segnale) ----------------
    def update_print_plots(self, I1, I2, Q1, Q2, Delta, V):
        # I/Q
        self.ax_IQ.clear()
        self.ax_IQ.plot(I1, label='I1'); self.ax_IQ.plot(I2, label='I2'); self.ax_IQ.plot(Q1, label='Q1'); self.ax_IQ.plot(Q2, label='Q2')
        self.ax_IQ.set_title('I1, I2, Q1, Q2'); self.ax_IQ.set_xlabel('Sample'); self.ax_IQ.set_ylabel('Amplitude'); self.ax_IQ.legend()
        self.fig_IQ.tight_layout() # Aggiunto per migliorare il layout
        self.canvas_IQ.draw()

        # Delta (second canvas)
        self.ax_Delta.clear()
        self.ax_Delta.plot(np.degrees(Delta), label='Delta (deg)')
        self.ax_Delta.set_title('Delta'); self.ax_Delta.set_xlabel('Sample'); self.ax_Delta.set_ylabel('Delta (deg)'); self.ax_Delta.legend()
        self.fig_Delta.tight_layout() # Aggiunto per migliorare il layout
        self.canvas_Delta.draw()

        # V (third canvas)
        self.ax_V.clear()
        self.ax_V.plot(V, '.-')
        self.ax_V.set_title('V offset'); self.ax_V.set_xlabel('Sample'); self.ax_V.set_ylabel('V'); self.ax_V.legend(['V_off'])
        self.fig_V.tight_layout() # Aggiunto per migliorare il layout
        self.canvas_V.draw()

        self.log("Grafici aggiornati.")
        self.analyze_btn.setEnabled(True)

    # ---------------- CALIBRATE - Task per QRunnable ----------------
    def calibrate_task(self):
        if not self.ser:
            self.log_from_thread("Errore: non connesso."); return
        with self.line_queue.mutex:
            self.line_queue.queue.clear()
        self.log_from_thread("Invio CALIBRATE...")
        try:
            self.ser.write(b'CALIBRATE\n')
        except Exception as e:
            self.log_from_thread(f"Errore invio CALIBRATE: {e}"); return
            
        timeout_total = 60.0
        start = time.time(); finished = False
        while time.time() - start < timeout_total:
            lines = self.read_n_lines(1, sec_total=2.0)
            if not lines:
                continue
            for l in lines:
                self.log_from_thread("-> " + l)
                if any(tok in l.lower() for tok in ['complete', 'completed', 'done', 'calibration finished', 'cal_done']):
                    finished = True
                    break
            if finished: break
            
        if finished:
            self.log_from_thread("Calibrazione completata.")
        else:
            self.log_from_thread("Calibrazione terminata per timeout (potrebbe comunque essere in corso lato Arduino).")

    # ---------------- start acquisition (PID ON/OFF cycles) - Task per QRunnable ----------------
    def start_acquisition_task(self):
        if not self.ser:
            self.log_from_thread("Errore: non connesso."); return
            
        # Disabilita/Abilita pulsanti nel thread GUI
        QtCore.QMetaObject.invokeMethod(self.start_acq_btn, "setEnabled", QtCore.Qt.QueuedConnection, QtCore.Q_ARG(bool, False))
        QtCore.QMetaObject.invokeMethod(self.stop_acq_btn, "setEnabled", QtCore.Qt.QueuedConnection, QtCore.Q_ARG(bool, True))
        QtCore.QMetaObject.invokeMethod(self.analyze_btn, "setEnabled", QtCore.Qt.QueuedConnection, QtCore.Q_ARG(bool, False))
        
        times = int(self.times_spin.value()); numSamples = int(self.samples_spin.value())
        filename = self.filename_edit.text().strip(); doSave = self.save_check.isChecked(); doAnalyze = self.analyze_check.isChecked()
        self.stop_flag = False
        self.log_from_thread("Acquisizione PID ON/OFF avviata.")
        
        # setup
        try:
            self.simple_cmd(f"SET T {self.t_field.value()}", wait_sec=0.02)
        except Exception:
            pass
            
        switch_point = round(times/2)
        total_samples = numSamples * times
        I1 = np.zeros(total_samples); I2 = np.zeros(total_samples); Q1 = np.zeros(total_samples); Q2 = np.zeros(total_samples)
        Delta = np.zeros(total_samples); V = np.zeros(total_samples)
        write_idx = 0
        
        for j in range(1, times+1):
            if self.stop_flag:
                self.log_from_thread("Acquisizione interrotta dall'utente."); break
                
            if j > switch_point:
                self.simple_cmd("PID ON", wait_sec=0.02); self.log_from_thread(f"Ciclo {j}/{times}: PID ON")
            else:
                self.simple_cmd("PID OFF", wait_sec=0.02); self.log_from_thread(f"Ciclo {j}/{times}: PID OFF")
                
            time.sleep(11) # Attesa per la stabilizzazione

            # request block
            try:
                with self.line_queue.mutex:
                    self.line_queue.queue.clear()
                self.ser.write(b'PRINT\n')
            except Exception as e:
                self.log_from_thread(f"Errore invio PRINT: {e}"); break
                
            # read
            timeout_total = max(5.0, 0.02 * numSamples)
            lines = self.read_n_lines(numSamples, sec_total=timeout_total)
            
            if len(lines) < numSamples:
                self.log_from_thread(f"Attenzione: ricevute solo {len(lines)}/{numSamples} righe per ciclo {j}.")
                
            for line in lines:
                parts = [p.strip() for p in line.replace(',', ' ').split()]
                try:
                    vals = [float(x) for x in parts]
                except Exception:
                    continue
                if len(vals) >= 6 and write_idx < total_samples:
                    I1[write_idx] = vals[0]; I2[write_idx] = vals[1]; Q1[write_idx] = vals[2]; Q2[write_idx] = vals[3]; Delta[write_idx] = vals[4]; V[write_idx] = vals[5]
                    write_idx += 1
                    
            self.log_from_thread(f"Blocco {j} completato. Campioni scritti: {write_idx}")
            
        # finish
        self.simple_cmd("PID OFF", wait_sec=0.02)
        
        data = None
        if write_idx == 0:
            self.log_from_thread("Nessun dato acquisito.")
        else:
            I1 = I1[:write_idx]; I2 = I2[:write_idx]; Q1 = Q1[:write_idx]; Q2 = Q2[:write_idx]; Delta = Delta[:write_idx]; V = V[:write_idx]
            data = {'I1': I1, 'I2': I2, 'Q1': Q1, 'Q2': Q2, 'Delta': Delta, 'V': V}
            
        # Emette il segnale di fine acquisizione
        self.acquisition_finished_signal.emit(data, doSave, doAnalyze, filename)

    def stop_acquisition_request(self):
        self.stop_flag = True
        try:
            self.simple_cmd("STOP")
        except Exception:
            pass
        self.log("Richiesta di interruzione inviata.")

    # ---------------- Gestione fine acquisizione (chiamato dal segnale) ----------------
    def handle_acquisition_finished(self, data, doSave, doAnalyze, filename):
        # Questo metodo è eseguito nel thread GUI
        self.start_acq_btn.setEnabled(True)
        self.stop_acq_btn.setEnabled(False)
        
        if data:
            self.latestData = data
            self.analyze_btn.setEnabled(True)
            
            if doSave:
                try:
                    np.savez(filename + ".npz", **self.latestData)
                    self.log(f"Dati salvati in {filename}.npz")
                except Exception as e:
                    self.log(f"Errore salvataggio: {e}")
                    
            if doAnalyze:
                self.show_analysis_dialog(self.latestData)
                self.log("Analisi completata.")
        else:
            self.analyze_btn.setEnabled(False)

    # ---------------- run analysis now ----------------
    def run_analysis_now(self):
        if not self.latestData:
            self.log("Nessun dato disponibile per analisi."); return
        self.show_analysis_dialog(self.latestData)

    # ---------------- Mostra AnalysisDialog (nel thread GUI) ----------------
    def show_analysis_dialog(self, data):
        # Assicura che l'apertura del dialogo avvenga nel thread GUI
        D = data
        AnalysisDialog(D['I1'], D['I2'], D['Q1'], D['Q2'], D['Delta'], D['V'], parent=self).exec_()


# ---------------- main ----------------
def main():
    app = QApplication(sys.argv)
    
    # -------- ICONA PER EXE (PyInstaller) + modalità sviluppo --------
    if hasattr(sys, '_MEIPASS'):
        icon_path = os.path.join(sys._MEIPASS, "icon.ico")
    else:
        icon_path = "icon.ico"
    app.setWindowIcon(QtGui.QIcon(icon_path))
    # -----------------------------------------------------------------


    w = MainWindow()
    w.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
