#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
arduino_pid_gui_fixed.py
GUI PyQt5 che riproduce il layout MATLAB fornito (versione "A").
- Usa PyQt5 + matplotlib embedding
- Parsing righe PRINT: "I1,I2,Q1,Q2,Delta,V" (6 valori separati da virgola)
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
import json # To create the database for the set parameters
from pathlib import Path 


# ---------------- Serial reader QThread ----------------
class SerialReader(QThread):
    # Rimosso line_received per ridurre l'overhead. La coda À il meccanismo principale.
    error = pyqtSignal(str)
    #monitor_line_received = pyqtSignal(str)
    monitor_block_received = pyqtSignal(list)


    def __init__(self, ser, line_queue, parent=None):
        super().__init__(parent)
        self.ser = ser
        self.queue = line_queue
        self._running = True
        
        # Data
        self.latestData = None
        self.stop_flag = False
        self.is_monitoring = False # <-- AGGIUNGI QUESTA RIGA

    def run(self):
        block_size = 500  # numero di campioni da accumulare prima di emettere
        monitor_block = []
    
        try:
            while self._running:
                try:
                    if self.ser.in_waiting > 0:
                        raw = self.ser.readline()
                        if not raw:
                            continue
                        try:
                            line = raw.decode('utf-8', errors='replace').strip()
                        except Exception:
                            line = str(raw)
    
                        if line.startswith('M,'):
                            monitor_block.append(line)
    
                            # Se abbiamo accumulato abbastanza dati, emetti il blocco
                            if len(monitor_block) >= block_size:
                                self.monitor_block_received.emit(monitor_block.copy())
                                monitor_block.clear()
                        else:
                            # Linea normale, inviala alla queue
                            try:
                                self.queue.put_nowait(line)
                            except queue.Full:
                                try:
                                    _ = self.queue.get_nowait()
                                    self.queue.put_nowait(line)
                                except queue.Empty:
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
    Worker thread to execute separated functions.
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
    def __init__(self, X1, Y1, X2, Y2, Delta, V, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Analysis - analyzer_simple_PID")
        self.resize(1200, 800)

        # CAMBIO QUI: Usiamo QGridLayout invece di QVBoxLayout
        layout = QGridLayout(self) 

        # Conversione degli input in array NumPy per garantire la compatibilit?
        X1 = np.array(X1)
        Y1 = np.array(Y1)
        X2 = np.array(X2)
        Y2 = np.array(Y2)
        Delta = np.array(Delta)
        V = np.array(V)

        # 1. Filtraggio dei dati non nulli (come in MATLAB: I1 ~= 0)
        indici_non_zeri = np.where(X1 != 0)
        
        X1_NZ = X1[indici_non_zeri]
        X2_NZ = X2[indici_non_zeri]
        Y1_NZ = Y1[indici_non_zeri]
        Y2_NZ = Y2[indici_non_zeri]
        Delta_NZ = Delta[indici_non_zeri]
        V_NZ = V[indici_non_zeri]
        
        if len(X1_NZ) == 0:
            no_data_label = QLabel("No valide input data for the analysis.")
            layout.addWidget(no_data_label, 0, 0, 1, 2) # Aggiunto su tutta la riga
            return

        # ------------------------------------------------------------------
        # Calcoli intermedi (Ampiezze e Fasi)
        # ------------------------------------------------------------------
        R1_est = 2.0 * np.sqrt(X1_NZ**2 + Y1_NZ**2)
        R2_est = 2.0 * np.sqrt(X2_NZ**2 + Y2_NZ**2)
        Theta1_rad = np.arctan2(Y1_NZ, X1_NZ)
        Theta2_rad = np.arctan2(Y2_NZ, X2_NZ)
        Theta1_deg = np.degrees(Theta1_rad)
        Theta2_deg = np.degrees(Theta2_rad)
        
        # ------------------------------------------------------------------
        # Figure 1: I/Q (Dati filtrati)
        # ------------------------------------------------------------------
        fig1 = Figure(figsize=(6,3))
        fig1.tight_layout(pad=3.0)
        ax1 = fig1.subplots()
        ax1.plot(X1_NZ, label='X1'); ax1.plot(X2_NZ, label='X2'); ax1.plot(Y1_NZ, label='Y1'); ax1.plot(Y2_NZ, label='Y2')
        ax1.set_title('X1, X2, Y1, Y2'); ax1.legend()
        canvas1 = FigureCanvas(fig1)
        layout.addWidget(canvas1, 0, 0) # Posizione: Riga 0, Colonna 0

        # ------------------------------------------------------------------
        # Figure 2: Ampiezze e Fasi (side-by-side)
        # ------------------------------------------------------------------
        fig2 = Figure(figsize=(8,3))
        fig2.tight_layout(pad=3.0)
        axA = fig2.add_subplot(1, 2, 1)
        axP = fig2.add_subplot(1, 2, 2)
        
        axA.plot(R1_est, label='R1'); axA.plot(R2_est, label='R2'); 
        axA.set_title('Amplitude'); axA.legend()
        
        axP.plot(Theta1_deg, label=r'$\theta_1$'); axP.plot(Theta2_deg, label=r'$\theta_2$'); 
        axP.set_title('Phase (deg)'); axP.legend()
            
        canvas2 = FigureCanvas(fig2)
        layout.addWidget(canvas2, 0, 1) # Posizione: Riga 0, Colonna 1

        # ------------------------------------------------------------------
        # Figure 3: Delta e V (con delta_0 calcolato)
        # ------------------------------------------------------------------
        fig3 = Figure(figsize=(6,4))
        fig3.tight_layout(pad=3.0)
        ax31 = fig3.add_subplot(2, 1, 1)
        ax32 = fig3.add_subplot(2, 1, 2)
        
        ax31.plot(np.degrees(Delta_NZ), label=r'$\delta$');
        ax31.set_ylabel('$\delta$ (deg)'); ax31.legend()
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
        sample_interval = 0.051 
        freq_off = np.fft.fftfreq(len(delta_off), d=sample_interval) 
        
        delta_on = Delta_NZ[len_half:]
        tf_on = np.fft.fft(delta_on)
        freq_on = np.fft.fftfreq(len(delta_on), d=sample_interval) 
        
        ax4.plot(freq_off[:len_half], np.abs(tf_off[:len_half]), label='PID OFF')
        ax4.plot(freq_on[:len_half], np.abs(tf_on[:len_half]), label='PID ON')
        # Limita l'asse X da 0 alla frequenza di Nyquist
        max_freq = 1.0 / (2.0 * sample_interval)
        ax4.set_xlim(0, max_freq/2)
        ax4.set_xlabel('Frequencies (Hz)')
        ax4.set_title('FFT of Delta');
        ax4.legend()
        
        canvas4 = FigureCanvas(fig4)
        layout.addWidget(canvas4, 1, 1) # Posizione: Riga 1, Colonna 1


class MonitorWindow(QWidget):
    closed = pyqtSignal()
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Monitor V+ / V-")
        self.setWindowFlags(Qt.Window | Qt.WindowStaysOnTopHint)

        # Aumento dimensione finestra perché serve spazio per il grafico
        self.setFixedSize(500, 400)

        layout = QGridLayout()
        self.setLayout(layout)

        # --- CAMPi V+ ---
        layout.addWidget(QLabel("V+"), 0, 0)
        self.vplus_field = QLineEdit("0.0")
        self.vplus_field.setReadOnly(True)
        self.vplus_field.setStyleSheet("background-color: #f0f0f0; font-size: 30pt;")
        layout.addWidget(self.vplus_field, 0, 1)

        # --- CAMPI V- ---
        layout.addWidget(QLabel("V-"), 1, 0)
        self.vminus_field = QLineEdit("0.0")
        self.vminus_field.setReadOnly(True)
        self.vminus_field.setStyleSheet("background-color: #f0f0f0; font-size: 30pt;")
        layout.addWidget(self.vminus_field, 1, 1)

        # -----------------------------------------------------
        #            ***  SEZIONE OSCILLOSCOPIO ***
        # -----------------------------------------------------
        self.osc_fig = Figure(figsize=(5, 3))
        self.osc_ax = self.osc_fig.add_subplot(111)
        self.osc_canvas = FigureCanvas(self.osc_fig)

        # BUFFER DATI
        self.buffer_size = 250
        self.x = np.arange(self.buffer_size)
        self.z = np.zeros(self.buffer_size)

        self.osc_line, = self.osc_ax.plot(self.x, self.z, 'r-')

        self.osc_ax.set_title("Oscilloscope Z")
        self.osc_ax.set_xlabel("Samples")
        self.osc_ax.set_ylabel("Z raw")
        self.osc_ax.set_ylim(0, 1023)
        self.osc_ax.grid(True)

        # Aggiungo il grafico alla finestra
        layout.addWidget(self.osc_canvas, 2, 0, 1, 2)

    def update_plot(self, value):
        self.z = np.roll(self.z, -1)
        self.z[-1] = value
    
        self.osc_line.set_ydata(self.z)
    
        # Controllo se devo aggiornare l'asse Y
        current_ylim = self.osc_ax.get_ylim()
        if value > current_ylim[1]:
            self.osc_ax.set_ylim(0, value*1.1)  # 10% di margine
    
        self.osc_canvas.draw()


    def closeEvent(self, event):
        self.closed.emit()
        super().closeEvent(event)


# ---------------- Main GUI window ----------------
class MainWindow(QWidget):
    # Segnali per comunicare dal thread di lavoro al thread GUI
    log_signal = pyqtSignal(str)
    update_plots_signal = pyqtSignal(object, object, object, object, object, object)
    acquisition_finished_signal = pyqtSignal(object, bool, bool, str) # data (dict), doSave, doAnalyze, filename
    
    update_monitor_fields_signal = pyqtSignal(float, float, float)
            
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
        
        # Percorso del file di configurazione (AGGIUNGI QUESTE 2 RIGHE)
        self.config_file = Path("pid_config.json")
        

        self._build_ui()
        self._connect_signals()
        self.load_parameters() # Carica i parametri all'avvio (AGGIUNGI QUESTA RIGA)
        
        self.pid_combo = QComboBox()
        self.pid_combo.addItems(["ON", "OFF"])
        self.pid_combo.currentIndexChanged.connect(self.toggle_pid)
        
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["OFF", "SinWave", "TTL"])
        self.mode_combo.currentIndexChanged.connect(self.toggle_pid)
        
        # --- Oscilloscope Plot Initialization ---
        self.buffer_size = 100 # Numero di punti da visualizzare
        self.z_data = np.zeros(self.buffer_size)
        self.x_data = np.arange(0, self.buffer_size)
        
        # Inizializzazione della figura e del canvas per l'oscilloscopio
        self.osc_fig = Figure(figsize=(5, 4), dpi=100)
        self.osc_canvas = FigureCanvas(self.osc_fig)
        self.osc_ax = self.osc_fig.add_subplot(111)
        self.osc_ax.set_title("Oscilloscope (Z value)")
        self.osc_ax.set_xlabel("Sample Index")
        self.osc_ax.set_ylabel("Z Value (raw_osc)")
        self.osc_ax.set_ylim(0, 1024) # Range tipico per analogRead a 10 bit
        self.osc_ax.grid(True)
        
        # Linea iniziale del plot
        self.osc_line, = self.osc_ax.plot(self.x_data, self.z_data, 'r-')
        self.osc_fig.tight_layout()

    def change_mode(self, idx):
        if idx == 0:
            self.simple_cmd("TTL OFF")
            self.simple_cmd("SET 1 0")   # FreqRef OFF
            self.log("FreqRef OFF (TTL OFF, SET 1 0)")
        
        elif idx == 1:
            self.simple_cmd("TTL OFF")
            self.simple_cmd("SET 1 0.5") # FreqRef ON
            self.log("FreqRef ON (TTL OFF, SET 1 0.5)")
        
        elif idx == 2:
            self.simple_cmd("SET 1 0")   # FreqRef OFF
            self.simple_cmd("TTL ON")
            self.log("TTL ON (FreqRef OFF, TTL ON)")
    
    def toggle_pid(self, idx):
        if idx == 0:
            self.simple_cmd("PID ON")
            self.log("PID ON")
        elif idx == 1:
            self.simple_cmd("PID OFF")
            self.log("PID OFF")


    def handle_monitor_block(self, lines):
        """
        lines = lista di stringhe tipo:
        ["M,0.1234,0.5678,1024", "M,...", ...]
        """
    
        vp_values = []
        vm_values = []
        z_values = []
    
        for line in lines:
            parts = line.strip().split(',')
            if len(parts) == 4 and parts[0] == 'M':
                try:
                    vp_values.append(float(parts[1]))
                    vm_values.append(float(parts[2]))
                    z_values.append(float(parts[3]))
                except ValueError:
                    pass
    
        # Aggiorna campi V+ e V- con l'ultimo valore ricevuto
        if vp_values:
            self.monitor_window.vplus_field.setText(f"{vp_values[-1]:.3f}")
            self.monitor_window.vminus_field.setText(f"{vm_values[-1]:.3f}")
    
        # Aggiorna il plot in modo efficiente
        if z_values:
            n = len(z_values)
            buffer_size = len(self.monitor_window.z)
    
            # Sposta i dati vecchi a sinistra e inserisce i nuovi
            if n >= buffer_size:
                self.monitor_window.z[:] = z_values[-buffer_size:]
            else:
                self.monitor_window.z = np.roll(self.monitor_window.z, -n)
                self.monitor_window.z[-n:] = z_values
    
            # Aggiorna la linea senza ridisegnare tutto l'asse
            self.monitor_window.osc_line.set_ydata(self.monitor_window.z)
    
            # Adatta l'asse Y se serve
            current_ylim = self.monitor_window.osc_ax.get_ylim()
            max_val = max(self.monitor_window.z.max(), current_ylim[1])
            if max_val > current_ylim[1]:
                self.monitor_window.osc_ax.set_ylim(0, max_val * 1.1)
    
            # Disegna velocemente
            self.monitor_window.osc_canvas.draw()

        
    def update_monitor_plot(self, new_values):
        for value in new_values:
            self.monitor_window.z = np.roll(self.monitor_window.z, -1)
            self.monitor_window.z[-1] = value
    
        self.monitor_window.osc_line.set_ydata(self.monitor_window.z)
    
        # Aggiorna asse Y se necessario
        current_ylim = self.monitor_window.osc_ax.get_ylim()
        max_value = max(new_values)
        if max_value > current_ylim[1]:
            self.monitor_window.osc_ax.set_ylim(0, max_value * 1.1)
    
        self.monitor_window.osc_canvas.draw()

    
    def update_monitor_fields(self, vplus, vminus):
        self.monitor_window.vplus_field.setText(f"{vplus:.3f}")
        self.monitor_window.vminus_field.setText(f"{vminus:.3f}")
        self.monitor_window.vplus_field.repaint()
        self.monitor_window.vminus_field.repaint()
        
    def update_oscilloscope_plot(self, z_value):
        # Sposta tutti i dati a sinistra
        self.z_data = np.roll(self.z_data, -1)
        # Inserisce il nuovo valore all'ultima posizione
        self.z_data[-1] = z_value
        
        # Aggiorna i dati della linea
        self.osc_line.set_ydata(self.z_data)
        
        # Ricalcola i limiti dell'asse Y se necessario (ad esempio, se il valore Z supera 1024)
        # Se il valore massimo À maggiore del limite attuale, lo aggiorna
        current_max = self.osc_ax.get_ylim()[1]
        data_max = np.max(self.z_data)
        if data_max > current_max:
            self.osc_ax.set_ylim(0, data_max * 1.1) # Aggiunge un 10% di margine
            self.osc_fig.canvas.draw() # Disegna tutto l'asse
        else:
            # Aggiorna solo la linea per maggiore velocit?
            self.osc_fig.canvas.restore_region(self.osc_background)
            self.osc_ax.draw_artist(self.osc_line)
            self.osc_fig.canvas.blit(self.osc_ax.bbox)
        
    def handle_monitor_closed(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.write(b"MONITOR OFF\n")
                print("DEBUG: MONITOR OFF sent")
        except Exception as e:
            print("ERROR sending MONITOR OFF:", e)
            
    def send_num_samples(self):
        if self.ser is None:
            self.log("WARNING: Serial not connected")
            return

        N = self.samples_spin.value()
        cmd = f"SET N {N}"

        try:
            self.ser.write((cmd + "\n").encode())
            self.log(f"Inviato: {cmd}")
        except Exception as e:
            self.log(f"Errore SET N: {e}")

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
        self.connect_btn = QPushButton("Connected"); self.disconnect_btn = QPushButton("Disconnect")
        self.disconnect_btn.setEnabled(False)
        cg.addWidget(self.connect_btn); cg.addWidget(self.disconnect_btn)
        self.status_label = QLabel("Status: Disconnected"); self.status_label.setStyleSheet("color:red")
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
        self.setP_btn = QPushButton("SET P")
        self.setI_btn = QPushButton("SET I")
        self.setD_btn = QPushButton("SET D")
        self.setO_btn = QPushButton("SET O")
        self.setA_btn = QPushButton("SET A")
        self.setT_btn = QPushButton("SET T")
        for w in [self.setP_btn, 
                  self.setI_btn, 
                  self.setD_btn, 
                  self.setO_btn, 
                  self.setA_btn, 
                  self.setT_btn]:
            btn_row.addWidget(w)
        pg.addLayout(btn_row, 2, 0, 1, 6)
        
        # Combined row: Save/Load, PID, FreqRef, Monitor Button
        combined_row = QHBoxLayout()
        self.save_params_btn = QPushButton("Save parms.")
        self.load_params_btn = QPushButton("Load parms.")
        self.send_all_params_btn = QPushButton("Send all parms.")
        
        # PID mode selector (ON, OFF)
        self.pid_combo = QComboBox()
        self.pid_combo.addItems(["ON", "OFF"])
        self.pid_combo.currentIndexChanged.connect(self.toggle_pid)
        
        # Add a title "PID" above the pid_combo
        pid_box = QVBoxLayout()
        pid_label = QLabel("PID")
        pid_label.setAlignment(Qt.AlignHCenter)
        
        pid_box.addWidget(pid_label)
        pid_box.addWidget(self.pid_combo)
        
        self.pid_widget = QWidget()
        self.pid_widget.setLayout(pid_box)
        

        # New mode selector (OFF, SinWave, TTL)
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["OFF", "SinWave", "TTL"])
        self.mode_combo.currentIndexChanged.connect(self.change_mode)
        # Add a title "FreqRef" above the mode_combo
        freqref_box = QVBoxLayout()
        freqref_label = QLabel("FreqRef")
        freqref_label.setAlignment(Qt.AlignHCenter)
        #freqref_label.setStyleSheet("font-weight: bold;")
        
        freqref_box.addWidget(freqref_label)
        freqref_box.addWidget(self.mode_combo)
        
        self.freqref_widget = QWidget()
        self.freqref_widget.setLayout(freqref_box)

        # NEW ? Monitor open button
        self.monitor_btn = QPushButton("Monitor V+ V-")
        self.monitor_btn.clicked.connect(self.show_monitor_window)
        
        # Add widgets
        for w in [
            self.save_params_btn,
            self.load_params_btn,
            self.send_all_params_btn,
            self.pid_widget,
            self.freqref_widget, 
            self.monitor_btn   # NEW
        ]:
            combined_row.addWidget(w)
            
        
        
        pg.addLayout(combined_row, 3, 0, 1, 6)
        
        left_v.addWidget(pid_group)


        # --- PGA panel ---
        pga_group = QGroupBox("Programmable Gain Amplifier (PGA)")
        pga_l = QGridLayout(pga_group)
        pga_l.addWidget(QLabel("CS"), 0, 0)
        self.cs_combo = QComboBox(); self.cs_combo.addItems(['1','2','3','4'])
        pga_l.addWidget(self.cs_combo, 0, 1)
        
        # pga_l.addWidget(QLabel("Gain"), 0, 2)
        # self.preset_combo = QComboBox(); self.preset_combo.addItems([str(i) for i in range(1,10)])
        # pga_l.addWidget(self.preset_combo, 0, 3)
        pga_l.addWidget(QLabel("Gain"), 0, 2)
        self.preset_combo = QComboBox()
        display_values = ["0.25", "1", "10", "120"]
        for i, text in enumerate(display_values, start=1):
            self.preset_combo.addItem(text, i)  # text = mostrato, i = valore interno
        pga_l.addWidget(self.preset_combo, 0, 3)  # aggiungi il combobox alla griglia
        
        # pga_l.addWidget(QLabel("Trim"), 1, 0)
        # self.chan_combo = QComboBox(); self.chan_combo.addItems([f"{i:02d}" for i in range(1,17)])
        # pga_l.addWidget(self.chan_combo, 1, 1)
        
        pga_l.addWidget(QLabel("Trim"), 1, 0)
        self.chan_combo = QComboBox()
        display_values = [
            "0", "1.3", "2.5", "3.8", "4.9", "6.1", "7.3", "8.4",
            "10.6", "11.7", "12.7", "13.7", "14.7", "15.7", "16.7", "17.6"
        ]
        for i, text in enumerate(display_values, start=1):
            self.chan_combo.addItem(text, i)  # text = mostrato, i = valore interno
        pga_l.addWidget(self.chan_combo, 1, 1)
        
        
        pga_l.addWidget(QLabel("Polarity"), 1, 2)
        self.pol_combo = QComboBox(); self.pol_combo.addItems(['P','N'])
        pga_l.addWidget(self.pol_combo, 1, 3)
        pga_l.addWidget(QLabel("Enable"), 2, 0)
        self.en_combo = QComboBox(); self.en_combo.addItems(['0','1'])
        pga_l.addWidget(self.en_combo, 2, 1)
        self.send_pga_btn = QPushButton("Send PGA Commands")
        pga_l.addWidget(self.send_pga_btn, 2, 2, 1, 2)
        # pga command label
        self.pga_cmd_label = QLabel("Command: -")
        pga_l.addWidget(self.pga_cmd_label, 3, 0, 1, 4)
        left_v.addWidget(pga_group)
        
        monitor_group = QGroupBox("Monitor V+ V-")
        monitor_layout = QGridLayout(monitor_group)

        # Pulsante Toggle per avviare/fermare il monitoraggio
        self.monitor_toggle_btn = QPushButton("MONITOR OFF")
        self.monitor_toggle_btn.setCheckable(True)  # Lo rende un pulsante "toggle"
        self.monitor_toggle_btn.setChecked(False)
        monitor_layout.addWidget(self.monitor_toggle_btn, 0, 0, 1, 2) # Riga 0, occupa 2 colonne

        # Campo per V+
        monitor_layout.addWidget(QLabel("V+"), 1, 0)
        self.vplus_field = QLineEdit("0.0")
        self.vplus_field.setReadOnly(True)
        self.vplus_field.setStyleSheet("background-color: #f0f0f0; font-size: 40pt;") # Stile per indicare che è di sola lettura
        monitor_layout.addWidget(self.vplus_field, 1, 1)

        # Campo per V-
        monitor_layout.addWidget(QLabel("V-"), 2, 0)
        self.vminus_field = QLineEdit("0.0")
        self.vminus_field.setReadOnly(True)
        self.vminus_field.setStyleSheet("background-color: #f0f0f0; font-size: 40pt;")
        monitor_layout.addWidget(self.vminus_field, 2, 1)

        #left_v.addWidget(monitor_group)
        self.monitor_group = monitor_group
        self.monitor_group.hide()  # ← il pannello parte nascosto
        
        #self.open_monitor_btn = QPushButton("Monitor V+ V-")
        #left_v.addWidget(self.open_monitor_btn)
        #self.open_monitor_btn.clicked.connect(self.show_monitor_window)

        self.monitor_window = MonitorWindow()  # istanza della finestra popup
        self.monitor_window.closed.connect(self.handle_monitor_closed)

        # --- Acquisition panel ---
        acq_group = QGroupBox("Data Collection")
        acq_l = QGridLayout(acq_group)
        acq_l.addWidget(QLabel("Num. of repetitions"), 0, 0)
        self.times_spin = QSpinBox(); self.times_spin.setRange(1, 100); self.times_spin.setValue(2)
        acq_l.addWidget(self.times_spin, 1, 0)
        self.times_spin.setFixedWidth(70)
        acq_l.addWidget(QLabel("Num. coll. points"), 0, 1)
        self.samples_spin = QSpinBox(); self.samples_spin.setRange(1, 4000); self.samples_spin.setValue(200)
        self.samples_spin.valueChanged.connect(self.send_num_samples)
        acq_l.addWidget(self.samples_spin, 1, 1)
        self.samples_spin.setFixedWidth(70)
        acq_l.addWidget(QLabel("Filename"), 0, 2)
        self.filename_edit = QLineEdit("./Test_PID_ON_OFF")
        acq_l.addWidget(self.filename_edit, 1, 2)
        self.filename_edit.setFixedWidth(250)
        self.save_check = QCheckBox("Save data"); self.save_check.setChecked(True)
        self.analyze_check = QCheckBox("analysis after collection")
        acq_l.addWidget(self.save_check, 2, 0); acq_l.addWidget(self.analyze_check, 2, 1)
        

        # acquisition buttons
        self.start_acq_btn = QPushButton("Start Acquisizione PID ON/OFF")
        self.stop_acq_btn = QPushButton("Stop"); self.stop_acq_btn.setEnabled(False)
        self.calibrate_btn = QPushButton("CALIBRATE")
        self.print_btn = QPushButton("PRINT (one block)")
        self.reset_btn = QPushButton("RESET")
        self.analyze_btn = QPushButton("Analysis data"); self.analyze_btn.setEnabled(False)
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

        # Canvas 1: X1,X2,Y1,Y2
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
        
        
    def show_monitor_window(self):
        if not self.monitor_window.isVisible():
            # 1) show window
            self.monitor_window.show()
            self.monitor_window.raise_()
            self.monitor_window.activateWindow()
    
            # 2) tell Arduino to start sending monitor data
            try:
                if self.ser and self.ser.is_open:
                    self.ser.write(b"MONITOR ON\n")
                    print("DEBUG: MONITOR ON sent")
            except Exception as e:
                print("ERROR sending MONITOR ON:", e)
   
            
    def handle_monitor_line(self, line):
        parts = line.strip().split(',')
        if len(parts) >= 4 and parts[0] == 'M':
            try:
                vplus = float(parts[1])
                vminus = float(parts[2])
                z_value = float(parts[3])   # <--- QUI VIENE DEFINITO
                
                # Aggiorno campi V+ e V-
                self.update_monitor_fields_signal.emit(vplus, vminus, z_value)
                
                # Aggiorno il grafico
                if self.monitor_window is not None:
                    self.monitor_window.update_plot(z_value)
    
            except ValueError:
                pass


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
        self.send_pga_btn.clicked.connect(self.send_pga)
        
        self.print_btn.clicked.connect(lambda: self.threadpool.start(Worker(self.print_once_task)))
        self.calibrate_btn.clicked.connect(lambda: self.threadpool.start(Worker(self.calibrate_task)))
        self.start_acq_btn.clicked.connect(lambda: self.threadpool.start(Worker(self.start_acquisition_task)))
        self.stop_acq_btn.clicked.connect(self.stop_acquisition_request)
        self.reset_btn.clicked.connect(lambda: self.simple_cmd('RESET'))
        self.analyze_btn.clicked.connect(self.run_analysis_now) # Non À un'operazione lunga, pu⁄ essere nel thread GUI

        self.log_signal.connect(self.log)
        self.update_plots_signal.connect(self.update_print_plots)
        self.acquisition_finished_signal.connect(self.handle_acquisition_finished)
        
        self.pid_combo.currentIndexChanged.connect(self.toggle_pid)    # PID ON/OFF
        self.mode_combo.currentIndexChanged.connect(self.change_mode)  # OFF / SinWave / TTL

        
        self.send_pga_btn.clicked.connect(self.send_pga)
        
        self.monitor_btn.clicked.connect(self.show_monitor_window)
        self.monitor_toggle_btn.toggled.connect(self.toggle_monitor)
        
        self.save_params_btn.clicked.connect(self.save_parameters)
        self.load_params_btn.clicked.connect(self.load_parameters)
        self.send_all_params_btn.clicked.connect(lambda: self.threadpool.start(Worker(self.send_all_parameters_task)))
        

        if self.reader:
            #self.reader.monitor_line_received.connect(self.handle_monitor_line)
            self.reader.monitor_block_received.connect(self.handle_monitor_block)
            self.update_monitor_fields_signal.connect(self.update_monitor_fields)
        
    


    # ---------------- utility: logging ----------------
    def log(self, txt):
        t = datetime.now().strftime("%H:%M:%S")
        s = f"[{t}] {txt}"
        # append direttamente, poich» questo metodo À chiamato nel thread GUI (tramite segnale)
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
            
            self.reader.monitor_block_received.connect(self.handle_monitor_block)
            self.update_monitor_fields_signal.connect(self.update_monitor_fields)
            
            self.reader.start()
            self.log(f"Connection success: {port} @ {baud}")
            self.status_label.setText(f"Status: Connected ({port} @ {baud})")
            self.status_label.setStyleSheet("color:green")
            self.connect_btn.setEnabled(False)
            self.disconnect_btn.setEnabled(True)
        except SerialException as e:
            self.log(f"Error connection: {e}")
            self.status_label.setText("Status: Error connection")
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
        
    def handle_pid_on(self):
        """Gestisce il click su 'PID ON', invia il comando e aggiorna lo stile."""
        self.simple_cmd('PID ON')
        self.pid_on_btn.setStyleSheet(self.style_pid_on_active)
        self.pid_off_btn.setStyleSheet(self.style_pid_default)

    def handle_pid_off(self):
        """Gestisce il click su 'PID OFF', invia il comando e aggiorna lo stile."""
        self.simple_cmd('PID OFF')
        self.pid_off_btn.setStyleSheet(self.style_pid_off_active)
        self.pid_on_btn.setStyleSheet(self.style_pid_default)

    # ---------------- simple_cmd ----------------
    def simple_cmd(self, cmd, wait_sec=0.05):
        if not self.ser:
            self.log("Error: not connected.")
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
                self.log(f"Sent command: {cmd}")
        except Exception as e:
            self.log(f"Error command: {e}")

    # ---------------- set param ----------------
    def set_param(self, param):
        if not self.ser:
            self.log("Error: not connected.")
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
        elif param == 'N':
            val = self.samples_spin.value()
        else:
            self.log("Parameters unknown")
            return
        
        if val is not None:
            cmd = f"SET {param} {val}"
            self.log(f"Sending: {cmd}")
            self.simple_cmd(cmd)      
            
        if val is not None:
            cmd = f"SET {param} {val}"
            self.log(f"Sending: {cmd}")
            self.simple_cmd(cmd)
            self.save_parameters() # Salva i parametri dopo ogni SET (AGGIUNGI QUESTA RIGA)
            
        # ---------------- Salva/Carica/Invia Parametri ----------------
    def get_parameters_from_gui(self):
        """Ritorna un dizionario con i parametri PID e SET correnti dalla GUI."""
        return {
            'Kp': self.kp_field.value(),
            'Ki': self.ki_field.value(),
            'Kd': self.kd_field.value(),
            'Offset': self.off_field.value(),
            'Amplitude': self.amp_field.value(),
            'SampleInterval': self.t_field.value(),
            'NumSamples': self.samples_spin.value()
        }

    def set_parameters_to_gui(self, params):
        """Imposta i parametri PID e SET nella GUI dal dizionario."""
        self.kp_field.setValue(params.get('Kp', self.kp_field.value()))
        self.ki_field.setValue(params.get('Ki', self.ki_field.value()))
        self.kd_field.setValue(params.get('Kd', self.kd_field.value()))
        self.off_field.setValue(params.get('Offset', self.off_field.value()))
        self.amp_field.setValue(params.get('Amplitude', self.amp_field.value()))
        self.t_field.setValue(params.get('SampleInterval', self.t_field.value()))
        self.samples_spin.setValue(params.get('NumSamples', self.samples_spin.value()))

    def save_parameters(self):
        """Salva i parametri correnti in un file JSON."""
        params = self.get_parameters_from_gui()
        try:
            with open(self.config_file, 'w') as f:
                json.dump(params, f, indent=4)
            self.log(f"Parameters saved in {self.config_file}")
        except Exception as e:
            self.log(f"Error during data saving: {e}")

    def load_parameters(self):
        """Carica i parametri da un file JSON e aggiorna la GUI."""
        if not self.config_file.exists():
            self.log("Json file not found. Default parameters used.")
            return
        try:
            with open(self.config_file, 'r') as f:
                params = json.load(f)
            self.set_parameters_to_gui(params)
            self.log(f"Loaded parameters from {self.config_file}")
        except Exception as e:
            self.log(f"Error during parameters loading: {e}")

    def send_all_parameters_task(self):
        """Invia tutti i parametri PID e SET all'Arduino (eseguito in un thread di lavoro)."""
        if not self.ser:
            self.log_from_thread("Error not connected. Parameters not found")
            return
        
        self.log_from_thread("Sending all SET parameters to Arduino...")
        
        params = self.get_parameters_from_gui()
        
        # Sequenza di invio dei comandi
        commands = [
            ('P', params['Kp']),
            ('I', params['Ki']),
            ('D', params['Kd']),
            ('O', params['Offset']),
            ('A', params['Amplitude']),
            ('T', params['SampleInterval']),
            ('N', self.samples_spin.value()) # Aggiunto per inviare anche NumSamples
        ]
        
        for param_char, value in commands:
            cmd = f"SET {param_char} {value}"
            self.log_from_thread(f"Inviando: {cmd}")
            # Usiamo simple_cmd che gestisce l'invio seriale e la lettura della risposta
            self.simple_cmd(cmd, wait_sec=0.1) 
            
        self.log_from_thread("All parameters send.")

    def toggle_monitor(self, checked):
        if checked:
            self.is_monitoring = True
            self.monitor_toggle_btn.setText("MONITOR ON")
            self.monitor_toggle_btn.setStyleSheet("background-color: #28a745; color: white;")
            
            # Mostra finestra popup
            self.monitor_window.show()
            self.monitor_window.raise_()  # Porta in primo piano
            
            self.simple_cmd('MONITOR ON')
            self.log("Monitoring V+/V- activated.")
        else:
            self.is_monitoring = False
            self.monitor_toggle_btn.setText("MONITOR OFF")
            self.monitor_toggle_btn.setStyleSheet("")
            
            # Nascondi finestra popup
            self.monitor_window.hide()
            
            self.simple_cmd('MONITOR OFF')
            self.log("Monitoring V+/V- deactivated.")



    def process_monitor_line(self, line):
        try:
            # Rimuove "M," e divide
            parts = line.split(',')
            if len(parts) != 3:
                self.log(f"Monitor format not valid {line}")
                return
    
            _, vplus, vminus = parts
            vplus = float(vplus)
            vminus = float(vminus)
    
            # Aggiorna la GUI tramite segnale thread-safe
            self.update_monitor_fields_signal.emit(vplus, vminus)
    
        except Exception as e:
            self.log(f"Error in process_monitor_line: {e}")
    
    
        except (ValueError, IndexError):
            pass
        

    # ---------------- send PGA ----------------
    def send_pga(self):
        cs = self.cs_combo.currentText()
        preset = self.preset_combo.currentData()
        chan = f"{int(self.chan_combo.currentData()):02d}"        
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
            self.log_from_thread("Error: not connected.")
            return
        num_points = int(self.samples_spin.value())
        self.log_from_thread(f"PRINT command sent. Waiting for {num_points} lines...")
        
        # Svuota la coda prima di inviare il comando
        with self.line_queue.mutex:
            self.line_queue.queue.clear()
            
        try:
            self.ser.write(b'PRINT\n')
        except Exception as e:
            self.log_from_thread(f"PRINT error: {e}")
            return
            
        timeout_total = max(5.0, 0.02 * num_points)
        lines = self.read_n_lines(num_points, sec_total=timeout_total)
        
        if len(lines) < num_points:
            self.log_from_thread(f"Attention: only {len(lines)}/{num_points} lines received (timeout {timeout_total}s).")
            
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
                self.log_from_thread(f"Non-numeric line ignored: '{line}'"); continue
            if len(vals) >= 6:
                I1[idx] = vals[0]; I2[idx] = vals[1]; Q1[idx] = vals[2]; Q2[idx] = vals[3]; Delta[idx] = vals[4]; V[idx] = vals[5]
                idx += 1
            else:
                self.log_from_thread(f"Riga con formato sbagliato ignorata: '{line}'")
                
        if idx == 0:
            self.log_from_thread("No valid data received from PRINT.")
            # Aggiorna la GUI per riabilitare il pulsante di analisi
            QtCore.QMetaObject.invokeMethod(self.analyze_btn, "setEnabled", QtCore.Qt.QueuedConnection, QtCore.Q_ARG(bool, False))
            return
            
        I1 = I1[:idx]; I2 = I2[:idx]; Q1 = Q1[:idx]; Q2 = Q2[:idx]; Delta = Delta[:idx]; V = V[:idx]
        self.latestData = {'I1': I1, 'I2': I2, 'Q1': Q1, 'Q2': Q2, 'Delta': Delta, 'V': V, 'NumSamples': idx}

        # Emette il segnale per l'aggiornamento della GUI
        self.update_plots_signal.emit(I1, I2, Q1, Q2, Delta, V)
        self.log_from_thread("PRINT commands received and graphics update signal issued.")


    # ---------------- update plots in GUI (chiamato dal segnale) ----------------
    def update_print_plots(self, I1, Q1, I2, Q2, Delta, V):
        # I/Q
        self.ax_IQ.clear()
        self.ax_IQ.plot(I1, label='X1'); self.ax_IQ.plot(I2, label='X2'); self.ax_IQ.plot(Q1, label='Y1'); self.ax_IQ.plot(Q2, label='Y2')
        #self.ax_IQ.set_title('X1, Y1, X2, Y2');
        self.ax_IQ.set_xlabel('Sample'); 
        self.ax_IQ.set_ylabel('Amplitude');
        self.ax_IQ.legend(fontsize=8)
        self.fig_IQ.tight_layout() 
        self.canvas_IQ.draw()

        # Delta (second canvas)
        self.ax_Delta.clear()
        self.ax_Delta.plot(np.degrees(Delta), label='$\delta$ (deg)')
        #self.ax_Delta.set_title('$\delta$ (deg)')
        self.ax_Delta.set_xlabel('Sample'); self.ax_Delta.set_ylabel('$\delta$ (deg)'); #self.ax_Delta.legend()
        self.fig_Delta.tight_layout() 
        self.canvas_Delta.draw()

        # V (third canvas)
        self.ax_V.clear()
        self.ax_V.plot(V, '.-')
        #self.ax_V.set_title('$V_{offset}$')
        self.ax_V.set_xlabel('Sample'); self.ax_V.set_ylabel('$V_{offset}$'); #self.ax_V.legend(['V_off'])
        self.fig_V.tight_layout()
        self.canvas_V.draw()

        self.log("Plots updated.")
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
            self.log_from_thread("Calibration Completed.")
        else:
            self.log_from_thread("Calibration completed due to timeout (may still be in progress on the Arduino side).")

    # ---------------- start acquisition (PID ON/OFF cycles) - Task per QRunnable ----------------
    def start_acquisition_task(self):
        if not self.ser:
            self.log_from_thread("Error: not connected."); return
            
        # Disabilita/Abilita pulsanti nel thread GUI
        QtCore.QMetaObject.invokeMethod(self.start_acq_btn, "setEnabled", QtCore.Qt.QueuedConnection, QtCore.Q_ARG(bool, False))
        QtCore.QMetaObject.invokeMethod(self.stop_acq_btn, "setEnabled", QtCore.Qt.QueuedConnection, QtCore.Q_ARG(bool, True))
        QtCore.QMetaObject.invokeMethod(self.analyze_btn, "setEnabled", QtCore.Qt.QueuedConnection, QtCore.Q_ARG(bool, False))
        
        times = int(self.times_spin.value()); numSamples = int(self.samples_spin.value())
        filename = self.filename_edit.text().strip(); doSave = self.save_check.isChecked(); doAnalyze = self.analyze_check.isChecked()
        self.stop_flag = False
        self.log_from_thread("PID ON/OFF acquisition started.")
        
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
                self.log_from_thread("Acquisition interrupted by the user."); break
                
            if j > switch_point:
                self.simple_cmd("PID ON", wait_sec=0.02); self.log_from_thread(f"Cycle {j}/{times}: PID ON")
            else:
                self.simple_cmd("PID OFF", wait_sec=0.02); self.log_from_thread(f"Cycle {j}/{times}: PID OFF")
                
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
                self.log_from_thread(f"Warning: only {len(lines)}/{numSamples} lines received per cycle {j}..")
                
            for line in lines:
                parts = [p.strip() for p in line.replace(',', ' ').split()]
                try:
                    vals = [float(x) for x in parts]
                except Exception:
                    continue
                if len(vals) >= 6 and write_idx < total_samples:
                    I1[write_idx] = vals[0]; I2[write_idx] = vals[1]; Q1[write_idx] = vals[2]; Q2[write_idx] = vals[3]; Delta[write_idx] = vals[4]; V[write_idx] = vals[5]
                    write_idx += 1
                    
            self.log_from_thread(f"Block {j} completed. Samples written: {write_idx}")
            
        # finish
        self.simple_cmd("PID OFF", wait_sec=0.02)
        
        data = None
        if write_idx == 0:
            self.log_from_thread("No data acquired.")
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
        self.log("STOP request sent.")

    # ---------------- Gestione fine acquisizione (chiamato dal segnale) ----------------
    def handle_acquisition_finished(self, data, doSave, doAnalyze, filename):
        # Questo metodo À eseguito nel thread GUI
        self.start_acq_btn.setEnabled(True)
        self.stop_acq_btn.setEnabled(False)
        
        if data:
            self.latestData = data
            self.analyze_btn.setEnabled(True)
            
            if doSave:
                try:
                    np.savez(filename + ".npz", **self.latestData)
                    self.log(f"Data saved in {filename}.npz")
                except Exception as e:
                    self.log(f"Save error: {e}")
                    
            if doAnalyze:
                self.show_analysis_dialog(self.latestData)
                self.log("Analysis completed.")
        else:
            self.analyze_btn.setEnabled(False)

    # ---------------- run analysis now ----------------
    def run_analysis_now(self):
        if not self.latestData:
            self.log("No data available for analysis."); return
        self.show_analysis_dialog(self.latestData)

    # ---------------- Mostra AnalysisDialog (nel thread GUI) ----------------
    def show_analysis_dialog(self, data):
        # Assicura che l'apertura del dialogo avvenga nel thread GUI
        D = data
        AnalysisDialog(D['I1'], D['I2'], D['Q1'], D['Q2'], D['Delta'], D['V'], parent=self).exec_()


# ---------------- main ----------------
def main():
    app = QApplication(sys.argv)
    
    # -------- ICONA PER EXE (PyInstaller) + modalit? sviluppo --------
    if hasattr(sys, '_MEIPASS'):
        icon_path = os.path.join(sys._MEIPASS, "icont_w_BG.png")
    else:
        icon_path = "icont_w_BG.png"
    app.setWindowIcon(QtGui.QIcon(icon_path))
    # -----------------------------------------------------------------


    w = MainWindow()
    w.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()