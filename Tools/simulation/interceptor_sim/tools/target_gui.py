#!/usr/bin/env python3
"""
Talon Target Drone Interactive Control Panel (PySide6)

Provides real-time interactive control over the simulated Talon target:
- Flight mode switching: Straight Flight, S-Turn (Weave), Orbit
- Directional D-Pad remote control (altitude nudge up/down, heading nudge left/right)
- Speed and altitude adjustment sliders & presets
- Real-time telemetry monitoring

Connects to target_sim.py HTTP API at http://localhost:8000
"""

import argparse
import json
import sys
import urllib.error
import urllib.request
from urllib.parse import urlencode

from PySide6.QtCore import QObject, Qt, QThread, QTimer, Signal
from PySide6.QtGui import QFont, QIcon, QKeySequence, QShortcut
from PySide6.QtWidgets import (
	QApplication,
	QFrame,
	QGridLayout,
	QGroupBox,
	QHBoxLayout,
	QLabel,
	QMainWindow,
	QPushButton,
	QSlider,
	QSpinBox,
	QVBoxLayout,
	QWidget,
)


class TelemetryWorker(QObject):
	"""Worker thread to poll target telemetry and dispatch commands asynchronously."""

	telemetry_received = Signal(dict)
	connection_status = Signal(bool, str)

	def __init__(self, host="127.0.0.1", port=8000):
		super().__init__()
		self.host = host
		self.port = port
		self.base_url = f"http://{host}:{port}"
		self._running = True

	def poll(self):
		if not self._running:
			return
		url = f"{self.base_url}/target"
		try:
			req = urllib.request.Request(url, headers={"User-Agent": "TalonGUI/1.0"})
			with urllib.request.urlopen(req, timeout=0.35) as resp:
				if resp.status == 200:
					data = json.loads(resp.read().decode("utf-8"))
					self.telemetry_received.emit(data)
					self.connection_status.emit(True, "BAĞLANDI")
					return
		except Exception as e:
			self.connection_status.emit(False, f"Bağlantı Aranıyor... ({self.host}:{self.port})")

	def send_command(self, cmd_dict):
		url = f"{self.base_url}/cmd"
		try:
			post_data = json.dumps(cmd_dict).encode("utf-8")
			req = urllib.request.Request(
				url,
				data=post_data,
				headers={"Content-Type": "application/json", "User-Agent": "TalonGUI/1.0"},
				method="POST",
			)
			with urllib.request.urlopen(req, timeout=0.5) as resp:
				pass
		except Exception:
			# Target might still be booting up
			pass


class TargetControlWindow(QMainWindow):
	def __init__(self, host="127.0.0.1", port=8000):
		super().__init__()
		self.host = host
		self.port = port
		self.current_mode = "straight"
		self.target_speed = 25.0
		self.target_alt = 100.0
		self.target_heading = 0.0

		self.setWindowTitle("Talon-1718 Hedef Kontrol Paneli")
		self.setFixedSize(400, 640)
		self.setStyleSheet(self._dark_theme_qss())

		self.worker = TelemetryWorker(host, port)
		self.poll_timer = QTimer(self)
		self.poll_timer.timeout.connect(self.worker.poll)
		self.worker.telemetry_received.connect(self.on_telemetry)
		self.worker.connection_status.connect(self.on_status)

		self._init_ui()
		self._setup_shortcuts()

		self.poll_timer.start(180)  # ~5.5 Hz polling

	def _init_ui(self):
		central = QWidget(self)
		self.setCentralWidget(central)
		main_layout = QVBoxLayout(central)
		main_layout.setContentsMargins(12, 10, 12, 10)
		main_layout.setSpacing(10)

		# 1. Header / Connection Status
		header_card = QFrame()
		header_card.setObjectName("card")
		h_layout = QHBoxLayout(header_card)
		h_layout.setContentsMargins(10, 8, 10, 8)

		self.lbl_status_dot = QLabel("●")
		self.lbl_status_dot.setStyleSheet("color: #e63946; font-size: 16px;")
		self.lbl_status_text = QLabel("Bağlantı Aranıyor...")
		self.lbl_status_text.setStyleSheet("font-weight: bold; color: #a0aec0; font-size: 12px;")

		self.badge_mode = QLabel("DÜZ UÇUŞ")
		self.badge_mode.setObjectName("modeBadge")
		self.badge_mode.setStyleSheet(
			"background-color: #2b6cb0; color: white; border-radius: 4px; "
			"padding: 4px 10px; font-weight: bold; font-size: 11px;"
		)

		h_layout.addWidget(self.lbl_status_dot)
		h_layout.addWidget(self.lbl_status_text)
		h_layout.addStretch()
		h_layout.addWidget(self.badge_mode)
		main_layout.addWidget(header_card)

		# 2. Flight Mode Selection ("şimdi S, şimdi Orbit, Düz Uçuş")
		mode_group = QGroupBox("UÇUŞ DAVRANIŞI (ANLIK MOD)")
		mode_layout = QVBoxLayout(mode_group)
		mode_layout.setSpacing(6)

		btn_row = QHBoxLayout()
		btn_row.setSpacing(6)

		self.btn_straight = QPushButton("✈️ Düz Uçuş")
		self.btn_straight.setToolTip("Hedef drone düz hatta uçar [Kısayol: 1]")
		self.btn_straight.clicked.connect(lambda: self.send_cmd({"mode": "straight"}))

		self.btn_sturn = QPushButton("〰️ S Manevra")
		self.btn_sturn.setToolTip("Hedef drone sağa-sola S çizer (Weave) [Kısayol: 2]")
		self.btn_sturn.clicked.connect(lambda: self.send_cmd({"mode": "s_turn"}))

		self.btn_orbit = QPushButton("🔄 Orbit")
		self.btn_orbit.setToolTip("Hedef drone daire çizer [Kısayol: 3]")
		self.btn_orbit.clicked.connect(lambda: self.send_cmd({"mode": "orbit"}))

		btn_row.addWidget(self.btn_straight)
		btn_row.addWidget(self.btn_sturn)
		btn_row.addWidget(self.btn_orbit)
		mode_layout.addLayout(btn_row)

		# Orbit direction toggle row
		orbit_row = QHBoxLayout()
		orbit_row.setSpacing(6)
		self.btn_orbit_left = QPushButton("⟲ Sola Orbit (CCW)")
		self.btn_orbit_left.clicked.connect(lambda: self.send_cmd({"mode": "orbit", "orbit_dir": "left"}))
		self.btn_orbit_right = QPushButton("⟳ Sağa Orbit (CW)")
		self.btn_orbit_right.clicked.connect(lambda: self.send_cmd({"mode": "orbit", "orbit_dir": "right"}))
		orbit_row.addWidget(self.btn_orbit_left)
		orbit_row.addWidget(self.btn_orbit_right)
		mode_layout.addLayout(orbit_row)

		main_layout.addWidget(mode_group)

		# 3. Directional D-Pad Remote Controller ("sağa sola yukarı aşağı kumanda")
		pad_group = QGroupBox("YÖN VE İRTİFA KUMANDASI")
		pad_layout = QVBoxLayout(pad_group)
		pad_layout.setSpacing(6)

		grid = QGridLayout()
		grid.setSpacing(6)

		self.btn_alt_up = QPushButton("▲ +10m")
		self.btn_alt_up.setToolTip("İrtifayı 10 metre artır [Kısayol: Yukarı Ok]")
		self.btn_alt_up.clicked.connect(lambda: self.send_cmd({"nudge_alt": 10.0}))

		self.btn_alt_down = QPushButton("▼ -10m")
		self.btn_alt_down.setToolTip("İrtifayı 10 metre azalt [Kısayol: Aşağı Ok]")
		self.btn_alt_down.clicked.connect(lambda: self.send_cmd({"nudge_alt": -10.0}))

		self.btn_turn_left = QPushButton("◀ Sola 15°")
		self.btn_turn_left.setToolTip("Rotayı 15° sola çevir [Kısayol: Sol Ok]")
		self.btn_turn_left.clicked.connect(lambda: self.send_cmd({"nudge_yaw": 15.0}))

		self.btn_turn_right = QPushButton("▶ Sağa 15°")
		self.btn_turn_right.setToolTip("Rotayı 15° sağa çevir [Kısayol: Sağ Ok]")
		self.btn_turn_right.clicked.connect(lambda: self.send_cmd({"nudge_yaw": -15.0}))

		self.btn_level = QPushButton("⌖ Düzelt")
		self.btn_level.setToolTip("Düz uçuşa geç ve rotayı sabitle")
		self.btn_level.clicked.connect(lambda: self.send_cmd({"mode": "straight"}))

		grid.addWidget(self.btn_alt_up, 0, 1)
		grid.addWidget(self.btn_turn_left, 1, 0)
		grid.addWidget(self.btn_level, 1, 1)
		grid.addWidget(self.btn_turn_right, 1, 2)
		grid.addWidget(self.btn_alt_down, 2, 1)
		pad_layout.addLayout(grid)

		# Quick turn angles row
		turn_steps = QHBoxLayout()
		turn_steps.setSpacing(4)
		for deg, label in [(-45, "-45°"), (-30, "-30°"), (30, "+30°"), (45, "+45°")]:
			b = QPushButton(label)
			b.setStyleSheet("padding: 4px; font-size: 11px;")
			# deg > 0 is right, but math ENU yaw left is positive, so nudge_yaw is -deg
			b.clicked.connect(lambda checked=False, d=-deg: self.send_cmd({"nudge_yaw": float(d)}))
			turn_steps.addWidget(b)
		pad_layout.addLayout(turn_steps)

		main_layout.addWidget(pad_group)

		# 4. Speed & Altitude Setpoints (Hız ve İrtifa)
		set_group = QGroupBox("HIZ VE İRTİFA AYARI")
		set_layout = QVBoxLayout(set_group)
		set_layout.setSpacing(8)

		# Speed row
		spd_layout = QHBoxLayout()
		lbl_spd_title = QLabel("Hız:")
		lbl_spd_title.setFixedWidth(50)
		self.slider_speed = QSlider(Qt.Horizontal)
		self.slider_speed.setRange(15, 38)
		self.slider_speed.setValue(25)
		self.lbl_speed_val = QLabel("25 m/s")
		self.lbl_speed_val.setFixedWidth(55)
		self.lbl_speed_val.setStyleSheet("font-weight: bold; color: #48bb78;")

		self.slider_speed.valueChanged.connect(self._on_speed_slider_changed)
		self.slider_speed.sliderReleased.connect(self._on_speed_slider_released)

		spd_layout.addWidget(lbl_spd_title)
		spd_layout.addWidget(self.slider_speed)
		spd_layout.addWidget(self.lbl_speed_val)
		set_layout.addLayout(spd_layout)

		# Speed presets
		spd_presets = QHBoxLayout()
		spd_presets.setSpacing(4)
		for spd in [18, 22, 25, 30, 35]:
			b = QPushButton(f"{spd} m/s")
			b.setStyleSheet("padding: 3px; font-size: 10px;")
			b.clicked.connect(lambda checked=False, s=spd: self._set_speed(s))
			spd_presets.addWidget(b)
		set_layout.addLayout(spd_presets)

		# Altitude row
		alt_layout = QHBoxLayout()
		lbl_alt_title = QLabel("İrtifa:")
		lbl_alt_title.setFixedWidth(50)
		self.slider_alt = QSlider(Qt.Horizontal)
		self.slider_alt.setRange(20, 220)
		self.slider_alt.setValue(100)
		self.lbl_alt_val = QLabel("100 m")
		self.lbl_alt_val.setFixedWidth(55)
		self.lbl_alt_val.setStyleSheet("font-weight: bold; color: #4299e1;")

		self.slider_alt.valueChanged.connect(self._on_alt_slider_changed)
		self.slider_alt.sliderReleased.connect(self._on_alt_slider_released)

		alt_layout.addWidget(lbl_alt_title)
		alt_layout.addWidget(self.slider_alt)
		alt_layout.addWidget(self.lbl_alt_val)
		set_layout.addLayout(alt_layout)

		# Altitude presets
		alt_presets = QHBoxLayout()
		alt_presets.setSpacing(4)
		for alt in [40, 70, 100, 130, 160]:
			b = QPushButton(f"{alt} m")
			b.setStyleSheet("padding: 3px; font-size: 10px;")
			b.clicked.connect(lambda checked=False, a=alt: self._set_alt(a))
			alt_presets.addWidget(b)
		set_layout.addLayout(alt_presets)

		main_layout.addWidget(set_group)

		# 5. Live Telemetry Monitor
		telem_group = QGroupBox("ANLIK TELEMETRİ")
		telem_layout = QGridLayout(telem_group)
		telem_layout.setContentsMargins(8, 6, 8, 6)
		telem_layout.setSpacing(4)

		telem_layout.addWidget(QLabel("Yer Hızı:"), 0, 0)
		self.tel_speed = QLabel("-- m/s")
		self.tel_speed.setStyleSheet("font-weight: bold; color: #48bb78;")
		telem_layout.addWidget(self.tel_speed, 0, 1)

		telem_layout.addWidget(QLabel("İrtifa (rel):"), 0, 2)
		self.tel_alt = QLabel("-- m")
		self.tel_alt.setStyleSheet("font-weight: bold; color: #4299e1;")
		telem_layout.addWidget(self.tel_alt, 0, 3)

		telem_layout.addWidget(QLabel("Baş (Yaw):"), 1, 0)
		self.tel_heading = QLabel("--°")
		self.tel_heading.setStyleSheet("font-weight: bold; color: #ecc94b;")
		telem_layout.addWidget(self.tel_heading, 1, 1)

		telem_layout.addWidget(QLabel("Yatma (Roll):"), 1, 2)
		self.tel_roll = QLabel("--°")
		self.tel_roll.setStyleSheet("font-weight: bold; color: #ed8936;")
		telem_layout.addWidget(self.tel_roll, 1, 3)

		main_layout.addWidget(telem_group)

	def _setup_shortcuts(self):
		QShortcut(QKeySequence(Qt.Key_Up), self, lambda: self.send_cmd({"nudge_alt": 10.0}))
		QShortcut(QKeySequence(Qt.Key_Down), self, lambda: self.send_cmd({"nudge_alt": -10.0}))
		QShortcut(QKeySequence(Qt.Key_Left), self, lambda: self.send_cmd({"nudge_yaw": 15.0}))
		QShortcut(QKeySequence(Qt.Key_Right), self, lambda: self.send_cmd({"nudge_yaw": -15.0}))
		QShortcut(QKeySequence("1"), self, lambda: self.send_cmd({"mode": "straight"}))
		QShortcut(QKeySequence("2"), self, lambda: self.send_cmd({"mode": "s_turn"}))
		QShortcut(QKeySequence("3"), self, lambda: self.send_cmd({"mode": "orbit"}))
		QShortcut(QKeySequence("W"), self, lambda: self._nudge_speed(2.0))
		QShortcut(QKeySequence("S"), self, lambda: self._nudge_speed(-2.0))

	def _nudge_speed(self, delta):
		cur = self.slider_speed.value()
		new_v = max(15, min(38, cur + int(delta)))
		self._set_speed(new_v)

	def _set_speed(self, spd):
		self.slider_speed.setValue(int(spd))
		self.lbl_speed_val.setText(f"{spd} m/s")
		self.send_cmd({"speed": float(spd)})

	def _set_alt(self, alt):
		self.slider_alt.setValue(int(alt))
		self.lbl_alt_val.setText(f"{alt} m")
		self.send_cmd({"alt": float(alt)})

	def _on_speed_slider_changed(self, val):
		self.lbl_speed_val.setText(f"{val} m/s")

	def _on_speed_slider_released(self):
		self.send_cmd({"speed": float(self.slider_speed.value())})

	def _on_alt_slider_changed(self, val):
		self.lbl_alt_val.setText(f"{val} m")

	def _on_alt_slider_released(self):
		self.send_cmd({"alt": float(self.slider_alt.value())})

	def send_cmd(self, cmd_dict):
		self.worker.send_command(cmd_dict)

	def on_status(self, connected, text):
		if connected:
			self.lbl_status_dot.setStyleSheet("color: #48bb78; font-size: 16px;")
			self.lbl_status_text.setText("Talon-1718 BAĞLI")
			self.lbl_status_text.setStyleSheet("font-weight: bold; color: #e2e8f0; font-size: 12px;")
		else:
			self.lbl_status_dot.setStyleSheet("color: #e53e3e; font-size: 16px;")
			self.lbl_status_text.setText(text)
			self.lbl_status_text.setStyleSheet("font-weight: bold; color: #a0aec0; font-size: 11px;")

	def on_telemetry(self, data):
		if not data.get("valid", False):
			return

		mode = data.get("mode", "straight")
		self.current_mode = mode

		mode_titles = {
			"straight": ("DÜZ UÇUŞ", "#2b6cb0"),
			"s_turn": ("S-MANEVRASI", "#dd6b20"),
			"weave": ("S-MANEVRASI", "#dd6b20"),
			"orbit": ("ORBİT (DAİRE)", "#805ad5"),
			"square": ("KARE DEVRE", "#319795"),
		}
		title, color = mode_titles.get(mode, (mode.upper(), "#4a5568"))
		self.badge_mode.setText(title)
		self.badge_mode.setStyleSheet(
			f"background-color: {color}; color: white; border-radius: 4px; "
			"padding: 4px 10px; font-weight: bold; font-size: 11px;"
		)

		# Highlight active mode button
		active_style = "background-color: #3182ce; color: white; font-weight: bold;"
		default_style = ""
		self.btn_straight.setStyleSheet(active_style if mode == "straight" else default_style)
		self.btn_sturn.setStyleSheet(active_style if mode in ("s_turn", "weave") else default_style)
		self.btn_orbit.setStyleSheet(active_style if mode == "orbit" else default_style)

		# Update telemetry labels
		spd = data.get("ground_speed_mps", 0.0)
		alt = data.get("alt_rel_m", 0.0)
		hdg = data.get("heading_deg", 0.0)
		roll = data.get("roll_deg", 0.0)

		self.tel_speed.setText(f"{spd:4.1f} m/s")
		self.tel_alt.setText(f"{alt:5.1f} m")
		self.tel_heading.setText(f"{hdg:5.1f}°")
		self.tel_roll.setText(f"{roll:4.1f}°")

	def _dark_theme_qss(self):
		return """
		QMainWindow {
			background-color: #1a202c;
		}
		QWidget {
			color: #e2e8f0;
			font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif;
			font-size: 12px;
		}
		QFrame#card {
			background-color: #2d3748;
			border-radius: 6px;
			border: 1px solid #4a5568;
		}
		QGroupBox {
			background-color: #2d3748;
			border: 1px solid #4a5568;
			border-radius: 6px;
			margin-top: 12px;
			padding-top: 10px;
			font-weight: bold;
			font-size: 11px;
			color: #cbd5e0;
		}
		QGroupBox::title {
			subcontrol-origin: margin;
			subcontrol-position: top left;
			left: 10px;
			padding: 0 4px;
		}
		QPushButton {
			background-color: #4a5568;
			color: #f7fafc;
			border: 1px solid #718096;
			border-radius: 4px;
			padding: 6px 10px;
			font-weight: 500;
		}
		QPushButton:hover {
			background-color: #718096;
			border-color: #a0aec0;
		}
		QPushButton:pressed {
			background-color: #2b6cb0;
			border-color: #3182ce;
		}
		QSlider::groove:horizontal {
			height: 6px;
			background: #4a5568;
			border-radius: 3px;
		}
		QSlider::sub-page:horizontal {
			background: #3182ce;
			border-radius: 3px;
		}
		QSlider::handle:horizontal {
			background: #e2e8f0;
			border: 1px solid #a0aec0;
			width: 14px;
			margin-top: -4px;
			margin-bottom: -4px;
			border-radius: 7px;
		}
		QSlider::handle:horizontal:hover {
			background: #ffffff;
			border-color: #63b3ed;
		}
		"""


def main():
	parser = argparse.ArgumentParser(description="Talon Hedef Kontrol Paneli (GUI)")
	parser.add_argument("--host", default="127.0.0.1", help="Target sim HTTP host")
	parser.add_argument("--port", type=int, default=8000, help="Target sim HTTP port")
	args = parser.parse_args()

	app = QApplication(sys.argv)
	app.setStyle("Fusion")
	win = TargetControlWindow(host=args.host, port=args.port)
	win.show()
	sys.exit(app.exec())


if __name__ == "__main__":
	main()
