import RPi.GPIO as GPIO
import time
import threading
import sys

from PyQt6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QSlider, QGroupBox
)
from PyQt6.QtCore import Qt

from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
from matplotlib.figure import Figure


IN1 = 17
IN2 = 27
ENA = 13

ENC_A = 23

PPR_MOTEUR = 24
REDUCTION = 226
DT = 0.1

pulse_count = 0
running = False

rpm_motor = 0
rpm_reducer = 0

time_data = []
rpm_motor_data = []
rpm_reducer_data = []

rpm_motor_setpoint = 0

Kp = 0
Ki = 0
Kd = 0

integral = 0
prev_error = 0
settings_ok = False

# GPIO
GPIO.setmode(GPIO.BCM)

GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(ENC_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)

pwm = GPIO.PWM(ENA, 1000)
pwm.start(0)

def encoder_callback(channel):
    global pulse_count
    pulse_count += 1

GPIO.add_event_detect(ENC_A, GPIO.RISING, callback=encoder_callback)

# MOTOR
def start_motor():
    global integral, prev_error
    integral = 0
    prev_error = 0

    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    pwm.ChangeDutyCycle(50)

def stop_motor():
    pwm.ChangeDutyCycle(0)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)

def measure_speed():
    global pulse_count, rpm_motor, rpm_reducer
    global integral, prev_error

    t0 = time.time()
    pwm_value = 50

    while running:
        pulse_count = 0
        time.sleep(DT)
        pulses = pulse_count

        alpha = 0.7
        new_rpm = (pulses / PPR_MOTEUR) * (60 / DT)
        rpm_motor = alpha * rpm_motor + (1 - alpha) * new_rpm
        rpm_reducer = rpm_motor / REDUCTION

        error = rpm_motor_setpoint - rpm_motor

        integral += error * DT
        integral = max(min(integral, 5000), -5000)

        derivative = (error - prev_error) / DT

        output = (Kp * error) + (Ki * integral) + (Kd * derivative)

        pwm_value += output
        prev_error = error

        pwm_value = max(10, min(100, pwm_value))
        pwm.ChangeDutyCycle(pwm_value)

        t = time.time() - t0
        time_data.append(t)
        rpm_motor_data.append(rpm_motor)
        rpm_reducer_data.append(rpm_reducer)

        window.update_plot()

# INTERFACE
class MotorWindow(QWidget):

    def __init__(self):
        super().__init__()

        self.setWindowTitle("PID Motor Control - Professional")

        # ===== STYLE GLOBAL =====
        self.setStyleSheet("""
        QWidget {
            background-color: #121212;
            color: #E0E0E0;
        }

        QPushButton {
            border-radius: 8px;
            padding: 10px;
            font-weight: bold;
        }

        QGroupBox {
            border: 1px solid #333;
            border-radius: 10px;
            margin-top: 10px;
            padding: 10px;
            font-weight: bold;
        }

        QSlider::groove:horizontal {
            height: 6px;
            background: #444;
            border-radius: 3px;
        }

        QSlider::handle:horizontal {
            background: #00aaff;
            border: 2px solid #007acc;
            width: 14px;
            margin: -5px 0;
            border-radius: 7px;
        }
        """)

        main_layout = QHBoxLayout()

        # ================= LEFT PANEL =================
        left_layout = QVBoxLayout()

        btn_layout = QHBoxLayout()

        self.btn_start = QPushButton("START")
        self.btn_stop = QPushButton("STOP")
        self.btn_quit = QPushButton("QUIT")

        self.btn_start.setStyleSheet("background-color: #28a745; color: white;")
        self.btn_stop.setStyleSheet("background-color: #dc3545; color: white;")
        self.btn_quit.setStyleSheet("background-color: #007bff; color: white;")

        self.btn_start.clicked.connect(self.start_system)
        self.btn_stop.clicked.connect(self.stop_system)
        self.btn_quit.clicked.connect(self.quit_system)

        btn_layout.addWidget(self.btn_start)
        btn_layout.addWidget(self.btn_stop)
        btn_layout.addWidget(self.btn_quit)

        control_group = QGroupBox("Paramètres de contrôle")
        control_layout = QVBoxLayout()

        self.rpm_slider = QSlider(Qt.Orientation.Horizontal)
        self.rpm_slider.setRange(0, 4000)
        self.rpm_label = QLabel("Consigne RPM: 0")

        self.kp_slider = QSlider(Qt.Orientation.Horizontal)
        self.kp_slider.setRange(0, 500)
        self.kp_label = QLabel("Kp: 0")

        self.ki_slider = QSlider(Qt.Orientation.Horizontal)
        self.ki_slider.setRange(0, 500)
        self.ki_label = QLabel("Ki: 0")

        self.kd_slider = QSlider(Qt.Orientation.Horizontal)
        self.kd_slider.setRange(0, 500)
        self.kd_label = QLabel("Kd: 0")

        for w in [self.rpm_label, self.rpm_slider,
                  self.kp_label, self.kp_slider,
                  self.ki_label, self.ki_slider,
                  self.kd_label, self.kd_slider]:
            control_layout.addWidget(w)

        control_group.setLayout(control_layout)

        left_layout.addLayout(btn_layout)
        left_layout.addWidget(control_group)

        self.fig = Figure(figsize=(8, 6))
        self.fig.patch.set_facecolor("#121212")

        self.ax1 = self.fig.add_subplot(211)
        self.ax2 = self.fig.add_subplot(212)

        self.canvas = FigureCanvasQTAgg(self.fig)

        main_layout.addLayout(left_layout, 1)
        main_layout.addWidget(self.canvas, 2)

        self.setLayout(main_layout)

        
        self.rpm_slider.valueChanged.connect(self.update_values)
        self.kp_slider.valueChanged.connect(self.update_values)
        self.ki_slider.valueChanged.connect(self.update_values)
        self.kd_slider.valueChanged.connect(self.update_values)

    def update_values(self):
        global rpm_motor_setpoint, Kp, Ki, Kd, settings_ok

        rpm_motor_setpoint = self.rpm_slider.value()

        Kp = self.kp_slider.value() / 100
        Ki = self.ki_slider.value() / 100
        Kd = self.kd_slider.value() / 100

        self.rpm_label.setText(f"Consigne RPM: {rpm_motor_setpoint}")
        self.kp_label.setText(f"Kp: {Kp:.2f}")
        self.ki_label.setText(f"Ki: {Ki:.2f}")
        self.kd_label.setText(f"Kd: {Kd:.2f}")

        settings_ok = True

    def update_plot(self):
        self.ax1.clear()
        self.ax2.clear()

        self.ax1.set_facecolor("#0d1117")
        self.ax2.set_facecolor("#0d1117")

        self.ax1.plot(time_data, rpm_motor_data, color="#4ea8de", linewidth=2, label="Mesure")
        self.ax1.plot(time_data, [rpm_motor_setpoint]*len(time_data),
                      '--', color="#f77f00", linewidth=2, label="Consigne")

        self.ax1.set_title("COMPORTEMENT MOTEUR EN VITESSE", color="white", fontweight='bold')
        self.ax1.set_ylabel("RPM (moteur)", color="white")
        self.ax1.tick_params(colors='white')
        self.ax1.grid(True, linestyle='--', alpha=0.3)
        self.ax1.legend(facecolor="#1e1e1e")

        self.ax2.plot(time_data, rpm_reducer_data, color="#4ea8de", linewidth=2, label="Mesure")
        self.ax2.plot(time_data, [rpm_motor_setpoint/REDUCTION]*len(time_data),
                      '--', color="#f77f00", linewidth=2, label="Consigne")

        self.ax2.set_title("COMPORTEMENT REDUCTEUR EN VITESSE", color="white", fontweight='bold')
        self.ax2.set_xlabel("Temps (s)", color="white")
        self.ax2.set_ylabel("RPM (réducteur)", color="white")
        self.ax2.tick_params(colors='white')
        self.ax2.grid(True, linestyle='--', alpha=0.3)
        self.ax2.legend(facecolor="#1e1e1e")

        self.canvas.draw()

    def start_system(self):
        global running

        if not settings_ok:
            print("⚠️ Ajuster les paramètres")
            return

        if not running:
            running = True
            start_motor()
            threading.Thread(target=measure_speed, daemon=True).start()

    def stop_system(self):
        global running
        running = False
        stop_motor()

    def quit_system(self):
        self.stop_system()
        GPIO.cleanup()
        QApplication.quit()

# MAIN
app = QApplication(sys.argv)
window = MotorWindow()
window.show()
sys.exit(app.exec())
