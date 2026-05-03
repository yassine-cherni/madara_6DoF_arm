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
DT = 0.2

pulse_count = 0
running = False

rpm_motor = 0
rpm_reducer = 0

time_data = []
rpm_data = []
rpm_red_data = []

pwm_value = 0

# GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(ENC_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)

pwm = GPIO.PWM(ENA, 5000)
pwm.start(0)

# ENCODER
def encoder_callback(channel):
    global pulse_count
    pulse_count += 1

GPIO.add_event_detect(ENC_A, GPIO.RISING, callback=encoder_callback)


# MOTOR
def set_motor(duty):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    pwm.ChangeDutyCycle(duty)

def stop_motor():
    pwm.ChangeDutyCycle(0)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)

# MEASURE SPEED
def measure_speed():
    global pulse_count, rpm_motor, rpm_reducer

    t0 = time.time()

    while running:
        pulse_count = 0
        time.sleep(DT)
        pulses = pulse_count

        rpm_motor = (pulses / PPR_MOTEUR) * (60 / DT)
        rpm_reducer = rpm_motor / REDUCTION

        t = time.time() - t0

        time_data.append(t)
        rpm_data.append(rpm_motor)
        rpm_red_data.append(rpm_reducer)

        window.update_plot()

# GUI
class MotorWindow(QWidget):

    def __init__(self):
        super().__init__()

        self.setWindowTitle("MESURE VITESSE - Professional")

        self.setStyleSheet("""
        QWidget { background:#121212; color:#E0E0E0; }

        QPushButton {
            border-radius:8px;
            padding:10px;
            font-weight:bold;
        }

        QGroupBox {
            border:1px solid #333;
            border-radius:10px;
            padding:10px;
        }

        QSlider::groove:horizontal {
            height:6px;
            background:#444;
        }

        QSlider::handle:horizontal {
            background:#00aaff;
            width:14px;
            border-radius:7px;
        }
        """)

        main = QHBoxLayout()

        # ================= LEFT =================
        left = QVBoxLayout()

        btns = QHBoxLayout()

        self.btn_start = QPushButton("START")
        self.btn_stop = QPushButton("STOP")
        self.btn_quit = QPushButton("QUIT")

        self.btn_start.setStyleSheet("background:#28a745; color:white;")
        self.btn_stop.setStyleSheet("background:#dc3545; color:white;")
        self.btn_quit.setStyleSheet("background:#007bff; color:white;")

        self.btn_start.clicked.connect(self.start_system)
        self.btn_stop.clicked.connect(self.stop_system)
        self.btn_quit.clicked.connect(self.quit_system)

        btns.addWidget(self.btn_start)
        btns.addWidget(self.btn_stop)
        btns.addWidget(self.btn_quit)

        group = QGroupBox("Contrôle PWM")

        layout = QVBoxLayout()

        self.slider_pwm = QSlider(Qt.Orientation.Horizontal)
        self.slider_pwm.setRange(0, 100)

        self.label_pwm = QLabel("PWM : 0%")
        self.label_rpm = QLabel("RPM moteur : 0")
        self.label_red = QLabel("RPM réducteur : 0")

        layout.addWidget(self.label_pwm)
        layout.addWidget(self.slider_pwm)
        layout.addWidget(self.label_rpm)
        layout.addWidget(self.label_red)

        group.setLayout(layout)

        left.addLayout(btns)
        left.addWidget(group)

        # ================= GRAPH =================
        self.fig = Figure()
        self.fig.patch.set_facecolor("#121212")

        self.ax1 = self.fig.add_subplot(211)
        self.ax2 = self.fig.add_subplot(212)

        self.canvas = FigureCanvasQTAgg(self.fig)

        main.addLayout(left, 1)
        main.addWidget(self.canvas, 2)

        self.setLayout(main)

        # SIGNAL
        self.slider_pwm.valueChanged.connect(self.update_pwm)

    def update_pwm(self):
        global pwm_value
        pwm_value = self.slider_pwm.value()

        self.label_pwm.setText(f"PWM : {pwm_value}%")
        set_motor(pwm_value)

    def update_plot(self):
        self.ax1.clear()
        self.ax2.clear()

        self.ax1.set_facecolor("#0d1117")
        self.ax2.set_facecolor("#0d1117")

        self.ax1.plot(time_data, rpm_data, color="#4ea8de")
        self.ax1.set_title("VITESSE MOTEUR", color="white")
        self.ax1.set_ylabel("RPM", color="white")

        self.ax2.plot(time_data, rpm_red_data, color="#4ea8de")
        self.ax2.set_title("VITESSE REDUCTEUR", color="white")
        self.ax2.set_ylabel("RPM", color="white")

        self.ax1.tick_params(colors='white')
        self.ax2.tick_params(colors='white')

        self.ax1.grid(True, linestyle='--', alpha=0.3)
        self.ax2.grid(True, linestyle='--', alpha=0.3)

        self.label_rpm.setText(f"RPM moteur : {rpm_motor:.2f}")
        self.label_red.setText(f"RPM réducteur : {rpm_reducer:.2f}")

        self.canvas.draw()

    def start_system(self):
        global running
        if not running:
            running = True
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
