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
DT = 0.02

class Params:
    def __init__(self):
        self.consigne = 1.0
        self.Kp = 20
        self.Ki = 10
        self.Kd = 5

params = Params()


position_count = 0
running = False

time_data = []
position_data = []

integral = 0
prev_error = 0

# GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(ENC_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)

pwm = GPIO.PWM(ENA, 1000)
pwm.start(0)

def encoder_callback(channel):
    global position_count
    position_count += 1

GPIO.add_event_detect(ENC_A, GPIO.RISING, callback=encoder_callback)

# MOTOR
def set_motor(speed):
    if speed > 0:
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
    else:
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)

    duty = abs(speed)
    if 0 < duty < 20:
        duty = 20

    pwm.ChangeDutyCycle(min(duty, 100))

def stop_motor():
    pwm.ChangeDutyCycle(0)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)

def position_control():
    global running, integral, prev_error

    t0 = time.time()

    while running:
        time.sleep(DT)

        position = position_count / (PPR_MOTEUR * REDUCTION)
        error = params.consigne - position

        if abs(error) < 0.02:
            stop_motor()
            running = False
            window.status.setText("Statut : Arrêté")
            print("✅ Position atteinte")
            break

        integral += error * DT
        derivative = (error - prev_error) / DT

        # ⚠️ normalisation PID (important avec plage 0-100)
        output = (params.Kp * error) + (params.Ki * integral) + (params.Kd * derivative)
        output = output / 50  # scaling pour éviter saturation

        prev_error = error

        output = max(min(output, 100), -100)
        set_motor(output)

        t = time.time() - t0
        time_data.append(t)
        position_data.append(position)

        window.update_plot(position)

# GUI
class MotorWindow(QWidget):

    def __init__(self):
        super().__init__()

        self.setWindowTitle("PID POSITION CONTROL - Professional")

        # ===== STYLE =====
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

        main = QVBoxLayout()
        top = QHBoxLayout()

        left = QVBoxLayout()

        # boutons
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

        group = QGroupBox("PARAMÈTRES")
        sliders = QVBoxLayout()

        # consigne
        self.consigne_slider = QSlider(Qt.Orientation.Horizontal)
        self.consigne_slider.setRange(0, 50)
        self.consigne_slider.setValue(10)

        self.label_c = QLabel()

        # PID 0 → 100
        self.kp_slider = QSlider(Qt.Orientation.Horizontal)
        self.kp_slider.setRange(0, 100)

        self.ki_slider = QSlider(Qt.Orientation.Horizontal)
        self.ki_slider.setRange(0, 100)

        self.kd_slider = QSlider(Qt.Orientation.Horizontal)
        self.kd_slider.setRange(0, 100)

        self.label_kp = QLabel()
        self.label_ki = QLabel()
        self.label_kd = QLabel()

        for w in [
            self.label_c, self.consigne_slider,
            self.label_kp, self.kp_slider,
            self.label_ki, self.ki_slider,
            self.label_kd, self.kd_slider
        ]:
            sliders.addWidget(w)

        group.setLayout(sliders)

        left.addLayout(btns)
        left.addWidget(group)

        self.fig = Figure()
        self.fig.patch.set_facecolor("#121212")

        self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvasQTAgg(self.fig)

        top.addLayout(left, 1)
        top.addWidget(self.canvas, 2)

        status_layout = QHBoxLayout()

        self.status = QLabel("Statut : Arrêté")
        self.pos_label = QLabel("Position actuelle : 0.00 tours")
        self.consigne_label = QLabel("Consigne : 1.00 tours")

        status_layout.addWidget(self.status)
        status_layout.addWidget(self.pos_label)
        status_layout.addWidget(self.consigne_label)

        main.addLayout(top)
        main.addLayout(status_layout)

        self.setLayout(main)

        # SIGNALS
        self.consigne_slider.valueChanged.connect(self.update_values)
        self.kp_slider.valueChanged.connect(self.update_values)
        self.ki_slider.valueChanged.connect(self.update_values)
        self.kd_slider.valueChanged.connect(self.update_values)

        self.update_values()

    def update_values(self):
        params.consigne = self.consigne_slider.value() / 10
        params.Kp = self.kp_slider.value()
        params.Ki = self.ki_slider.value()
        params.Kd = self.kd_slider.value()

        self.label_c.setText(f"Position (tours): {params.consigne:.2f}")
        self.label_kp.setText(f"Kp: {params.Kp}")
        self.label_ki.setText(f"Ki: {params.Ki}")
        self.label_kd.setText(f"Kd: {params.Kd}")

        self.consigne_label.setText(f"Consigne : {params.consigne:.2f} tours")

    def update_plot(self, position):
        self.ax.clear()
        self.ax.set_facecolor("#0d1117")

        self.ax.plot(time_data, position_data, color="#4ea8de", label="Position")
        self.ax.axhline(params.consigne, linestyle='--', color="#f77f00", label="Consigne")

        self.ax.set_title("COMPORTEMENT POSITION", color="white", fontweight='bold')
        self.ax.set_xlabel("Temps (s)", color="white")
        self.ax.set_ylabel("Tours", color="white")

        self.ax.tick_params(colors='white')
        self.ax.grid(True, linestyle='--', alpha=0.3)
        self.ax.legend(facecolor="#1e1e1e")

        self.pos_label.setText(f"Position actuelle : {position:.2f} tours")

        self.canvas.draw()

    def start_system(self):
        global running, position_count, integral, prev_error

        if not running:
            running = True
            self.status.setText("Statut : En marche")

            position_count = 0
            integral = 0
            prev_error = 0
            time_data.clear()
            position_data.clear()

            threading.Thread(target=position_control, daemon=True).start()

    def stop_system(self):
        global running
        running = False
        self.status.setText("Statut : Arrêté")
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
