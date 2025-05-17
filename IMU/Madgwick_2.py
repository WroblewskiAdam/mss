import sys
import time
import numpy as np
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph.opengl as gl
from imufusion import Ahrs, Offset, Settings, CONVENTION_NED

import board
import busio
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
)

# ----------- Inicjalizacja IMU ----------
i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
bno = BNO08X_I2C(i2c)
time.sleep(1)

bno.enable_feature(BNO_REPORT_ACCELEROMETER)
bno.enable_feature(BNO_REPORT_GYROSCOPE)
bno.enable_feature(BNO_REPORT_MAGNETOMETER)

sample_rate_hz = 50
interval = 1.0 / sample_rate_hz

offset = Offset(sample_rate_hz)
ahrs = Ahrs()
ahrs.settings = Settings(CONVENTION_NED, 0.5, 2000, 10, 10, 5 * sample_rate_hz)

# ----------- Inicjalizacja sceny OpenGL ----------
app = QtGui.QApplication([])
win = gl.GLViewWidget()
win.setWindowTitle('IMU Orientation Viewer')
win.setGeometry(0, 0, 800, 600)
win.show()
win.setCameraPosition(distance=4)

# Siatka pomocnicza
g = gl.GLGridItem()
g.scale(1, 1, 1)
win.addItem(g)

# Klocek IMU
cube = gl.GLBoxItem(size=QtGui.QVector3D(1, 0.2, 0.5), color=(0.2, 0.6, 1, 1))
win.addItem(cube)

# ----------- Aktualizacja pozycji ----------
def update():
    accel = bno.acceleration
    gyro = bno.gyro
    mag = bno.magnetic

    accel_g = np.array(accel) / 9.80665
    gyro_deg = np.degrees(gyro)
    mag_uT = np.array(mag)

    gyro_deg = offset.update(gyro_deg)
    ahrs.update(gyro_deg, accel_g, mag_uT, interval)
    euler = np.radians(ahrs.quaternion.to_euler())  # roll, pitch, yaw w radianach

    roll, pitch, yaw = euler

    # Oblicz macierz rotacji 3D
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])

    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])

    R = Rz @ Ry @ Rx
    m = QtGui.QMatrix4x4()
    m.translate(0, 0, 0)
    for i in range(3):
        for j in range(3):
            m.setRow(i, QtGui.QVector4D(R[j, 0], R[j, 1], R[j, 2], 0) if i == j else m.row(i))
    m.setRow(3, QtGui.QVector4D(0, 0, 0, 1))
    cube.setTransform(m)

    QtGui.QApplication.processEvents()

# ----------- Timer aktualizacji ----------
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(int(interval * 1000))  # co 20 ms

# Start GUI loop
if __name__ == '__main__':
    QtGui.QApplication.instance().exec_()
