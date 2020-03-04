import sys

from PySide2.QtWidgets import QApplication, QLabel
from PySide2.QtQml import QQmlApplicationEngine
from sensor_interface import *

if __name__ == "__main__":
    sensor_interface = SensorInterface()

    app = QApplication(sys.argv)
    engine = QQmlApplicationEngine()

    context = engine.rootContext()
    context.setContextProperty("sensor", sensor_interface)

    engine.load("qml/interface.qml")
    sys.exit(app.exec_())
