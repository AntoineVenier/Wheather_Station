from PySide2.QtCore import Slot, QObject, Signal, Property, QTimer, QThread, SIGNAL
from sensor import *

class SensorInterface(QObject):

    def __init__(self):
        super().__init__()
        self.sensor = Sensor()

    @Signal
    def temperature_changed(self):
        pass

    @Signal
    def pressure_changed(self):
        pass

    @Signal
    def humidity_changed(self):
        pass

    def get_temperature(self):
        return self.sensor.temperature

    def get_pressure(self):
        return self.sensor.pressure / 100

    def get_humidity(self):
        return self.sensor.humidity

    @Slot()
    def refresh(self):
        self.sensor.refresh_data()
        self.temperature_changed.emit()
        self.humidity_changed.emit()
        self.pressure_changed.emit()

    temperature = Property(float, get_temperature, notify=temperature_changed)
    pressure = Property(float, get_pressure, notify = pressure_changed)
    humidity = Property(float, get_humidity, notify = humidity_changed)
