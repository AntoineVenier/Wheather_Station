from PySide2.QtCore import Slot, QObject, Signal, Property, QTimer
from sensor import *

class SensorInterface(QObject):

    def __init__(self):
        super().__init__()
        self.sensor = Sensor()
        self.time_refresh = 1000 # Time between value refresh in milliseconds
        self.timer = Qtimer(interval=self.time_refresh)
        self.timer.timeout.connect(self.sensor.refresh_data)
        self.timer.start()

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
        return self.sensor.pressure

    def get_humidity(self):
        return self.sensor.humidity

    temperature = Property(float, get_temperature, notify=temperature_changed)
    pressure = Property(float, get_pressure, notify = pressure_changed)
    humidity = Property(float, get_humidity, notify = humidity_changed)


