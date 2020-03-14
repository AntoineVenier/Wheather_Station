from serial import *

class Sensor:

    def __init__(self):
        self.humidity = 0
        self.pressure = 0
        self.temperature = 0

    def refresh_data(self):
        """ Récupération sur la liaison série des nouvelles données """
        with Serial(port="/dev/ttyUSB0", baudrate=115200, timeout=1, writeTimeout=1) as port_serie:
            if port_serie.isOpen():
                i = 0
                while i < 3:
                    ligne = port_serie.readline()
                    ligne = ligne.decode().replace("\x00", "")
                    valeur = float(ligne[:ligne.find('.')+2])
                    if i == 0:
                        self.temperature = valeur
                    elif i == 1:
                        self.pressure = valeur
                    elif i == 2:
                        self.humidity = valeur
                    i += 1
