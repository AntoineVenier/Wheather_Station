class Sensor:

    def __init__(self):
        self.humidity = 0
        self.pressure = 0
        self.temperature = 0

    def refresh_data(self):
        """ Récupération sur la liaison série des nouvelles données """
        pass
