from .sensor_interface import SensorInterface

class UltrasoundSensor(SensorInterface):
    def read_data(self):
        # Triggern + Messen der Ultraschallzeit
        return 0.0