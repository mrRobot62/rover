from .sensor_interface import SensorInterface

class CameraSensor(SensorInterface):
    def capture_frame(self):
        # Bild von Kamera holen
        return None

    def get_image(self):
        # Rückgabe von Bilddaten (z. B. als NumPy-Array)
        return None