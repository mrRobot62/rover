class ObstacleDetector:
    def __init__(self):
        self.lidar_points = []
        self.camera_data = None

    def update_from_lidar(self, points):
        self.lidar_points = points

    def update_from_camera(self, image):
        self.camera_data = image

    def detect(self):
        # Hindernisse anhand Sensorfusion erkennen
        return []