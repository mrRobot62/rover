class KinematicsModel:
    def __init__(self, wheelbase: float, track_width: float):
        self.wheelbase = wheelbase
        self.track_width = track_width

    def twist_to_wheel(self, linear_vel: float, angular_vel: float):
        # Rechnet Geschwindigkeit + Drehung in linkes und rechtes Rad um
        return linear_vel - angular_vel, linear_vel + angular_vel

    def update_odometry(self, delta_left: float, delta_right: float):
        # Dummy: Update Position/Orientierung basierend auf Encoder-Delta
        return (0.0, 0.0, 0.0)