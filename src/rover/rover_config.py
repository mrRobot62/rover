import yaml

class RoverConfig:
    """
    Diese Klasse vereinfacht den Zugriff auf die die rover.yaml ROS2-Konfigurationsdatei.

    Die Kapselung für den Zugriff ermöglichst es, dass auch in einfachen Python-Klassen Daten
    aus der yaml-Konfiguration gelesen werden können. 
    
    """

    def __init__(self, yaml_file):
        self.yaml_file = yaml_file
        self.config = {}
        self.load_config()

    def load_config(self):
        with open(self.yaml_file, 'r') as f:
            self.config = yaml.safe_load(f)
            print("[RoverConfig] Config loaded:", self.config)

    def get(self, key_path, default=None):
        keys = key_path.split('.')
        val = self.config
        for k in keys:
            if isinstance(val, dict):
                val = val.get(k, default)
            else:
                return default
        return val
