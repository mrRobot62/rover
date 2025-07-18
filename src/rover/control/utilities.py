import os
import yaml
from ament_index_python.packages import get_package_share_directory


"""
Kleine Hilfsfunktionen
"""

class Utilities:

    @staticmethod
    def clamp(value: float, min_value: float, max_value: float) -> float:
        """
        Begrenzt einen numerischen Wert auf den Bereich zwischen min_value und max_value.
        
        Args:
            value (float): Der zu prüfende Wert.
            min_value (float): Untere Grenze.
            max_value (float): Obere Grenze.
        
        Returns:
            float: Der begrenzte Wert.
        """
        return max(min_value, min(value, max_value))

    @staticmethod
    def clamp(value: int, min_value: int, max_value: int) -> int:
        """
        Begrenzt einen numerischen Wert auf den Bereich zwischen min_value und max_value.
        
        Args:
            value (int): Der zu prüfende Wert.
            min_value (int): Untere Grenze.
            max_value (int): Obere Grenze.
        
        Returns:
            int: Der begrenzte Wert.
        """
        return max(min_value, min(value, max_value))

    @staticmethod
    def find_key_by_value(d: dict, value) -> str | None:
        """
        Sucht in einem Dictionary nach einem bestimmten Wert.
        
        Args:
            d (dict): Das Dictionary.
            value: Der zu suchende Wert.
        
        Returns:
            str | None: Der zugehörige Key, wenn gefunden, sonst None.
        """
        for key, val in d.items():
            if val == value:
                return key
        return None

    @staticmethod
    def get_value_or_default(my_dict, key, default_key=5):
        return my_dict.get(key, my_dict.get(default_key))
    
    @staticmethod
    def load_yaml_file(relative_path: str) -> dict:
        """
        Lädt eine YAML-Datei relativ zum share-Verzeichnis des rover-Pakets.
        """
        share_dir = get_package_share_directory('rover')
        yaml_path = os.path.join(share_dir, relative_path)

        if not os.path.exists(yaml_path):
            raise FileNotFoundError(f"YAML-Datei nicht gefunden: {yaml_path}")

        with open(yaml_path, 'r') as f:
            return yaml.safe_load(f)

    @staticmethod
    def load_common_topics(relative_path='config/rover_common.yaml') -> dict:
        """
        Lädt die common_topics aus einer YAML-Datei.
        """
        data = Utilities.load_yaml_file(relative_path)
        return data.get('common_topics', {}).get('ros__parameters', {})

    @staticmethod
    def get_common_topic(key: str, default = None, logger=None) -> str:
        """
        Gibt einen bestimmten Topic-Namen aus common_topics zurück.
        """
        try:
            topics = Utilities.load_common_topics()
            return topics.get(key, default)
        except Exception as e:
            if logger:
                logger.warn(f"Konnte common topic '{key}' nicht laden: {e}")
            else:
                print(f"[WARN] Konnte common topic '{key}' nicht laden: {e}")
            return default