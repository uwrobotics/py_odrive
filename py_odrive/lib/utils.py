from typing import Optional
import yaml


class CanDevice:
    """
    Represents a CAN device with basic functionality to toggle its status
    between 'online' and 'offline'.

    Attributes:
        bus (str): The name of the CAN device.
        status (str): The current status of the CAN device ('online' or 'offline').
    """

    def __init__(self, can_device: str, status: str = 'offline'):
        """
        Initializes the CanDevice instance.

        Args:
            can_device (str): The name of the CAN device.
            status (str, optional): The initial status of the CAN device. Defaults to 'offline'.
        """
        self.bus = can_device
        self.toggle_status(status)

    def get_bus(self) -> str:
        """
        Returns the name of the CAN device.

        Returns:
            str: The CAN device name.
        """
        return self.bus

    def toggle_status(self, status: str = 'offline'):
        """
        Toggles the status of the CAN device.

        Args:
            status (str): The new status ('online' or 'offline'). Defaults to 'offline'.
        """
        self.status = status if status == 'online' else 'offline'

    def get_status(self) -> str:
        """
        Returns the current status of the CAN device.

        Returns:
            str: The current status ('online' or 'offline').
        """
        return self.status


class ProcessYaml:
    """
    Handles reading and parsing YAML configuration files.

    Note: This module does not handle invalid YAML config files.
    """

    def __init__(self, config_file: str):
        """
        Initializes the ProcessYaml instance and reads the configuration file.

        Args:
            config_file (str): Path to the YAML configuration file.
        """
        self.yaml_obj = self.read_config(config_file)

    def get_config(self, key: Optional[str] = None, device_name: Optional[str] = None, dct: Optional[dict] = None) -> Optional[any]:
        """
        Retrieves a configuration value based on the provided key.

        Args:
            key (str): The key to retrieve the value for.
            device_name (str, optional): The name of the device to access in the YAML file. Defaults to None.
            dct (dict, optional): A dictionary to search for the key. Defaults to None.

        Returns:
            Optional[any]: The retrieved value or None if the key is not found.

        Raises:
            ValueError: If both `device_name` and `dct` are None or both are provided.
        """
        if key is None and device_name is None and dct is None:
            return self.yaml_obj
        if (dct is None) == (device_name is None):
            raise ValueError("Either `device_name` or `dct` must be provided, but not both.")
        
        return self._safe_access(self.yaml_obj[device_name] if dct is None else dct, key)

    def _safe_access(self, dct: dict, key: any) -> Optional[any]:
        """
        Safely accesses a key in the provided dictionary.

        Args:
            dct (dict): The dictionary to search.
            key (any): The key to retrieve.

        Returns:
            Optional[any]: The value associated with the key or None if the key is not found.
        """
        return dct.get(key, None)

    def read_config(self, config_file: str) -> dict:
        """
        Reads and parses the YAML configuration file.

        Args:
            config_file (str): Path to the YAML configuration file.

        Returns:
            dict: The parsed YAML configuration.

        Raises:
            TypeError: If the YAML file contains a list at the root level.
        """
        with open(config_file, mode='r', encoding="utf-8") as file:
            yaml_obj = yaml.safe_load(file)

        if not isinstance(yaml_obj, dict):
            raise TypeError("YAML file should contain key-value mappings, not a list.")
        
        return yaml_obj
