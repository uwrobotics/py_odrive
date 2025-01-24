import pytest
from py_odrive.lib.utils import CanDevice, ProcessYaml

# Fixtures for CanDevice instances
@pytest.fixture()
def online_can_instance():
    """
    Provides an online instance of CanDevice for testing.

    Returns:
        CanDevice: An instance with the status 'online'.
    """
    return CanDevice('CanDevice', 'online')

@pytest.fixture()
def offline_can_instance():
    """
    Provides an offline instance of CanDevice for testing.

    Returns:
        CanDevice: An instance with the status 'offline'.
    """
    return CanDevice('CanDevice', 'offline')

@pytest.fixture()
def null_can_instance():
    """
    Provides a CanDevice instance with no explicitly set status.

    Returns:
        CanDevice: An instance defaulting to 'offline'.
    """
    return CanDevice('CanDevice')

# Tests for CanDevice functionality
def test_device_setup_normal(online_can_instance):
    """
    Tests normal setup and status toggling of an online CanDevice instance.

    Args:
        online_can_instance (CanDevice): Fixture providing an online CanDevice instance.

    Validates:
        - Initial status is 'online'.
        - Status toggles correctly to 'offline'.
    """
    assert online_can_instance.get_bus() == 'CanDevice', "Mismatched CAN Device attribute"
    assert online_can_instance.get_status() == 'online', "Initial status is not 'online'"
    online_can_instance.toggle_status('offline')
    assert online_can_instance.get_status() == 'offline', "Status did not toggle to 'offline'"

def test_device_setup_abnormal(online_can_instance):
    """
    Tests handling of invalid status toggles for an online CanDevice instance.

    Args:
        online_can_instance (CanDevice): Fixture providing an online CanDevice instance.

    Validates:
        - Initial status is 'online'.
        - Invalid status toggle defaults to 'offline'.
    """
    assert online_can_instance.get_bus() == 'CanDevice', "Mismatched CAN Device attribute"
    assert online_can_instance.get_status() == 'online', "Initial status is not 'online'"
    online_can_instance.toggle_status('random')  # Assuming 'random' defaults to 'offline'
    assert online_can_instance.get_status() == 'offline', "Invalid status did not default to 'offline'"

def test_device_setup_init(offline_can_instance, null_can_instance):
    """
    Tests initialization of CanDevice with offline and default (null) statuses.

    Args:
        offline_can_instance (CanDevice): Fixture providing an offline CanDevice instance.
        null_can_instance (CanDevice): Fixture providing a CanDevice instance with no explicit status.

    Validates:
        - Correct initialization of offline and null statuses.
    """
    assert offline_can_instance.get_bus() == 'CanDevice', "Mismatched CAN Device attribute"
    assert offline_can_instance.get_status() == 'offline', "Offline status initialization failed"
    assert null_can_instance.get_bus() == 'CanDevice', "Mismatched CAN Device attribute"
    assert null_can_instance.get_status() == 'offline', "Default status initialization failed"

# Fixtures for ProcessYaml
@pytest.fixture
def test_valid_config_path():
    """
    Provides the path to a valid YAML configuration file for testing.

    Returns:
        str: Path to the valid YAML file.
    """
    return 'test/test_asset/valid_config.yaml'

@pytest.fixture
def test_invalid_config_path():
    """
    Provides the path to an invalid YAML configuration file for testing.

    Returns:
        str: Path to the invalid YAML file.
    """
    return 'test/test_asset/invalid_config.yaml'

@pytest.fixture
def test_mapping_dct():
    """
    Provides a sample dictionary for motor mapping.

    Returns:
        dict: A mapping of motor names to axis identifiers.
    """
    return {
        'FrontLeftMotor': 'axis1',
        'MiddleLeftMotor': 'axis2',
        'BackLeftMotor': 'axis3',
        'FrontRightMotor': 'axis4',
        'MiddleRightMotor': 'axis5',
        'BackRightMotor': 'axis6'
    }

@pytest.fixture
def test_config_dct():
    """
    Provides a sample dictionary representing YAML configurations.

    Returns:
        dict: A dictionary representing a configuration file.
    """
    return {
        'Drivetrain': {
            'bustype': 'socketcan',
            'channel': 'can0',
            'bitrate': 500000,
            'mapping': {
                'FrontLeftMotor': 'axis1',
                'MiddleLeftMotor': 'axis2',
                'BackLeftMotor': 'axis3',
                'FrontRightMotor': 'axis4',
                'MiddleRightMotor': 'axis5',
                'BackRightMotor': 'axis6'
            }
        },
        'Drivetrain1': {
            'bustype': 'slcan',
            'channel': '/dev/ttyACM1',
            'bitrate': 500000
        }
    }

# Tests for ProcessYaml functionality
def test_read_yaml(test_valid_config_path, test_mapping_dct, test_config_dct):
    """
    Tests reading and parsing a valid YAML configuration file using ProcessYaml.

    Args:
        test_valid_config_path (str): Path to the valid YAML configuration file.
        test_mapping_dct (dict): Expected motor mapping dictionary.
        test_config_dct (dict): Expected parsed YAML configuration dictionary.

    Validates:
        - Correct parsing of YAML into a dictionary.
        - Retrieval of specific configuration keys and values.
        - Handling of invalid key accesses.
    """
    yaml_dct = ProcessYaml(test_valid_config_path)

    # Validate overall configuration
    assert yaml_dct.get_config() == test_config_dct, "Parsed YAML does not match the expected configuration"

    # Test key-value retrieval
    assert yaml_dct.get_config(key='bustype', device_name='Drivetrain') == 'socketcan', "Incorrect bustype value"
    assert yaml_dct.get_config(key='bitrate', device_name='Drivetrain') == 500000, "Incorrect bitrate value"

    # Test motor mapping retrieval
    motor_mapping = yaml_dct.get_config(key='mapping', device_name='Drivetrain')
    assert motor_mapping == test_mapping_dct, "Motor mapping does not match expected"

    # Test invalid key access
    assert yaml_dct.get_config(key='random', device_name='Drivetrain') is None, "Invalid key did not return None"

def test_read_yaml_exception(test_invalid_config_path):
    """
    Tests exception handling when reading an invalid YAML configuration file.

    Args:
        test_invalid_config_path (str): Path to the invalid YAML configuration file.

    Validates:
        - Proper exception is raised when attempting to parse an invalid YAML file.
    """
    with pytest.raises(Exception):
        ProcessYaml(test_invalid_config_path)
