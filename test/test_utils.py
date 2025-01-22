import pytest
from py_odrive.lib.utils import CanDevice, ProcessYaml

# Fixtures for CanDevice instances
@pytest.fixture()
def online_can_instance():
    """Provides an online instance of CanDevice."""
    return CanDevice('CanDevice', 'online')

@pytest.fixture()
def offline_can_instance():
    """Provides an offline instance of CanDevice."""
    return CanDevice('CanDevice', 'offline')

@pytest.fixture()
def null_can_instance():
    """Provides a CanDevice instance without a specified status."""
    return CanDevice('CanDevice')

# Tests for CanDevice functionality
def test_device_setup_normal(online_can_instance):
    """Tests normal setup and status toggling of an online CanDevice instance."""
    assert online_can_instance.get_bus() == 'CanDevice', "Mismatched CAN Device attribute"
    assert online_can_instance.get_status() == 'online', f"Mismatched CAN Device status: {online_can_instance.get_status()} != 'online'"
    online_can_instance.toggle_status('offline')
    assert online_can_instance.get_status() == 'offline', f"Mismatched CAN Device status after toggle: {online_can_instance.get_status()} != 'offline'"

def test_device_setup_abnormal(online_can_instance):
    """Tests abnormal status toggling for an online CanDevice instance."""
    assert online_can_instance.get_bus() == 'CanDevice', "Mismatched CAN Device attribute"
    assert online_can_instance.get_status() == 'online', f"Mismatched CAN Device status: {online_can_instance.get_status()} != 'online'"
    online_can_instance.toggle_status('random')  # Assuming 'random' defaults to 'offline'
    assert online_can_instance.get_status() == 'offline', f"Mismatched CAN Device status after invalid toggle: {online_can_instance.get_status()} != 'offline'"

def test_device_setup_init(offline_can_instance, null_can_instance):
    """Tests the initialization of CanDevice with offline and null statuses."""
    assert offline_can_instance.get_bus() == 'CanDevice', "Mismatched CAN Device attribute"
    assert offline_can_instance.get_status() == 'offline', f"Mismatched CAN Device status: {offline_can_instance.get_status()} != 'offline'"
    assert null_can_instance.get_bus() == 'CanDevice', "Mismatched CAN Device attribute"
    assert null_can_instance.get_status() == 'offline', f"Mismatched CAN Device status: {null_can_instance.get_status()} != 'offline'"

# Fixtures for ProcessYaml
@pytest.fixture
def test_valid_config_path():
    """Provides the path to a valid YAML configuration file."""
    return 'test/test_asset/valid_config.yaml'

@pytest.fixture
def test_invalid_config_path():
    """Provides the path to an invalid YAML configuration file."""
    return 'test/test_asset/invalid_config.yaml'

@pytest.fixture
def test_mapping_dct():
    """Provides a sample dictionary for motor mapping."""
    return {
        'FrontLeftMotor': 'axis1',
        'MiddleLeftMotor': 'axis2',
        'BackLeftMotor': 'axis3',
        'FrontRightMotor': 'axis4',
        'MiddleRightMotor': 'axis5',
        'BackRightMotor': 'axis6'
    }

# Tests for ProcessYaml functionality
def test_read_yaml(test_valid_config_path, test_mapping_dct):
    """
    Tests reading and parsing a valid YAML configuration file using ProcessYaml.
    
    - Verifies the structure and types of returned configurations.
    - Validates correct mapping and attribute retrieval.
    """
    yaml_dct = ProcessYaml(test_valid_config_path)
    
    # Test exception when accessing a key with missing required arguments
    with pytest.raises(Exception):
        yaml_dct.get_config('bustype', device_name='Drivetrain', dct=test_mapping_dct)
    
    # Test valid configuration retrieval
    assert isinstance(yaml_dct.get_config('bustype', device_name='Drivetrain'), str)
    assert yaml_dct.get_config('bustype', device_name='Drivetrain') == 'socketcan'
    assert isinstance(yaml_dct.get_config('bitrate', device_name='Drivetrain'), int)
    assert yaml_dct.get_config('bitrate', device_name='Drivetrain') == 500000
    
    # Test motor mapping retrieval
    motor_mapping = yaml_dct.get_config('mapping', device_name='Drivetrain')
    assert isinstance(motor_mapping, dict)
    assert motor_mapping == test_mapping_dct
    assert isinstance(yaml_dct.get_config('FrontLeftMotor', dct=motor_mapping), str)
    assert yaml_dct.get_config('FrontLeftMotor', dct=motor_mapping) == 'axis1'
    
    # Test invalid access
    assert yaml_dct.get_config('random', device_name='Drivetrain') is None
    assert yaml_dct.get_config('Motor', dct=motor_mapping) is None

def test_read_yaml_exception(test_invalid_config_path):
    """Tests exception handling when reading an invalid YAML configuration file."""
    with pytest.raises(Exception):
        ProcessYaml(test_invalid_config_path)
