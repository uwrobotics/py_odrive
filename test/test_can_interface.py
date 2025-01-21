
import pytest
from py_odrive.lib.can_interface import CanDevice

@pytest.fixture()
def test_online_can_instance():
    return CanDevice('CanDevice', 'online')

@pytest.fixture()
def test_offline_can_instance():
    return CanDevice('CanDevice', 'offline')

@pytest.fixture()
def test_null_can_instance():
    return CanDevice('CanDevice')

def test_device_setup_normal(test_online_can_instance):
    print()
    assert test_online_can_instance.get_bus() == 'CanDevice', F'Miss matched CAN Device attribute'
    assert test_online_can_instance.get_status() == 'online', F'Miss matched CAN Device status {test_online_can_instance.get_status} == online'
    test_online_can_instance.toggle_status('offline')
    assert test_online_can_instance.get_status() == 'offline', F'Miss matched CAN Device status {test_online_can_instance.get_status} == offline'
    
def test_device_setup_abnormal(test_online_can_instance):
    assert test_online_can_instance.get_bus() == 'CanDevice', F'Miss matched CAN Device attribute'
    assert test_online_can_instance.get_status() == 'online', F'Miss matched CAN Device status {test_online_can_instance.get_status} == online'
    test_online_can_instance.toggle_status('random')
    assert test_online_can_instance.get_status() == 'offline', F'Miss matched CAN Device status {test_online_can_instance.get_status} == offline'

def test_device_setup_init(test_offline_can_instance, test_null_can_instance):
    assert test_offline_can_instance.get_bus() == 'CanDevice', F'Miss matched CAN Device attribute'
    assert test_offline_can_instance.get_status() == 'offline', F'Miss matched CAN Device status {test_offline_can_instance.get_status} == offline'
    assert test_null_can_instance.get_bus() == 'CanDevice', F'Miss matched CAN Device attribute'
    assert test_null_can_instance.get_status() == 'offline', F'Miss matched CAN Device status {test_null_can_instance.get_status} == offline'