import pytest
from unittest.mock import patch, MagicMock
from your_module import SensorWriterBase  # Replace 'your_module' with the name of your module

# Mock the rospy functionalities
@pytest.fixture
def mock_rospy():
    with patch("your_module.rospy") as mock:
        yield mock

# Test the initialization of SensorWriterBase
def test_sensor_writer_base_init(mock_rospy):
    sensor_name = "test_sensor"
    sensor_channel = "test_channel"
    sensor_data_type = "String"  # Use actual data type if necessary
    anonymous = False
    queue_size = 10
    rate_hz = 10

    # Initialize SensorWriterBase
    sensor_writer = SensorWriterBase(sensor_name, sensor_channel, sensor_data_type, anonymous, queue_size, rate_hz)

    # Assertions to validate the initialization process
    mock_rospy.Publisher.assert_called_with(sensor_channel, sensor_data_type, queue_size=queue_size)
    mock_rospy.init_node.assert_called_with(sensor_name, anonymous=anonymous)
    mock_rospy.Rate.assert_called_with(rate_hz)
