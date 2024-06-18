import unittest
from unittest.mock import patch, MagicMock
import dvl_driver

class TestDVLDriver(unittest.TestCase):
    @patch('dvl_driver.DVLDriver.s.recv')
    def test_extract_data(self, mock_recv):
        # Create an instance of DVLDriver
        driver = dvl_driver.DVLDriver()

        # Define the behavior of the mock object
        mock_recv.side_effect = [b'data', b'\n']

        # Call the method under test
        result = driver.extract_data()

        # Assert that the method behaved as expected
        self.assertEqual(result, 'data')

    @patch('json.loads')
    def test_receive_dvl(self, mock_json_loads):
        # Create an instance of DVLDriver
        driver = dvl_driver.DVLDriver()

        # Define the behavior of the mock object
        mock_json_loads.return_value = {
            "vx": 1.0,
            "vy": 2.0,
            "vz": 3.0,
            "covariance": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            "altitude": 4.0,
            "transducers": [
                {"distance": 5.0, "velocity": 6.0},
                {"distance": 7.0, "velocity": 8.0},
                {"distance": 9.0, "velocity": 10.0},
                {"distance": 11.0, "velocity": 12.0}
            ],
            "velocity_valid": True
        }

        # Call the method under test
        driver.receive_dvl('raw_data')

        # Assert that the method behaved as expected
        # This could involve checking the values of theDVL and the published messages

if __name__ == '__main__':
    unittest.main()