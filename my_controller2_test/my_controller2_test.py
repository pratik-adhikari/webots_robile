import unittest
from unittest.mock import MagicMock, patch
from my_controller2 import *
#from my_controller2 import PatrolRobot, TIMESTEP, SECTIONS_CLOSE, SECTIONS_MID, SECTIONS_LONG

class TestPatrolRobot(unittest.TestCase):
    def setUp(self):
        patcher_robot = patch('controller.Robot', autospec=True)
        patcher_lidar = patch('controller.Lidar', autospec=True)
        self.addCleanup(patcher_robot.stop)
        self.addCleanup(patcher_lidar.stop)
        
        self.mock_robot_class = patcher_robot.start()
        self.mock_lidar_class = patcher_lidar.start()
        
        self.mock_robot = self.mock_robot_class.return_value
        self.mock_lidar = self.mock_lidar_class.return_value
        self.mock_robot.getDevice.return_value = self.mock_lidar
        
        self.patrol_robot = PatrolRobot()

    def test_initialization(self):
        self.mock_robot.getDevice.assert_any_call("robile_1_drive_left_hub_wheel_joint")
        self.mock_robot.getDevice.assert_any_call("lidar")
        self.mock_lidar.enable.assert_called_once_with(TIMESTEP)
        self.mock_lidar.enablePointCloud.assert_called_once()

    def test_filter_lidar_data(self):
        # Mock lidar data
        lidar_ranges = [float('inf')] * 720
        filtered_ranges = self.patrol_robot.filter_lidar_data(lidar_ranges)
        self.assertEqual(len(filtered_ranges), 360)
        self.assertTrue(all(distance == 12.0 for distance in filtered_ranges))

    def test_process_lidar_data(self):
        # Mock lidar data
        self.mock_lidar.getRangeImage.return_value = [float('inf')] * 720
        min_distances = self.patrol_robot.process_lidar_data(SECTIONS_CLOSE)
        self.assertEqual(len(min_distances), SECTIONS_CLOSE)
        self.assertTrue(all(distance == 12.0 for distance in min_distances))

    def test_process_lidar_data_cumulative(self):
        # Mock lidar data
        self.mock_lidar.getRangeImage.return_value = [float('inf')] * 720
        cumulative_distances = self.patrol_robot.process_lidar_data_cumulative(SECTIONS_LONG)
        self.assertEqual(len(cumulative_distances), 5)  # 5 groups
        self.assertTrue(all(distance == 72.0 for distance in cumulative_distances))

    def test_is_valid_group(self):
        cumulative_distances = [10, 25, 30, 40, 15]
        self.assertTrue(self.patrol_robot.is_valid_group(cumulative_distances))

        cumulative_distances = [10, 15, 15, 15, 15]
        self.assertFalse(self.patrol_robot.is_valid_group(cumulative_distances))

    def test_find_safest_direction(self):
        min_distances = [1.0, 2.0, 3.0]
        min_safe_distance = 1.5
        direction = self.patrol_robot.find_safest_direction(min_distances, min_safe_distance, 1)
        self.assertIsNone(direction)

        min_distances = [2.0, 3.0, 4.0]
        direction = self.patrol_robot.find_safest_direction(min_distances, min_safe_distance, 1)
        self.assertEqual(direction, 30.0)

    def test_find_safest_direction_cumulative(self):
        cumulative_distances = [10, 20, 30, 40, 50]
        direction = self.patrol_robot.find_safest_direction_cumulative(cumulative_distances, 3)
        self.assertEqual(direction, 60)

        cumulative_distances = [10, 20, 30, 10, 10]
        direction = self.patrol_robot.find_safest_direction_cumulative(cumulative_distances, 3)
        self.assertEqual(direction, 0)

if __name__ == '__main__':
    unittest.main()
