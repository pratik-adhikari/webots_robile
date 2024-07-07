import sys
import math
import time
sys.path.append('/home/pratik/ros_ws/colcon_ws_AMR/src/robile_description/controllers/my_controller')

from controller import Robot
from my_controller import DifferentialDriveRobot

# Constants
TIMESTEP = 32
SECTIONS = 13
FORWARD_SPEED = 5.0
MAX_ANGLE = 2 * math.pi  # Maximum angle for full rotation
FORWARD_BIAS = 3.5  # Bias for central section to prioritize straight movement

class PatrolRobot:
    def __init__(self, driver):
        self.driver = driver  # DifferentialDriveRobot instance
        self.robot = Robot()
        self.lidar = self.robot.getLidar('lidar')
        self.lidar.enable(TIMESTEP)
        self.lidar.enablePointCloud()  # Ensure your lidar supports this

    def filter_scan_and_publish(self, lidar_ranges):
        filtered_ranges = []
        total_ranges = len(lidar_ranges)
        quarter_idx = total_ranges // 4
        three_quarters_idx = total_ranges * 3 // 4

        for i in range(three_quarters_idx, total_ranges):
            filtered_ranges.append(lidar_ranges[i] if not math.isinf(lidar_ranges[i]) else 5.0)
        for i in range(0, quarter_idx + 1):
            filtered_ranges.append(lidar_ranges[i] if not math.isinf(lidar_ranges[i]) else 5.0)
        
        return filtered_ranges

    def divide_into_sections_and_find_min(self, filtered_ranges):
        section_min_distances = [float('inf')] * SECTIONS
        points_per_section = len(filtered_ranges) // SECTIONS

        for section in range(SECTIONS):
            start_idx = section * points_per_section
            end_idx = (start_idx + points_per_section) if (section + 1 < SECTIONS) else len(filtered_ranges)
            section_min_distances[section] = min(filtered_ranges[start_idx:end_idx])

        return section_min_distances
        
    def find_safest_direction(self, min_distances):
        safest_section = min(range(SECTIONS), key=lambda i: min_distances[i])
        angle_per_section = MAX_ANGLE / SECTIONS
        direction = (safest_section - (SECTIONS // 2)) * angle_per_section
        direction_degrees = direction * (180 / math.pi)  # Convert to degrees

        # Normalize direction to a range of -30 to 30 degrees
        direction_degrees = max(min(direction_degrees, 30), -30)
        
        return direction_degrees

    def control_loop(self):
        while self.robot.step(TIMESTEP) != -1:
            lidar_ranges = self.lidar.getRangeImage()
            filtered_ranges = self.filter_scan_and_publish(lidar_ranges)
            min_distances = self.divide_into_sections_and_find_min(filtered_ranges)
            direction_degrees = self.find_safest_direction(min_distances)
            
            # Control the robot using the driver
            self.driver.set_front_pivot_angle(direction_degrees)
            self.driver.run(angle_degrees=direction_degrees, speed=FORWARD_SPEED)
            print(f"Driving with front steering to {direction_degrees} degrees.")

if __name__ == "__main__":
    drive_robot = DifferentialDriveRobot()
    patrol_robot = PatrolRobot(drive_robot)
    patrol_robot.control_loop()
