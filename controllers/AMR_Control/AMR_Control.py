from controller import Robot
import math

# Constants
TIMESTEP = 32
DEFAULT_FORWARD_SPEED = 10.0  # Default speed value
SECTIONS = 13
MAX_ANGLE = 2 * math.pi  # Maximum angle for full rotation

class BaseDriveRobot:
    def __init__(self):
        self.robot = Robot()
        
        # Initialize the drive motors (all wheels are motorized for driving)
        self.drive_motors = [
            self.robot.getDevice("robile_1_drive_left_hub_wheel_joint"),
            self.robot.getDevice("robile_1_drive_right_hub_wheel_joint"),
            self.robot.getDevice("robile_2_drive_left_hub_wheel_joint"),
            self.robot.getDevice("robile_2_drive_right_hub_wheel_joint"),
            self.robot.getDevice("robile_5_drive_left_hub_wheel_joint"),
            self.robot.getDevice("robile_5_drive_right_hub_wheel_joint"),
            self.robot.getDevice("robile_6_drive_left_hub_wheel_joint"),
            self.robot.getDevice("robile_6_drive_right_hub_wheel_joint")
        ]
        
        # Initialize the front steering pivot motors
        self.front_pivot_motors = [
            self.robot.getDevice("robile_1_drive_pivot_joint"),
            self.robot.getDevice("robile_2_drive_pivot_joint")
        ]

        # Rear passive pivots with limited freedom (not actively controlled)
        self.rear_pivot_motors = [
            self.robot.getDevice("robile_5_drive_pivot_joint"),
            self.robot.getDevice("robile_6_drive_pivot_joint")
        ]

        # Setup the motors for continuous rotation, initially stopped
        for motor in self.drive_motors + self.rear_pivot_motors:
            motor.setPosition(float('inf'))  # Allow continuous rotation for drive motors
            motor.setVelocity(0.0)  # Start with no movement

    def set_front_pivot_angle(self, angle_degrees):
        """Set the front pivot angles within limits."""
        angle_radians = angle_degrees * math.pi / 180
        for motor in self.front_pivot_motors:
            motor.setPosition(angle_radians)
        print(f"Front pivots set to {angle_degrees} degrees.")

    def run(self, angle_degrees=0, speed=DEFAULT_FORWARD_SPEED):
        """Run the robot with specified steering angle and speed."""
        for motor in self.drive_motors:
            motor.setVelocity(speed)
        self.set_front_pivot_angle(angle_degrees)
        print(f"Driving forward at {speed} m/s with front steering to {angle_degrees} degrees.")


class PatrolRobot(BaseDriveRobot):
    def __init__(self):
        super().__init__()
        self.lidar = self.robot.getDevice('lidar')
        self.lidar.enable(TIMESTEP)
        self.lidar.enablePointCloud()  # Ensure your lidar supports this feature

    def process_lidar_data(self):
        lidar_ranges = self.lidar.getRangeImage()
        section_count = SECTIONS  # The total number of sections
        section_width = len(lidar_ranges) // section_count
        section_min_distances = []
        
        print("Section-wise Lidar Data:")
        for i in range(section_count):
            section = lidar_ranges[i * section_width: (i + 1) * section_width]
            min_distance = min(section, default=float('inf'))  # Handle empty sections safely
            section_min_distances.append(min_distance)
            print(f"Section {i+1}: Min distance: {min_distance:.2f}")
        
        # Find the safest section based on the minimum distances
        safest_section = section_min_distances.index(min(section_min_distances))
        
        # Calculate the angle per section based on a total steering range of 60 degrees
        angle_per_section = 60 / section_count  # Total of 60 degrees span (-30 to +30)
        
        # Calculate the steering direction in degrees
        center_index = section_count // 2
        direction_degrees = (safest_section - center_index) * angle_per_section
        
        # Clamp the direction degrees between -30 and 30 degrees
        direction_degrees = max(min(direction_degrees, 30), -30)
        
        print(f"Safest section: {safest_section + 1}, Direction: {direction_degrees:.2f} degrees")
        return direction_degrees
    
    
    

    def navigation_loop(self):
        """Control loop for navigating based on lidar data."""
        while self.robot.step(TIMESTEP) != -1:
            direction_degrees = self.process_lidar_data()
            self.set_front_pivot_angle(direction_degrees)
            self.run(angle_degrees=direction_degrees, speed=DEFAULT_FORWARD_SPEED)
            print(f"Navigating with direction {direction_degrees} degrees.")

if __name__ == "__main__":
    patrol_robot = PatrolRobot()
    patrol_robot.navigation_loop()  # Start navigation based on lidar data
