## Controller

from controller import Robot
import math

# Constants
TIMESTEP = 32
DEFAULT_FORWARD_SPEED = 10.0  # Default speed value
REDUCED_SPEED = 1.0
SECTIONS_CLOSE = 3  # Sections for close distance
SECTIONS_MID = 7  # Sections for mid distance
SECTIONS_LONG = 13  # Sections for long distance
MIN_SAFE_DISTANCE_CLOSE = 0.8  # Minimum safe distance for close sections
MIN_SAFE_DISTANCE_MID = 0.5  # Minimum safe distance for mid sections
MIN_SAFE_DISTANCE_LONG = 4.0  # Minimum safe distance for long sections

class BaseDriveRobot:
    def __init__(self):
        """Initialize the robot and its motors."""
        try:
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
        except Exception as e:
            print(f"Error initializing robot: {e}")
            raise

    def set_front_pivot_angle(self, angle_degrees):
        """Set the front pivot angles within limits."""
        try:
            angle_radians = angle_degrees * math.pi / 180
            for motor in self.front_pivot_motors:
                motor.setPosition(angle_radians)
            print(f"Front pivots set to {angle_degrees} degrees.")
        except Exception as e:
            print(f"Error setting front pivot angle: {e}")

    def run(self, angle_degrees=0, speed=DEFAULT_FORWARD_SPEED):
        """Run the robot with specified steering angle and speed."""
        try:
            if angle_degrees is not None:
                for motor in self.drive_motors:
                    motor.setVelocity(speed)
                self.set_front_pivot_angle(angle_degrees)
                print(f"Driving forward at {speed} m/s with front steering to {angle_degrees} degrees.")
            else:
                print("Invalid direction degrees. Stopping the robot.")
                for motor in self.drive_motors:
                    motor.setVelocity(0.0)
                self.set_front_pivot_angle(0)
        except Exception as e:
            print(f"Error running the robot: {e}")


class PatrolRobot(BaseDriveRobot):
    def __init__(self):
        """Initialize the patrol robot with LIDAR."""
        super().__init__()
        try:
            self.lidar = self.robot.getDevice('lidar')
            self.lidar.enable(TIMESTEP)
            self.lidar.enablePointCloud()  # Ensure your lidar supports this feature
        except Exception as e:
            print(f"Error initializing lidar: {e}")
            raise

    def filter_lidar_data(self, lidar_ranges):
        """Filter lidar data to process relevant ranges."""
        filtered_ranges = []
        total_ranges = len(lidar_ranges)
        quarter_idx = total_ranges // 4
        three_quarters_idx = total_ranges * 3 // 4

        # Process fourth & first quadrant data
        for i in range(three_quarters_idx, total_ranges):
            filtered_ranges.append(lidar_ranges[i] if not math.isinf(lidar_ranges[i]) else 12.0)
        for i in range(quarter_idx):
            filtered_ranges.append(lidar_ranges[i] if not math.isinf(lidar_ranges[i]) else 12.0)

        return filtered_ranges

    def process_lidar_data(self, section_count):
        """Process lidar data and find minimum distances in each section."""
        try:
            lidar_ranges = self.lidar.getRangeImage()
            filtered_ranges = self.filter_lidar_data(lidar_ranges)
            section_width = len(filtered_ranges) // section_count
            section_min_distances = []

            for i in range(section_count):
                section = filtered_ranges[i * section_width: (i + 1) * section_width]
                min_distance = min(section, default=float('inf'))  # Handle empty sections safely
                section_min_distances.append(min_distance)

            return section_min_distances
        except Exception as e:
            print(f"Error processing lidar data: {e}")
            return []

    def process_lidar_data_cumulative(self, section_count):
        """Process lidar data and find cumulative distances in each group of sections."""
        try:
            lidar_ranges = self.lidar.getRangeImage()
            filtered_ranges = self.filter_lidar_data(lidar_ranges)
            group_cumulative_distances = []
            
            # Calculate the size of each section
            section_size = len(filtered_ranges) // section_count
    
            # Define groups based on section size
            groups = [
                range(0 * section_size, 2 * section_size),  # Group 1: First two sections
                range(2 * section_size, 5 * section_size),  # Group 2: Next three sections
                range(5 * section_size, 8 * section_size),  # Group 3: Next three sections
                range(8 * section_size, 11 * section_size), # Group 4: Next three sections
                range(11 * section_size, 13 * section_size) # Group 5: Last two sections
            ]
    
            for idx, group in enumerate(groups):
                group_values = [filtered_ranges[i] for i in group]
                cumulative_distance = sum(group_values)
                print(f"Group {idx + 1}: Cumulative {cumulative_distance}")
                group_cumulative_distances.append(cumulative_distance)
    
            print(f"Group cumulative distances: {group_cumulative_distances}")
            return group_cumulative_distances
        except Exception as e:
            print(f"Error processing lidar data: {e}")
            return []
    
    def is_valid_group(self, cumulative_distances):
        """Check if any of the 2nd, 3rd, or 4th group has the largest cumulative distance above a threshold."""
        valid_groups = [1, 2, 3]  # Indices of 2nd, 3rd, and 4th groups
        max_distance = max(cumulative_distances)

        for i in valid_groups:
            if cumulative_distances[i] == max_distance and cumulative_distances[i] >= 20:
                return True
        return False

    def find_safest_direction(self, min_distances, min_safe_distance, level):
        """Find the safest direction based on lidar data with safety checks."""
        try:
            max_distance = 0.0
            safest_section = 0

            for i, distance in enumerate(min_distances):
                if distance > max_distance:
                    max_distance = distance
                    safest_section = i

            if max_distance < min_safe_distance:
                print(f"Level {level} triggered: Section {safest_section + 1} with distance {max_distance:.2f}")
                return None  # No safe direction found for this level

            angle_per_section = 180 / len(min_distances)  # Total of 180 degrees span
            direction_degrees = (safest_section - len(min_distances) // 2) * angle_per_section
            direction_degrees = max(min(direction_degrees, 90), -90)

            print(f"Level {level} safe: Section {safest_section + 1} with distance {max_distance:.2f}, Angle: {direction_degrees:.2f} degrees")
            return direction_degrees
        except Exception as e:
            print(f"Error finding safest direction: {e}")
            return None

    def find_safest_direction_cumulative(self, cumulative_distances, level):
        """Find the safest direction based on cumulative distances in groups of sections."""
        try:
            max_cumulative_distance = 0.0
            safest_group = None

            for i, cumulative_distance in enumerate(cumulative_distances):
                if cumulative_distance > max_cumulative_distance and cumulative_distance >= 20:
                    max_cumulative_distance = cumulative_distance
                    safest_group = i

            if safest_group is None:
                print(f"Level {level} triggered but no valid group found.")
                return None

            group_to_angle = {
                0: -60,
                1: -30,
                2: 0,
                3: 30,
                4: 60
            }

            direction_degrees = group_to_angle[safest_group]

            print(f"Level {level} selected: Group {safest_group + 1} with cumulative distance {max_cumulative_distance:.2f}, Angle: {direction_degrees:.2f} degrees")
            return direction_degrees
        except Exception as e:
            print(f"Error finding safest direction by cumulative distance: {e}")
            return None

    def navigation_loop(self):
        """Control loop for navigating based on lidar data."""
        try:
            while self.robot.step(TIMESTEP) != -1:
                # Level 1: Close distance (2 meters) - 3 sections
                min_distances = self.process_lidar_data(SECTIONS_CLOSE)
                if any(distance < MIN_SAFE_DISTANCE_CLOSE for distance in min_distances):
                    direction_degrees = self.find_safest_direction(min_distances, MIN_SAFE_DISTANCE_CLOSE, 1)
                    if direction_degrees is not None:
                        self.run(angle_degrees=direction_degrees, speed=DEFAULT_FORWARD_SPEED)
                        continue

                # Level 2: Mid distance (5 meters) - 7 sections in 180 degrees
                min_distances = self.process_lidar_data(SECTIONS_MID)
                if any(distance < MIN_SAFE_DISTANCE_MID for distance in min_distances):
                    left_distance = sum(min_distances[:SECTIONS_MID // 2])
                    right_distance = sum(min_distances[SECTIONS_MID // 2:])
                    turn_angle = -70 if left_distance > right_distance else 70
                    print(f"Level 2 triggered: slowing down and rotating {turn_angle} degrees.")
                    self.run(angle_degrees=turn_angle, speed=REDUCED_SPEED)
                    
                    # Continuously rotate until Level 3 condition is satisfied
                    while True:
                        min_distances = self.process_lidar_data(SECTIONS_MID)
                        if all(distance >= MIN_SAFE_DISTANCE_MID for distance in min_distances):
                            cumulative_distances = self.process_lidar_data_cumulative(SECTIONS_LONG)
                            print(f"Level 3 triggered: Cumulative distances: {cumulative_distances}")
                            if self.is_valid_group(cumulative_distances):
                                direction_degrees = self.find_safest_direction_cumulative(cumulative_distances, 3)
                                if direction_degrees is not None:
                                    self.run(angle_degrees=direction_degrees, speed=DEFAULT_FORWARD_SPEED)
                                    break
                        self.run(angle_degrees=turn_angle, speed=REDUCED_SPEED)
                        if self.robot.step(TIMESTEP) == -1:
                            break

                # Level 3: Long distance (10 meters) - 13 sections in 180 degrees
                cumulative_distances = self.process_lidar_data_cumulative(SECTIONS_LONG)
                print(f"Level 3 triggered: Cumulative distances: {cumulative_distances}")
                direction_degrees = self.find_safest_direction_cumulative(cumulative_distances, 3)
                if direction_degrees is not None:
                    self.run(angle_degrees=direction_degrees, speed=DEFAULT_FORWARD_SPEED)
                
        except Exception as e:
            print(f"Error in navigation loop: {e}")

if __name__ == "__main__":
    try:
        patrol_robot = PatrolRobot()
        patrol_robot.navigation_loop()  # Start navigation based on lidar data
    except Exception as e:
        print(f"Error starting patrol robot: {e}")