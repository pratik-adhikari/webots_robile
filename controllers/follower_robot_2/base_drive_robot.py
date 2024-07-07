from controller import Robot
import math

# Constants
TIMESTEP = 32
DEFAULT_FORWARD_SPEED = 5.0  # Speed value

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
            # print(f"Front pivots set to {angle_degrees} degrees.")
        except Exception as e:
            print(f"Error setting front pivot angle: {e}")

    def run(self, angle_degrees=0, speed=DEFAULT_FORWARD_SPEED):
        """Run the robot with specified steering angle and speed."""
        try:
            if angle_degrees is not None:
                for motor in self.drive_motors:
                    motor.setVelocity(speed)
                self.set_front_pivot_angle(angle_degrees)
                # print(f"Driving forward at {speed} m/s with front steering to {angle_degrees} degrees.")
            else:
                print("Invalid direction degrees. Stopping the robot.")
                for motor in self.drive_motors:
                    motor.setVelocity(0.0)
                self.set_front_pivot_angle(0)
        except Exception as e:
            print(f"Error running the robot: {e}")

    def stop(self):
        """Stop the robot."""
        for motor in self.drive_motors:
            motor.setVelocity(0.0)
        print("Robot stopped.")
