from controller import Robot
import math

# Constants
TIMESTEP = 32
DEFAULT_FORWARD_SPEED = 10.0  # Default speed value

class DifferentialDriveRobot:
    def __init__(self):
        self.robot = Robot()
        
        # Initialize the drive motors (all four wheels are motorized for driving)
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

        # Set up the motors for continuous rotation, initially stopped
        for motor in self.drive_motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(0.0)

        # Set passive pivots for rear wheels
        for motor in self.rear_pivot_motors:
            motor.setPosition(0.0)

    def set_front_pivot_angle(self, angle_degrees):
        """Set the front pivot angles within limits."""
        angle_radians = angle_degrees * math.pi / 180  # Convert to radians
        for motor in self.front_pivot_motors:
            motor.setPosition(angle_radians)
        print(f"Front pivots set to {angle_degrees} degrees.")

    def run(self, angle_degrees=0, speed=DEFAULT_FORWARD_SPEED):
        """Run the robot with specified steering angle and speed."""
        while self.robot.step(TIMESTEP) != -1:
            # Set drive motor velocities to the specified forward speed
            for motor in self.drive_motors:
                motor.setVelocity(speed)
            
            # Set front steering based on the input angle
            self.set_front_pivot_angle(angle_degrees)
            
            print(f"Driving forward at {speed} m/s with front steering to {angle_degrees} degrees.")

if __name__ == "__main__":
    robot = DifferentialDriveRobot()
    robot.run()  # Will use default speed and 0 degrees angle
    # robot.run(15, 12)  # Optionally, provide specific steering angle and speed
