# leader_robot.py

from controller import Robot, GPS
import math

# Constants
TIMESTEP = 32
DEFAULT_FORWARD_SPEED = 10.0  # Default speed value

class LeaderRobot:
    def __init__(self):
        """Initialize the robot and its motors."""
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

        # Initialize the GPS
        self.gps = self.robot.getDevice("gps")
        self.gps.enable(TIMESTEP)

    def set_front_pivot_angle(self, angle_degrees):
        """Set the front pivot angles within limits."""
        angle_radians = angle_degrees * math.pi / 180
        for motor in self.front_pivot_motors:
            motor.setPosition(angle_radians)

    def run(self, angle_degrees=0, speed=DEFAULT_FORWARD_SPEED):
        """Run the robot with specified steering angle and speed."""
        for motor in self.drive_motors:
            motor.setVelocity(speed)
        self.set_front_pivot_angle(angle_degrees)

    def move_to_goal(self, goal_position):
        """Move the robot to the goal position using GPS."""
        while self.robot.step(TIMESTEP) != -1:
            current_position = self.gps.getValues()
            distance_to_goal = math.sqrt((goal_position[0] - current_position[0])**2 + 
                                         (goal_position[1] - current_position[1])**2 + 
                                         (goal_position[2] - current_position[2])**2)
            if distance_to_goal < 0.1:  # Stop if within 0.1 meters of the goal
                self.run(angle_degrees=0, speed=0)
                break
            
            # Simple proportional controller to move towards goal
            angle_to_goal = math.atan2(goal_position[1] - current_position[1], 
                                       goal_position[0] - current_position[0]) * 180 / math.pi
            self.run(angle_degrees=angle_to_goal, speed=DEFAULT_FORWARD_SPEED)

if __name__ == "__main__":
    leader_robot = LeaderRobot()
    
    current_position = leader_robot.gps.getValues()
    print(f"Current GPS Position: {current_position}")
    
    # Example goal position, replace with user input if needed
    goal_position = [5.0, 0.0, 0.0]  # Assuming a 2D plane (x, y, z)

    leader_robot.move_to_goal(goal_position)
