from controller import GPS, Accelerometer, Gyro
import math
import socket
import pickle
from base_drive_robot import BaseDriveRobot

# Constants
TIMESTEP = 32
TARGET_DISTANCE_THRESHOLD = 0.1  # Threshold to determine if the follower is close enough to the target
TARGET_ANGLE_THRESHOLD = 0.1  # Threshold to determine if the follower's orientation is close enough to the target orientation
DEFAULT_FORWARD_SPEED = 5.0  # Speed value


class FollowerRobot(BaseDriveRobot):
    def __init__(self, x_offset, y_offset, port):
        """Initialize the follower robot and its sensors."""
        super().__init__()
        self.x_offset = x_offset
        self.y_offset = y_offset
        self.port = port
        
        # Initialize the GPS devices
        self.gps_back = self.robot.getDevice("gps6")
        self.gps_back.enable(TIMESTEP)
        self.gps_front = self.robot.getDevice("gps2")
        self.gps_front.enable(TIMESTEP)
        self.wait_for_gps_initialization()

        # Initialize the accelerometer
        self.accelerometer = self.robot.getDevice("accelerometer")
        self.accelerometer.enable(TIMESTEP)

        # Initialize the gyro
        self.gyro = self.robot.getDevice("gyro")
        self.gyro.enable(TIMESTEP)

        # Socket to receive state from leader
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.client_socket.bind(('localhost', self.port))

    def wait_for_gps_initialization(self):
        """Wait until the GPS sensor provides valid data."""
        while self.robot.step(TIMESTEP) != -1:
            position_back = self.gps_back.getValues()
            position_front = self.gps_front.getValues()
            if not any(math.isnan(coord) for coord in position_back) and not any(math.isnan(coord) for coord in position_front):
                break

    def get_leader_state(self):
        """Receive leader state via UDP."""
        data, _ = self.client_socket.recvfrom(4096)
        leader_state = pickle.loads(data)
        return leader_state

    def follow_leader(self):
        """Follow the leader robot by adjusting the follower's position."""
        while self.robot.step(TIMESTEP) != -1:
            (leader_position_back, leader_position_front), leader_acceleration, leader_rotation = self.get_leader_state()
            
            target_position_back = [
                leader_position_back[0] + self.x_offset,
                leader_position_back[1] + self.y_offset,
                leader_position_back[2]
            ]

            current_position_back = self.gps_back.getValues()
            current_position_front = self.gps_front.getValues()

            distance_to_target_back_x = target_position_back[0] - current_position_back[0]
            distance_to_target_back_y = target_position_back[1] - current_position_back[1]

            print(f"Distance to target back (x, y): ({distance_to_target_back_x}, {distance_to_target_back_y})")

            yaw_difference = math.atan2(
                leader_position_front[0] - leader_position_back[0],
                leader_position_front[2] - leader_position_back[2]
            ) - math.atan2(
                current_position_front[0] - current_position_back[0],
                current_position_front[2] - current_position_back[2]
            )

            print(f"Yaw difference: {yaw_difference}")

            # Check if the follower is close enough to the target
            if abs(distance_to_target_back_x) < TARGET_DISTANCE_THRESHOLD and abs(distance_to_target_back_y) < TARGET_DISTANCE_THRESHOLD:
                print("Follower has reached the target position.")
                self.stop()
                continue  # Continue monitoring the position

            # Set the velocity and steering angle of the robot
            velocity = DEFAULT_FORWARD_SPEED
            steering_angle = math.degrees(yaw_difference)
            print(f"Setting velocity to {velocity} and steering angle to {steering_angle}")
            self.run(angle_degrees=steering_angle, speed=velocity)

if __name__ == "__main__":
    follower_robot = FollowerRobot(x_offset=-0.0, y_offset=-0.6, port=10001)
    follower_robot.follow_leader()
