from controller import GPS, Accelerometer, Gyro, Lidar
import math
import socket
import pickle
from base_drive_robot import BaseDriveRobot

# Constants
TIMESTEP = 32
DEFAULT_FORWARD_SPEED = 3.0
FOLLOWER_PORTS = [10001, 10002]  # Add more ports as needed for additional followers
OBSTACLE_THRESHOLD_DISTANCE = 0.3  # Threshold distance for obstacle detection
LIDAR_FOV = 60  # Field of view for obstacle detection

class LeaderRobot(BaseDriveRobot):
    def __init__(self):
        """Initialize the leader robot and its sensors."""
        super().__init__()

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

        # Initialize the Lidar
        try:
            self.lidar = self.robot.getDevice('lidar')
            self.lidar.enable(TIMESTEP)
            self.lidar.enablePointCloud()
            print("Lidar initialized and enabled.")
        except Exception as e:
            print(f"Error initializing lidar: {e}")
            raise

        # Socket to send state to followers
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def wait_for_gps_initialization(self):
        """Wait until the GPS sensor provides valid data."""
        while self.robot.step(TIMESTEP) != -1:
            position_back = self.gps_back.getValues()
            position_front = self.gps_front.getValues()
            if not any(math.isnan(coord) for coord in position_back) and not any(math.isnan(coord) for coord in position_front):
                break

    def get_current_state(self):
        """Get the current state of the leader robot (positions, acceleration, rotation)."""
        position_back = self.gps_back.getValues()
        position_front = self.gps_front.getValues()
        acceleration = self.accelerometer.getValues()
        rotation = self.gyro.getValues()
        return (position_back, position_front), acceleration, rotation

    def send_state(self):
        """Send the current state of the robot to the followers."""
        state = self.get_current_state()
        data = pickle.dumps(state)
        for port in FOLLOWER_PORTS:
            self.server_socket.sendto(data, ('localhost', port))

    def check_for_obstacles(self):
        """Check for obstacles within a 60-degree field of view in front of the robot."""
        lidar_ranges = self.lidar.getRangeImage()
        total_ranges = len(lidar_ranges)
        fov_ranges = int((LIDAR_FOV / 360.0) * total_ranges)

        for i in range(-fov_ranges // 2, fov_ranges // 2):
            distance = lidar_ranges[i]
            if not math.isinf(distance) and distance < OBSTACLE_THRESHOLD_DISTANCE:
                print(f"Obstacle detected at distance: {distance} within index range: {i}")
                return True
        return False

    def move_straight(self):
        """Move the robot in a straight line until an obstacle is detected."""
        self.run(speed=DEFAULT_FORWARD_SPEED)
        while self.robot.step(TIMESTEP) != -1:
            self.send_state()
            if self.check_for_obstacles():
                self.stop()
                print("Obstacle detected, stopping the robot.")
                break

    def main_loop(self):
        """Main loop to keep the leader robot running and sending state."""
        while self.robot.step(TIMESTEP) != -1:
            self.send_state()

if __name__ == "__main__":
    leader_robot = LeaderRobot()
    leader_robot.move_straight()
    leader_robot.main_loop()
