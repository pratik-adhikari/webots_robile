# supervisor.py

from controller import Supervisor
import time

# Constants
TIMESTEP = 32

class RobotSupervisor:
    def __init__(self):
        """Initialize the supervisor."""
        self.supervisor = Supervisor()
        self.leader_robot = self.supervisor.getFromDef("LEADER_ROBOT")
        if self.leader_robot is None:
            raise ValueError("Leader robot not found. Ensure the DEF name is correct.")
        self.translation_field = self.leader_robot.getField("translation")
    
    def get_leader_position(self):
        """Get the current position of the leader robot."""
        return self.translation_field.getSFVec3f()

    def move_leader_relative(self, delta_x, delta_y):
        """Move the leader robot to a position relative to its current position."""
        current_position = self.get_leader_position()
        new_position = [current_position[0] + delta_x, current_position[1] + delta_y, current_position[2]]
        self.translation_field.setSFVec3f(new_position)
        self.supervisor.simulationResetPhysics()

    def run(self):
        """Run the supervisor control loop."""
        while self.supervisor.step(TIMESTEP) != -1:
            user_input = input("Enter the target (x,y) relative position in meters (comma separated), or 'exit' to quit: ")
            if user_input.lower() == 'exit':
                break
            try:
                delta_x, delta_y = map(float, user_input.split(','))
                self.move_leader_relative(delta_x, delta_y)
            except ValueError:
                print("Invalid input. Please enter the target position as two comma-separated numbers.")
            time.sleep(0.1)  # Small delay to avoid busy-waiting

if __name__ == "__main__":
    supervisor = RobotSupervisor()
    supervisor.run()
