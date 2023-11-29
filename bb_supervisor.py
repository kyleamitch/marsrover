"""bb_supervisor controller."""

from controller import Supervisor, Emitter
import sys

class SupervisorRobot:
    def __init__(self):
        # taken from supervisor_lab2.py -------------------------------------------------------

        # Simulation Parameters
        self.time_step = 32 # (ms)
        # self.time_light = 20 # (s)
        # self.flag_light = 1 # You can use the flag to identify the current position of the light node
        
        # Initiate Supervisor Module
        self.supervisor = Supervisor()
        # Get the robot node from your world environment
        self.robot_node = self.supervisor.getFromDef("Controller")
        # Check if the robot node exists 
        if self.robot_node is None:
            sys.stderr.write("No DEF Controller node found in the current world file\n")
            sys.exit(1)  

        # taken from supervisor_lab2.py -------------------------------------------------------  

        self.emitter = Emitter("emitter")

    def run(self):
        while self.supervisor.step(self.time_step) != -1:
            self.robot_position = self.robot_node.getPosition()
            #print(self.robot_position[1]) 
            if (self.robot_position[1] > 0.74):
                self.emitter.send('D')
                print("D")
            else:
                self.emitter.send('E')
                print("E")
        
if __name__ == "__main__":
    # Create Supervisor Controller
    model = SupervisorRobot()
    # Run Supervisor Controller
    model.run()    