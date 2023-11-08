"""bb_controller controller."""

from controller import Robot
from datetime import datetime
import math
import numpy as np

class Controller:
    def __init__(self, robot):        
        # taken from e-puck_line_lab1.py -------------------------------------------------------
        # Robot Parameters
        self.robot = robot
        self.time_step = 32 # ms
        self.max_speed = 1  # m/s
 
        # Enable Motors
        self.left_motor = self.robot.getDevice('left wheel motor')
        self.right_motor = self.robot.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        self.velocity_left = 0
        self.velocity_right = 0
    
        # Enable Proximity Sensors
        self.proximity_sensors = []
        for i in range(8):
            sensor_name = 'ps' + str(i)
            self.proximity_sensors.append(self.robot.getDevice(sensor_name))
            self.proximity_sensors[i].enable(self.time_step)
        # taken from e-puck_line_lab1.py -------------------------------------------------------
       
        # Enable Ground Sensors
        self.ground_sensors = [] # left, center, right
        for i in range(3):
            sensor_name = "gs" + str(i)
            self.ground_sensors.append(self.robot.getDevice(sensor_name))
            self.ground_sensors[i].enable(self.time_step)

        # taken from e-puck_light_lab2.py -------------------------------------------------------     
        self.light_sensors = []
        for i in range(8):
            sensor_name = "ls" + str(i)
            self.light_sensors.append(self.robot.getDevice(sensor_name))
            self.light_sensors[i].enable(self.time_step)
        # taken from e-puck_light_lab2.py -------------------------------------------------------

    def read_light_sensors(self):
        for i in range(8):
            print("ls" + str(i) + ": " , self.light_sensors[i].getValue())

    def read_distance_sensors(self):
        for i in range(8):
            print("ps" + str(i) + ": " , self.proximity_sensors[i].getValue())

    def read_ground_sensors(self):
        # for i in range(3):
        #     print("gs" + str(i) + ": " , self.ground_sensors[i].getValue())
        self.ground_values = []
        for i in range(len(self.ground_sensors)):
            self.ground_values.append(self.ground_sensors[i].getValue())
        print(self.ground_values)

    def read_all_sensors(self):
        while self.robot.step(self.time_step) != -1:
            self.read_distance_sensors()
            print(" ")
            self.read_ground_sensors()
            print(" ")
            self.read_light_sensors()

    def follow_line(self):
        while self.robot.step(self.time_step) != -1:
            self.read_ground_sensors()

            if (self.ground_values[0] > 500):
                self.velocity_left = 1
                self.velocity_right = 0.5

            elif (self.ground_values[2] > 500):
                self.velocity_left = 0.5
                self.velocity_right = 1

            elif (self.ground_values[1] < 500):
                self.velocity_left = 1
                self.velocity_right = 1

            self.left_motor.setVelocity(self.velocity_left)
            self.right_motor.setVelocity(self.velocity_right)

if __name__ == "__main__":
    my_robot = Robot()
    controller = Controller(my_robot)
    controller.follow_line()
    # controller.read_ground_sensors()
    