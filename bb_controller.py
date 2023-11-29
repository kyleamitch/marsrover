"""bb_controller controller."""

from controller import Robot, Receiver

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

        self.light_on = False
        self.leave_obstacle_avoidance = False
        
        self.receiver = Receiver("receiver")
        self.receiver.enable(self.time_step)
        self.message = ""


    # This methods checks to see if the simullation is still running when the program is within while loops. - Reece
    def sim_check(self):
        # If the simulation has stopped, terminate bb_controller.py
        if (self.robot.step(self.time_step) == -1):
            exit()

    # This method delays the program execution by a specified amount of iterations, it acts as a time.sleep() substitute as this didn't work for some reason. - Reece
    def delay(self, value):
        i = 0
        while(i <= value):
            self.sim_check()
            i = i + 1

    # This method checks the beacon and updates the light_on variable to dictate what route the robot goes along. - Reece
    def check_illumination_levels(self):      
        for i in range(8):
            # If beacon detected on any light sensor, change light_on state
            if (self.light_sensors[i].getValue() == 0.0):
                self.light_on = True

    # This method is responsible for the robot following the line. - Reece
    def follow_line(self):
        
        #print("L: ", self.ground_sensors[0].getValue(), " C: ",  self.ground_sensors[1].getValue(), " R: ",  self.ground_sensors[2].getValue())     

        # No line
        if (self.ground_sensors[0].getValue() > 500 and self.ground_sensors[1].getValue() > 500 and self.ground_sensors[2].getValue() > 500):
            if (self.light_on == True):
                self.left_motor.setVelocity(0.1)
                self.right_motor.setVelocity(1)
            # else:
            #     self.left_motor.setVelocity(1)
            #     self.right_motor.setVelocity(0.1) 

        # No line on left, Turn right
        elif (self.ground_sensors[0].getValue() > 500):
            self.left_motor.setVelocity(1)
            self.right_motor.setVelocity(0.5)

        # No line on right, Turn left
        elif (self.ground_sensors[2].getValue() > 500):
            self.left_motor.setVelocity(0.5)
            self.right_motor.setVelocity(1)

        # Line in the middle, Go forward
        elif (self.ground_sensors[1].getValue() < 500):
            self.left_motor.setVelocity(1)
            self.right_motor.setVelocity(1)

    # This method is responsible for realigning the ground sensors with the line so that the follow_line() method can take over once the program breaks out of the loops in avoid_obstacles(). - Reece 
    def exit_obstacle_avoidance(self, i):
        if (i > 5 and self.ground_sensors[0].getValue() < 500 and self.ground_sensors[1].getValue() < 500 and self.ground_sensors[2].getValue() < 500):
            self.left_motor.setVelocity(1)
            self.right_motor.setVelocity(0.1)
            self.delay(100)
            self.leave_obstacle_avoidance = True

    # This method is responsible for avoiding obstacles on the line. - Reece
    def avoid_obstacles(self): 
        # Set threshold
        threshold = 200

        # If obstacle close to front:
        if (self.proximity_sensors[0].getValue() > threshold or self.proximity_sensors[7].getValue() > threshold):

            #print("obstacle detected!")

            # Spin right until the left side sensor is facing the obstacle,
            while (self.proximity_sensors[5].getValue() < threshold):
                self.sim_check()                

                #print("re-aligning left sensor by right turn...")
                self.left_motor.setVelocity(1)
                self.right_motor.setVelocity(-1)

            i = 0
            while(1):
                self.sim_check()
                self.exit_obstacle_avoidance(i)
                if (self.leave_obstacle_avoidance):
                    break

                # Spin left until the left side sensor is facing the obstacle, 
                while (self.proximity_sensors[5].getValue() < threshold):
                    self.sim_check()
                    self.exit_obstacle_avoidance(i)
                    if (self.leave_obstacle_avoidance):
                        break

                    #print("re-aligning left sensor by left turn...")
                    self.left_motor.setVelocity(-1)
                    self.right_motor.setVelocity(1)

                    # If the robot has undershot and is spinning too far left:
                    if (self.proximity_sensors[6].getValue() > threshold):
                        
                        # Correct by spinning right until the left side sensor is facing the obstacle,
                        while (self.proximity_sensors[5].getValue() < threshold):
                            self.sim_check()
                            self.exit_obstacle_avoidance(i)
                            if (self.leave_obstacle_avoidance):
                                break

                            #print("correcting far left undershoot...")
                            self.left_motor.setVelocity(1)
                            self.right_motor.setVelocity(-1)

                    # If the robot has overshot and is spinning too far right:
                    elif (self.proximity_sensors[4].getValue() > threshold):

                        # Correct by spinning left until the left side sensor is facing the obstacle,
                        while (self.proximity_sensors[5].getValue() < threshold):
                            self.sim_check()
                            self.exit_obstacle_avoidance(i)
                            if (self.leave_obstacle_avoidance):
                                break

                            #print("correcting far right overshoot...")
                            self.left_motor.setVelocity(-1)
                            self.right_motor.setVelocity(1)

                # Move forward until the left side sensor is no longer facing the obstacle.
                while (self.proximity_sensors[5].getValue() > threshold):
                    self.sim_check()
                    self.exit_obstacle_avoidance(i)
                    if (self.leave_obstacle_avoidance):
                       break

                    #print("scooting along the obstacle...")
                    self.left_motor.setVelocity(1)
                    self.right_motor.setVelocity(1)

                i = i + 1

    # This is the main robot control loop. - Reece
    def run_bb_controller(self):
        while self.robot.step(self.time_step) != -1:
            
            self.follow_line()

            self.check_illumination_levels()    
              
            self.follow_line()

            if (self.receiver.getQueueLength() > 0):
                self.message = self.receiver.getString()
                self.receiver.nextPacket()

            # Once the robot approches the final destination, don't avoid obstacles so it docks within the port
            if (self.message == "E"):
                self.avoid_obstacles()
                self.leave_obstacle_avoidance = False
            

if __name__ == "__main__":
    my_robot = Robot()
    controller = Controller(my_robot)
    controller.run_bb_controller()
    