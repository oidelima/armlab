import time
import numpy as np
import camera_cal


"""
TODO: Add states and state functions to this class
        to implement all of the required logic for the armlab
"""
class StateMachine():
    def __init__(self, rexarm, planner, kinect):
        self.rexarm = rexarm
        self.tp = planner
        self.kinect = kinect
        self.status_message = "State: Idle"
        self.current_state = "idle"
        self.next_state = "idle"
        self.prev_state = "idle"
        self.waypoints = []


    def set_next_state(self, state):
        self.next_state = state

    """ This function is run continuously in a thread"""

    def run(self):
        
        if(self.current_state == "manual"):
            self.prev_state = "manual"
            if (self.next_state == "manual"):
                self.manual()
            if(self.next_state == "idle"):
                self.idle()                
            if(self.next_state == "estop"):
                self.estop()
            if(self.next_state == "execute"):
                self.execute()
            if(self.next_state == "record"):
                self.record()
            if(self.next_state == "calibrate"):
                self.calibrate()

        if(self.current_state == "execute"):
            self.prev_state = "execute"
            if (self.next_state == "idle"):
                self.idle()
            if(self.next_state == "estop"):
                self.estop()
            if(self.next_state == "manual"):
                self.manual()

        if(self.current_state == "idle"):
            self.prev_state = "idle"
            if(self.next_state == "manual"):
                self.manual()
            if(self.next_state == "idle"):
                self.idle()
            if(self.next_state == "estop"):
                self.estop()
            if(self.next_state == "calibrate"):
                self.calibrate()
            if(self.next_state == "execute"):
                self.execute()
        
        if(self.current_state == "record"):
            self.prev_state = "record"
            if(self.next_state == "manual"):
                self.manual()
            if(self.next_state == "record"):
                self.record()
            
                
        if(self.current_state == "estop"):
            self.prev_state = "estop"
            self.next_state = "estop"
            self.estop()  

        if(self.current_state == "calibrate"):
            self.prev_state = "calibrate"
            if(self.next_state == "idle"):
                self.idle()
            if(self.next_state == "manual"):
                self.manual()

    """Functions run for each state"""


    def manual(self):
        self.status_message = "State: Manual - Use sliders to control arm"
        self.current_state = "manual"
        self.rexarm.send_commands()
        self.rexarm.get_feedback()

    def idle(self):
        self.status_message = "State: Idle - Waiting for input"
        self.current_state = "idle"
        self.rexarm.get_feedback()

    def estop(self):
        self.status_message = "EMERGENCY STOP - Check Rexarm and restart program"
        self.current_state = "estop"
        self.rexarm.disable_torque()
        self.rexarm.get_feedback()

    def execute(self):
        self.status_message = "Executing ..."
        self.current_state = "execute"
        self.rexarm.get_feedback()
        print("Execute: ",self.waypoints)
        for i in range(len(self.waypoints)):
            [q, v]= self.tp.generate_cubic_spline(self.rexarm.get_positions(), self.waypoints[i], 4)

            self.tp.execute_plan([q,v])

        if self.prev_state == "manual":
            self.set_next_state("manual")
        else:
            self.set_next_state("idle")
        

    def record(self):
        
        self.current_state = "record"
        print("get positions : ",self.rexarm.get_positions())
        self.waypoints.append(self.rexarm.get_positions()[:]) 
        self.status_message = "Recording waypoint: " + str(self.rexarm.get_positions())
        print(self.rexarm.get_positions())
        print("Waypoint: ", self.waypoints)
        self.set_next_state("manual")

   

        
    def calibrate(self):
        self.current_state = "calibrate"
        #self.next_state = "idle"
        if self.prev_state == "manual":
            self.set_next_state("manual")
        else:
            self.set_next_state("idle")


        self.tp.go(max_speed=2.0)
        location_strings = ["lower left corner of board",
                            "upper left corner of board",
                            "upper right corner of board",
                            "lower right corner of board",
                            "center of shoulder motor"]
        i = 0
        for j in range(5):
            self.status_message = "Calibration - Click %s in RGB image" % location_strings[j]
            while (i <= j):
                self.rexarm.get_feedback()
                if(self.kinect.new_click == True):
                    self.kinect.rgb_click_points[i] = self.kinect.last_click.copy()
                    i = i + 1
                    self.kinect.new_click = False        
        
        i = 0
        for j in range(5):
            self.status_message = "Calibration - Click %s in depth image" % location_strings[j]
            while (i <= j):
               self.rexarm.get_feedback()
               if(self.kinect.new_click == True):
                    self.kinect.depth_click_points[i] = self.kinect.last_click.copy()
		    i = i + 1
                    self.kinect.new_click = False

        
   
        print(self.kinect.rgb_click_points)
        print(self.kinect.depth_click_points)
	
        """TODO Perform camera calibration here"""
        affine_transform = self.kinect.getAffineTransform(self.kinect.rgb_click_points,self.kinect.depth_click_points)

	self.kinect.kinectCalibrated = True
        self.status_message = "Calibration - Completed Calibration"
        time.sleep(1)
