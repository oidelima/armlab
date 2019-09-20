import numpy as np 
import time
from rexarm import Rexarm

"""
TODO: build a trajectory generator and https://docs.python.org/2/library/math.htmlwaypoint planner 
        so it allows your state machine to iterate through
        the plan at the desired command update rate
"""

class TrajectoryPlanner():
    def __init__(self, rexarm):
        self.idle = True
        self.rexarm = rexarm
        self.num_joints = rexarm.num_joints
        self.initial_wp = [0.0]*self.num_joints
        self.final_wp = [0.0]*self.num_joints 
        self.dt = 0.05 # command rate

    
    def set_initial_wp(self):
        self.initial_wp = self.rexarm.get_positions()

    def set_final_wp(self, waypoint):
        self.final_wp = waypoint

    def go(self, max_speed = 2.5):
        pass

    def stop(self):
        for joint in self.rexarm.joints:
            joint.set_speed(0)

    def calc_time_from_waypoints(self, initial_wp, final_wp, max_speed):
        pass

    def generate_cubic_spline(self, initial_wp, final_wp, T):
        a = np.zeros((len(initial_wp), 4))
        for joint_num in range(len(initial_wp)):
            M = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [1, T, T**2, T**3], [0, 1, 2*T, 3*T**2]])
            b = np.array([initial_wp[joint_num], 0, final_wp[joint_num], 0])
            a[joint_num] = np.matmul(np.linalg.inv(M),b)
        t = np.linspace(0,T, num = T//self.dt)
        q = np.zeros((len(t),len(initial_wp)))
        v = np.zeros((len(t),len(initial_wp)))
        for time in range(len(t)):
            q[time]= [a[i][0]+a[i][1]*t[time]+a[i][2]*t[time]**2+a[i][3]*t[time]**3 for i in range(len(initial_wp))]           
            v[time]= [a[i][1]+2*a[i][2]*t[time]+3*a[i][3]*t[time]**2 for i in range(len(initial_wp))]


        '''import matplotlib.pyplot as plt
        plt.plot(t, q)
        plt.show()
        plt.plot(t, v)
        plt.show()'''

        return q, v
    
    def execute_plan(self, plan, look_ahead=25):
        q = plan[0]
        v = plan[1]
        
        for step in range(len(q)):
            if step < len(q) - look_ahead:
                target_pos=q[step + look_ahead]
            else:
                target_pos=q[-1]
            max_speed = v[step]
            self.rexarm.set_positions(target_pos)
            self.rexarm.set_speeds(max_speed)
            self.rexarm.pause(self.dt)

