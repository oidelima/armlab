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
        #initial_wp = [0.023782507988034496, 0.6068375547914533, 0.45340201615897424, -0.0025591338005868103, -0.007677401401759543]
        #final_wp = [-1.9463298080529916, 0.6467307948358973, 0.5500664054974358, 0.002559133800586366, -0.0025591338005868103]
        a = np.zeros((len(initial_wp), 4))
        for joint_num in range(len(initial_wp)):
            M = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [1, T, T**2, T**3], [0, 1, 2*T, 3*T**2]])
            b = np.array([initial_wp[joint_num], 0, final_wp[joint_num], 0])
            a[joint_num] = np.matmul(np.linalg.inv(M),b)
        t = np.linspace(0,T)
        q = np.zeros((len(t),len(initial_wp)))
        v = np.zeros((len(t),len(initial_wp)))
        for time in range(len(t)):
            q[time]= [a[0][0]+a[0][1]*t[time]+a[0][2]*t[time]**2+a[0][3]*t[time]**3,
                    a[1][0]+a[1][1]*t[time]+a[1][2]*t[time]**2+a[1][3]*t[time]**3,
                    a[2][0]+a[2][1]*t[time]+a[2][2]*t[time]**2+a[2][3]*t[time]**3,
                    a[3][0]+a[3][1]*t[time]+a[3][2]*t[time]**2+a[3][3]*t[time]**3,
                    a[4][0]+a[4][1]*t[time]+a[4][2]*t[time]**2+a[4][3]*t[time]**3]
            
            v[time]= [a[0][1]+2*a[0][2]*t[time]+3*a[0][3]*t[time]**2,
                    a[1][1]+2*a[1][2]*t[time]+3*a[1][3]*t[time]**2,
                    a[2][1]+2*a[2][2]*t[time]+3*a[2][3]*t[time]**2,
                    a[3][1]+2*a[3][2]*t[time]+3*a[3][3]*t[time]**2,
                    a[4][1]+2*a[4][2]*t[time]+3*a[4][3]*t[time]**2]
                              


        
        '''import matplotlib.pyplot as plt
        plt.plot(t, q)
        plt.show()
        plt.plot(t, v)
        plt.show()'''
        #print(M)
        #print(b.transpose())
        #print(np.matmul(np.linalg.inv(M),b))
        #print(np.matmul(np.linalg.inv(M),b.transpose()))
        return q, v
    
    def execute_plan(self, plan, look_ahead=8):
        q = plan[0]
        v = plan[1]
        for step in range(len(q)):
            target_pos=q[step]
            max_speed = v[step]
            self.rexarm.set_positions(target_pos)
            self.rexarm.set_speeds(max_speed)
            self.rexarm.pause(1)

