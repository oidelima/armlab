#!/usr/bin/python3
import sys
import cv2
import numpy as np
import time
from functools import partial

from PyQt4.QtCore import (QThread, Qt, pyqtSignal, pyqtSlot, QTimer)
from PyQt4.QtGui import (QPixmap, QImage, QApplication, QWidget, QLabel, QMainWindow, QCursor)

import os
os.sys.path.append('dynamixel/') # Path setting
os.system('cls' if os.name == 'nt' else 'clear')
os.system('cls' if os.name == 'nt' else 'clear')
os.system('cls' if os.name == 'nt' else 'clear')
from dynamixel_XL import *
from dynamixel_AX import *
from dynamixel_MX import *
from dynamixel_bus import *

from ui import Ui_MainWindow
from rexarm import Rexarm
from kinect import Kinect
from trajectory_planner import TrajectoryPlanner
from state_machine import StateMachine


""" Radians to/from  Degrees conversions """
D2R = 3.141592/180.0
R2D = 180.0/3.141592
#pixel Positions of image in GUI
MIN_X = 240
MAX_X = 880
MIN_Y = 40
MAX_Y = 520

""" Serial Port Parameters"""
BAUDRATE   = 1000000
DEVICENAME = "/dev/ttyACM0".encode('utf-8')

"""Threads"""
class VideoThread(QThread):
	updateFrame = pyqtSignal(QImage, QImage, QImage, QImage, QImage, QImage)

	def __init__(self, kinect, parent=None):
		QThread.__init__(self, parent=parent) 
		self.kinect = kinect

	def run(self):
		while True:
			self.kinect.captureVideoFrame()
			self.kinect.captureDepthFrame()
			rgb_frame = self.kinect.convertFrame()
			depth_frame = self.kinect.convertDepthFrame()
			block_depth_frame = self.kinect.convertBlockDepthFrame()
			hsv_frame = self.kinect.convertHSVFrame()
			self.updateFrame.emit(rgb_frame, depth_frame, block_depth_frame, hsv_frame[0], hsv_frame[1], hsv_frame[2])

class LogicThread(QThread):   
	def __init__(self, state_machine, parent=None):
		QThread.__init__(self, parent=parent) 
		self.sm=state_machine

	def run(self):
		while True:    
			self.sm.run()
			time.sleep(0.05)

class DisplayThread(QThread):
	updateStatusMessage = pyqtSignal(str)
	updateJointReadout = pyqtSignal(list)
	updateEndEffectorReadout = pyqtSignal(list)

	def __init__(self, rexarm, state_machine, parent=None):
		QThread.__init__(self, parent=parent) 
		self.rexarm = rexarm
		self.sm=state_machine

	def run(self):
		while True:
			self.updateStatusMessage.emit(self.sm.status_message)
			self.updateJointReadout.emit(self.rexarm.joint_angles_fb)
			self.updateEndEffectorReadout.emit(self.rexarm.get_wrist_pose())    
			time.sleep(0.1)
	
"""GUI Class"""
class Gui(QMainWindow):
	""" 
	Main GUI Class
	contains the main function and interfaces between 
	the GUI and functions
	"""
	def __init__(self,parent=None):
		QWidget.__init__(self,parent)
		self.ui = Ui_MainWindow()
		self.ui.setupUi(self)

		""" Set GUI to track mouse """
		QWidget.setMouseTracking(self,True)

		"""
		Dynamixel bus
		TODO: add other motors here as needed with their correct address"""
		self.dxlbus = DXL_BUS(DEVICENAME, BAUDRATE)

		port_num = self.dxlbus.port()
		base = DXL_MX(port_num, 1)
		shld = DXL_MX(port_num, 2)
		elbw = DXL_MX(port_num, 3)
		wrst = DXL_AX(port_num, 4)
		wrst2 = DXL_AX(port_num, 5)
		#wrst3 = DXL_XL(port_num, 6)
		#grip = DXL_XL(port_num, 7)
		#wrst2.set_compliance(8,64)

		"""Objects Using Other Classes"""
		self.kinect = Kinect()
		self.rexarm = Rexarm((base,shld,elbw,wrst,wrst2),0)
		self.tp = TrajectoryPlanner(self.rexarm)
		self.sm = StateMachine(self.rexarm, self.tp, self.kinect)
	
		""" 
		Attach Functions to Buttons & Sliders
		TODO: NAME AND CONNECT BUTTONS AS NEEDED
		"""
		self.ui.btn_estop.clicked.connect(self.estop)
		self.ui.btn_exec.clicked.connect(self.execute)
		self.ui.btn_task1.clicked.connect(self.record) 
		self.ui.btn_task2.clicked.connect(self.clear_waypoints) 
		self.ui.btn_task3.clicked.connect(self.toggle_gripper) 
		self.ui.btnUser1.setText("Calibrate")
		self.ui.btnUser1.clicked.connect(partial(self.sm.set_next_state, "calibrate"))
		self.ui.btnUser2.setText("Block Detector")
		self.ui.btnUser2.clicked.connect(partial(self.sm.set_next_state, "block detection"))
		self.ui.btnUser2.setText("Color Buckets")
		self.ui.btnUser2.clicked.connect(partial(self.sm.set_next_state, "color buckets"))
		self.ui.btnUser3.setText("Click Grab/Drop Mode")
		self.ui.btnUser3.clicked.connect(partial(self.sm.set_next_state, "click grab drop"))
		self.ui.btnUser4.setText("Pick n' Stack")
		self.ui.btnUser4.clicked.connect(partial(self.sm.set_next_state, "pick and stack"))
		self.ui.btnUser5.setText("Line 'em up")
		self.ui.btnUser5.clicked.connect(partial(self.sm.set_next_state, "line them up"))
		self.ui.btnUser6.setText("Stack 'em high")
		self.ui.btnUser6.clicked.connect(partial(self.sm.set_next_state, "stack them high"))
		self.ui.btnUser7.setText("Block slider")
		self.ui.btnUser7.clicked.connect(partial(self.sm.set_next_state, "block slider"))
		self.ui.btnUser8.setText("Hot swap")
		self.ui.btnUser8.clicked.connect(partial(self.sm.set_next_state, "hot swap"))
		self.ui.sldrBase.valueChanged.connect(self.sliderChange)
		self.ui.sldrShoulder.valueChanged.connect(self.sliderChange)
		self.ui.sldrElbow.valueChanged.connect(self.sliderChange)
		self.ui.sldrWrist.valueChanged.connect(self.sliderChange)

		self.ui.sldrWrist2.valueChanged.connect(self.sliderChange)
		self.ui.sldrWrist3.valueChanged.connect(self.sliderChange)
		self.ui.sldrGrip1.valueChanged.connect(self.sliderChange)

		self.ui.sldrMaxTorque.valueChanged.connect(self.sliderChange)
		self.ui.sldrSpeed.valueChanged.connect(self.sliderChange)
		self.ui.chk_directcontrol.stateChanged.connect(self.directControlChk)
		self.ui.rdoutStatus.setText("Waiting for input")

		"""initalize manual control off"""
		self.ui.SliderFrame.setEnabled(False)

		"""initalize rexarm"""
		self.rexarm.initialize()

		"""Setup Threads"""
		self.videoThread = VideoThread(self.kinect)
		self.videoThread.updateFrame.connect(self.setImage)        
		self.videoThread.start()

		
		self.logicThread = LogicThread(self.sm)
		self.logicThread.start()
		

		self.displayThread = DisplayThread(self.rexarm, self.sm)
		self.displayThread.updateJointReadout.connect(self.updateJointReadout)
		self.displayThread.updateEndEffectorReadout.connect(self.updateEndEffectorReadout)
		self.displayThread.updateStatusMessage.connect(self.updateStatusMessage)
		self.displayThread.start()

		""" 
		Setup Timer 
		this runs the trackMouse function every 50ms
		"""
		self._timer = QTimer(self)
		self._timer.timeout.connect(self.trackMouse)
		self._timer.start(50)

	""" Slots attach callback functions to signals emitted from threads"""

	@pyqtSlot(QImage, QImage, QImage, QImage, QImage, QImage)
	def setImage(self, rgb_image, depth_image, blocks_depth_image, h_image, s_image, v_image):
		if(self.ui.radioVideo.isChecked()):
			self.ui.videoDisplay.setPixmap(QPixmap.fromImage(rgb_image))
		if(self.ui.radioDepth.isChecked()):
			self.ui.videoDisplay.setPixmap(QPixmap.fromImage(depth_image))
		if(self.ui.radioUsr1.isChecked()):
			self.ui.videoDisplay.setPixmap(QPixmap.fromImage(blocks_depth_image))
		if(self.ui.radioUsr2.isChecked()):
			self.ui.videoDisplay.setPixmap(QPixmap.fromImage(h_image))
		if(self.ui.radioUsr3.isChecked()):
			self.ui.videoDisplay.setPixmap(QPixmap.fromImage(s_image))
		if(self.ui.radioUsr4.isChecked()):
			self.ui.videoDisplay.setPixmap(QPixmap.fromImage(v_image))


	@pyqtSlot(list)
	def updateJointReadout(self, joints):
		self.ui.rdoutBaseJC.setText(str("%+.2f" % (joints[0]*R2D)))
		self.ui.rdoutShoulderJC.setText(str("%+.2f" % ((joints[1]*R2D)+90.0)))
		self.ui.rdoutElbowJC.setText(str("%+.2f" % (joints[2]*R2D)))
		self.ui.rdoutWristJC.setText(str("%+.2f" % (joints[3]*R2D)))
		self.ui.rdoutWrist2JC.setText(str("%+.2f" % (joints[4]*R2D)))

		if(len(joints)>5):
			self.ui.rdoutWrist3JC.setText(str("%+.2f" % (joints[5]*R2D)))

		else:
			self.ui.rdoutWrist3JC.setText(str("N.A."))

	@pyqtSlot(list)
	def updateEndEffectorReadout(self, pos):
		self.ui.rdoutX.setText(str("%+.2f" % (pos[0])))
		self.ui.rdoutY.setText(str("%+.2f" % (pos[1])))
		self.ui.rdoutZ.setText(str("%+.2f" % (pos[2])))
		self.ui.rdoutT.setText(str("%+.2f" % (pos[3])))
		self.ui.rdoutG.setText(str("%+.2f" % (pos[4])))
		self.ui.rdoutP.setText(str("%+.2f" % (pos[5])))

	@pyqtSlot(str)
	def updateStatusMessage(self, msg):
		self.ui.rdoutStatus.setText(msg)


	""" Other callback functions attached to GUI elements"""

	def estop(self):
		self.rexarm.estop = True
		self.sm.set_next_state("estop")

	def execute(self):
		self.sm.set_next_state("execute")
		self.ui.sldrMaxTorque.setValue(50)

	def toggle_gripper(self):
		self.rexarm.toggle_gripper()
		#self.rexarm.pause(1)

	def record(self):
		self.sm.set_next_state("record")

	def clear_waypoints(self):
		self.sm.waypoints = []
		

	def sliderChange(self):
		""" 
		Function to change the slider labels when sliders are moved
		and to command the arm to the given position
		"""
		self.ui.rdoutBase.setText(str(self.ui.sldrBase.value()))
		self.ui.rdoutShoulder.setText(str(self.ui.sldrShoulder.value()))
		self.ui.rdoutElbow.setText(str(self.ui.sldrElbow.value()))
		self.ui.rdoutWrist.setText(str(self.ui.sldrWrist.value()))

		self.ui.rdoutWrist2.setText(str(self.ui.sldrWrist2.value()))

		self.ui.rdoutTorq.setText(str(self.ui.sldrMaxTorque.value()) + "%")
		self.ui.rdoutSpeed.setText(str(self.ui.sldrSpeed.value()) + "%")
		self.rexarm.set_torque_limits([self.ui.sldrMaxTorque.value()/100.0]*self.rexarm.num_joints, update_now = False)
		self.rexarm.set_speeds_normalized_global(self.ui.sldrSpeed.value()/100.0, update_now = False)
		joint_positions = np.array([self.ui.sldrBase.value()*D2R, 
						   self.ui.sldrShoulder.value()*D2R,
						   self.ui.sldrElbow.value()*D2R,
						   self.ui.sldrWrist.value()*D2R,
						   self.ui.sldrWrist2.value()*D2R,
						   self.ui.sldrWrist3.value()*D2R])
		self.rexarm.set_positions(joint_positions, update_now = False)

	def directControlChk(self, state):
		if state == Qt.Checked:
			self.sm.set_next_state("manual")
			self.ui.SliderFrame.setEnabled(True)
			self.ui.sldrMaxTorque.setValue(0)
		else:
			self.sm.set_next_state("idle")
			self.ui.SliderFrame.setEnabled(False)
			self.ui.sldrMaxTorque.setValue(50)

	def trackMouse(self):
		""" 
		Mouse position presentation in GUI
		TODO: after implementing workspace calibration 
		display the world coordinates the mouse points to 
		in the RGB video image.
		"""
		x = QWidget.mapFromGlobal(self,QCursor.pos()).x()
		y = QWidget.mapFromGlobal(self,QCursor.pos()).y()
		if ((x < MIN_X) or (x >= MAX_X) or (y < MIN_Y) or (y >= MAX_Y)):
			self.ui.rdoutMousePixels.setText("(-,-,-)")
			self.ui.rdoutMouseWorld.setText("(-,-,-)")
		else:
			x = x - MIN_X
			y = y - MIN_Y
			if(self.kinect.currentDepthFrame.any() != 0):			
				z = self.kinect.currentDepthFrame[y][x]
				self.ui.rdoutMousePixels.setText("(%.0f,%.0f,%.0f)" % (x,y,z))
				camera_coordinates = np.array([[x],[y],[1]]).astype(np.float32)
				xy_world = np.matmul(self.kinect.workcamera_affine,camera_coordinates) 
				z_w = .1236*np.tan(z/2842.5 + 1.1863) - 0.94
				self.ui.rdoutMouseWorld.setText("(%.2f,%.2f,%.2f)" % (xy_world[0], xy_world[1], z_w))

	def mousePressEvent(self, QMouseEvent):
		""" 
		Function used to record mouse click positions for calibration 
		"""

		""" Get mouse posiiton """
		#print("mouse event")
		x = QMouseEvent.x()
		y = QMouseEvent.y()

		""" If mouse position is not over the camera image ignore """
		if ((x < MIN_X) or (x > MAX_X) or (y < MIN_Y) or (y > MAX_Y)): return
		if self.sm.current_state == "click grab drop":
			if not self.sm.grab_position:
				self.sm.grab_position = (x,y)
				print("Have grab position")
				return
			if self.sm.grab_position:
				self.sm.drop_position = (x,y)
				print("Have drop position")
		if self.sm.current_state == "calibrate":
			""" Change coordinates to image axis """
			self.kinect.last_click[0] = x - MIN_X 
			self.kinect.last_click[1] = y - MIN_Y
			self.kinect.new_click = True
		#print(self.kinect.last_click)


"""main function"""
def main():
	app = QApplication(sys.argv)
	app_window = Gui()
	app_window.show()
	sys.exit(app.exec_())
 
if __name__ == '__main__':
	main()
