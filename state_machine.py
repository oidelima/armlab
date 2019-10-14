import time
import numpy as np
import kinematics
import cv2
#import camera_cal


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
		self.grab_position = ()
		self.drop_position = ()
		self.click_grab = False
		self.click_dorp = False
		self.calibrateaccuracypt = ()


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
			if(self.next_state == "block detection"):
				self.block_detection()
			if(self.next_state == "color buckets"):
				self.color_buckets()

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
			if(self.next_state == "block detection"):
				self.block_detection()
			if(self.next_state == "color buckets"):
				self.color_buckets()
			if(self.next_state == "click grab drop"):
				self.clickgrabdrop()
			if(self.next_state == "pick and stack"):
				self.pickandstack()
			if(self.next_state == "line them up"):
				self.linethemup()
			if(self.next_state == "stack them high"):
				self.stackthemhigh()
			if(self.next_state == "block slider"):
				self.blockslider()
			if(self.next_state == "hot swap"):
				self.hotswap()
			if(self.next_state == "Calibration Accuracy"):
				self.calibrationaccuracy()

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

		if(self.current_state == "color buckets"):
			self.prev_state = "color buckets"
			if(self.next_state == "idle"):
				self.idle()
			if(self.next_state == "manual"):
				self.manual()

		if(self.current_state == "click grab drop"):
			self.prev_state = "click grab drop"
			if(self.next_state == "idle"):
				self.idle()
			if(self.next_state == "estop"):
				self.estop()

		if(self.current_state == "pick and stack"):
			self.prev_state = "pick and stack"
			if(self.next_state == "idle"):
				self.idle()
			if(self.next_state == "estop"):
				self.estop()

		if(self.current_state == "line them up"):
			self.prev_state = "line them up"
			if(self.next_state == "idle"):
				self.idle()
			if(self.next_state == "estop"):
				self.estop()

		if(self.current_state == "stack them high"):
			self.prev_state = "stack them high"
			if(self.next_state == "idle"):
				self.idle()
			if(self.next_state == "estop"):
				self.estop()

		if(self.current_state == "block slider"):
			self.prev_state = "block slider"
			if(self.next_state == "idle"):
				self.idle()
			if(self.next_state == "estop"):
				self.estop()

		if(self.current_state == "hot swap"):
			self.prev_state = "hot swap"
			if(self.next_state == "idle"):
				self.idle()
			if(self.next_state == "estop"):
				self.estop()

		if(self.current_state == "Calibration Accuracy"):
			self.prev_state = "Calibration Accuracy"
			if(self.next_state == "idle"):
				self.idle()
			if(self.next_state == "estop"):
				self.estop()


	"""Functions run for each state"""
	def worldCoordinates(self,x,y):
		z = self.kinect.currentDepthFrame[int(y)][int(x)]
		z = .1236*np.tan(z/2842.5 + 1.1863)

		pixel_coordinates = np.array([[x*1.75],[y*1.764],[1]]).astype(np.float32)

		camera_coordinates = np.matmul(self.kinect.intrinsic_matrix_inverse,pixel_coordinates)
		camera_coordinates = np.append(camera_coordinates,[[1]], axis=0)

		xy_world = np.matmul(self.kinect.work_camera_extrinsic_inv, camera_coordinates)
		x = xy_world[0] - 0.208
		y = xy_world[1] + 0.165
		z = .94 - z
		return x,y,z


	def calibrationaccuracy(self):
		self.status_message = "Click a known point for accuracy check"
		self.current_state = "Calibration Accuracy"
		#while(not self.calibrateaccuracypt):
		#	pass
		#xy = self.worldCoordinates(self.calibrateaccuracypt[1],self.calibrateaccuracypt[0])
		#print("world coordinates", xy )
		depth_image = self.kinect.currentDepthFrame
		rgb_image = self.kinect.currentVideoFrame
		image = depth_image*0
		for i in range(480):
			for j in range(640):
				if depth_image[i][j]> 720:
					depth_image[i][j] = 0

		depth_image = depth_image.astype(np.float32)
		depth_image = depth_image/2048.0*255
		for i in range(480):
			for j in range(640):
				image[i][j] = depth_image[i][j]
		cv2.imwrite("depthmap.jpg", image)
		self.set_next_state("idle")

	def pickandstack(self):
		#predetermined stacking position
		x_pred = -150.0
		y_pred = -20.0

		#moving above red block
		x = self.kinect.blocks["red"]["centroid"][0]
		y = self.kinect.blocks["red"]["centroid"][1]
		x,y,z = self.worldCoordinates(x,y)
		print("Red coordinates",x,y,z)

		theta =kinematics.IK([x*1000, y*1000,  50])
		#theta = kinematics.IK([166.57, -55.87])
		self.rexarm.open_gripper()
		[q, v]= self.tp.generate_cubic_spline(self.rexarm.get_positions(), theta, 3)
		self.tp.execute_plan([q,v])

		theta =kinematics.IK([x*1000, y*1000,  0])
		#theta = kinematics.IK([166.57, -55.87])
		[q, v]= self.tp.generate_cubic_spline(self.rexarm.get_positions(), theta, 3)
		self.tp.execute_plan([q,v])
		self.rexarm.close_gripper()

		#move up
		#print([x*1000, y*1000, z*1000 + 30])
		theta =kinematics.IK([x*1000, y*1000, 100])
		#print(theta)
		[q, v]= self.tp.generate_cubic_spline(self.rexarm.get_positions(), theta, 3)
		self.tp.execute_plan([q,v])


		#move to predetermined position
		theta =kinematics.IK([x_pred, y_pred, 0])
		#theta = kinematics.IK([166.57, -55.87])
		[q, v]= self.tp.generate_cubic_spline(self.rexarm.get_positions(), theta, 3)
		self.tp.execute_plan([q,v])

		self.rexarm.open_gripper()

		#move up
		#print([x*1000, y*1000, z*1000 + 30])
		theta =kinematics.IK([x_pred, y_pred, 100])
		#print(theta)
		[q, v]= self.tp.generate_cubic_spline(self.rexarm.get_positions(), theta, 3)
		self.tp.execute_plan([q,v])



  ###########################################

		#moving above bluee block
		x = self.kinect.blocks["blue"]["centroid"][0]
		y = self.kinect.blocks["blue"]["centroid"][1]

		x,y,z = self.worldCoordinates(x,y)
		print("Red coordinates",x,y, z)


		theta =kinematics.IK([x*1000, y*1000, 50])
		#theta = kinematics.IK([166.57, -55.87])
		self.rexarm.open_gripper()
		[q, v]= self.tp.generate_cubic_spline(self.rexarm.get_positions(), theta, 3)
		self.tp.execute_plan([q,v])


		theta =kinematics.IK([x*1000, y*1000, 0])
		#theta = kinematics.IK([166.57, -55.87])
		[q, v]= self.tp.generate_cubic_spline(self.rexarm.get_positions(), theta, 3)
		self.tp.execute_plan([q,v])

		self.rexarm.close_gripper()

		#move up
		#print([x*1000, y*1000, z*1000 + 30])
		theta =kinematics.IK([x*1000, y*1000, 80])
		#print(theta)
		[q, v]= self.tp.generate_cubic_spline(self.rexarm.get_positions(), theta, 3)
		self.tp.execute_plan([q,v])

		x = self.kinect.blocks["red"]["centroid"][0]
		y = self.kinect.blocks["red"]["centroid"][1]
		x,y,z = self.worldCoordinates(x,y)
		x = x_pred
		y = y_pred


		#move to predetermined position
		theta =kinematics.IK([x-10, y, 43])
		#theta = kinematics.IK([166.57, -55.87])
		[q, v]= self.tp.generate_cubic_spline(self.rexarm.get_positions(), theta, 3)
		self.tp.execute_plan([q,v])

		self.rexarm.open_gripper()

		#move up
		#print([x*1000, y*1000, z*1000 + 30])
		theta =kinematics.IK([x, y, 80])
		#print(theta)
		[q, v]= self.tp.generate_cubic_spline(self.rexarm.get_positions(), theta, 3)
		self.tp.execute_plan([q,v])



		###########################################

		#moving above green block

		x = self.kinect.blocks["yellow"]["centroid"][0]
		y = self.kinect.blocks["yellow"]["centroid"][1]
		x,y,z = self.worldCoordinates(x,y)
		print("Red coordinates",x,y, z)


		theta =kinematics.IK([x*1000, y*1000, 100])
		#theta = kinematics.IK([166.57, -55.87])
		self.rexarm.open_gripper()
		[q, v]= self.tp.generate_cubic_spline(self.rexarm.get_positions(), theta, 3)
		self.tp.execute_plan([q,v])

		theta =kinematics.IK([x*1000, y*1000, 0])
		#theta = kinematics.IK([166.57, -55.87])
		[q, v]= self.tp.generate_cubic_spline(self.rexarm.get_positions(), theta, 3)
		self.tp.execute_plan([q,v])
		self.rexarm.close_gripper()

		#move up
		#print([x*1000, y*1000, z*1000 + 30])
		theta =kinematics.IK([x*1000, y*1000, 100])
		#print(theta)
		[q, v]= self.tp.generate_cubic_spline(self.rexarm.get_positions(), theta, 3)
		self.tp.execute_plan([q,v])

		x = self.kinect.blocks["blue"]["centroid"][0]
		y = self.kinect.blocks["blue"]["centroid"][1]
		x,y,z = self.worldCoordinates(x,y)

		x = x_pred
		y = y_pred

		#move to predetermined position
		theta =kinematics.IK([x+30, y, 90])
		#theta = kinematics.IK([166.57, -55.87])
		[q, v]= self.tp.generate_cubic_spline(self.rexarm.get_positions(), theta, 3)
		self.tp.execute_plan([q,v])

		self.rexarm.open_gripper()


		self.set_next_state("idle")

	def linethemup(self):
		print("line em up")
		# you start with a predefined location of where each block is going to end
		#for every block in board, if block is not in correct position:
		#move block to correct position
		#colors = ["black", "red", "orange", "yellow", "green", "blue", "violet", "pink"]
		#temp_locs = [(-150, -150), (-150, -110), (-150, -70), (-150, -30), (-150, 10), (-150,50), (-150, 90), (-150, 130)]
		colors = ["yellow", "red", "green", "blue"]
		temp_locs = [ (-150, -30), (-150, 130),(-150,50)]
		height = [0, 30, 0]
#		height=[90, 0, 0, 0, 55, 0, 0, 0]
		for i, color in enumerate(colors):

			x = self.kinect.blocks[color]["centroid"][0]
			y = self.kinect.blocks[color]["centroid"][1]
			x,y,z = self.worldCoordinates(x,y)
			print(color, " ", x, " ", y, " ", z)
			theta =kinematics.IK([x*1000, y*1000,  height[i] + 50])
			self.rexarm.open_gripper()
			[q, v]= self.tp.generate_cubic_spline(self.rexarm.get_positions(), theta, 3)
			self.tp.execute_plan([q,v])

			theta =kinematics.IK([x*1000, y*1000,  height[i]])
			[q, v]= self.tp.generate_cubic_spline(self.rexarm.get_positions(), theta, 3)
			self.tp.execute_plan([q,v])
			self.rexarm.close_gripper()

			#move up
			theta =kinematics.IK([x*1000, y*1000, 100])
			[q, v]= self.tp.generate_cubic_spline(self.rexarm.get_positions(), theta, 3)
			self.tp.execute_plan([q,v])


			#move to predetermined position
			theta =kinematics.IK([temp_locs[i][0],temp_locs[i][1], 0])
			[q, v]= self.tp.generate_cubic_spline(self.rexarm.get_positions(), theta, 3)
			self.tp.execute_plan([q,v])
			self.rexarm.open_gripper()

			#move up
			theta =kinematics.IK([temp_locs[i][0],temp_locs[i][1], 100])
			[q, v]= self.tp.generate_cubic_spline(self.rexarm.get_positions(), theta, 3)
			self.tp.execute_plan([q,v])


		self.set_next_state("idle")

	def stackthemhigh(self):
		colors = ["black", "red", "orange", "yellow", "green", "blue", "violet", "pink"]
		temp_locs = [(100, 100), (120, 120), (140, 140), (160, 160), (180,180), (200, 200), (220, 220)]
		x_pred = -130.4
		y_pred = 101.84
		for i, color in enumerate(colors):
			count = i
			while type(self.kinect.blocks[color]["centroid"][0]) != "float":
				count = count + 1
				x = self.kinect.blocks[colors[count]]["centroid"][0]
				y = self.kinect.blocks[colors[count]]["centroid"][1]

				x, y, z = self.worldCoordinates(x, y)

				theta = kinematics.IK([x * 1000, y * 1000, 50])
				# theta = kinematics.IK([166.57, -55.87])
				self.rexarm.open_gripper()
				[q, v] = self.tp.generate_cubic_spline(self.rexarm.get_positions(), theta, 3)
				self.tp.execute_plan([q, v])
				self.rexarm.pause(2)

				theta = kinematics.IK([x * 1000, y * 1000, 0])
				# theta = kinematics.IK([166.57, -55.87])
				[q, v] = self.tp.generate_cubic_spline(self.rexarm.get_positions(), theta, 3)
				self.tp.execute_plan([q, v])
				self.rexarm.pause(2)
				self.rexarm.close_gripper()

				# move up
				# print([x*1000, y*1000, z*1000 + 30])
				theta = kinematics.IK([x * 1000, y * 1000, 40 * i + 50])
					# print(theta)
				[q, v] = self.tp.generate_cubic_spline(self.rexarm.get_positions(), theta, 3)
				self.tp.execute_plan([q, v])
				self.rexarm.pause(2)

				# move to predetermined position
				theta = kinematics.IK([temp_locs[count-1][0],temp_locs[count-1][1], 40 * i + 50])
					# theta = kinematics.IK([166.57, -55.87])
				[q, v] = self.tp.generate_cubic_spline(self.rexarm.get_positions(), theta, 3)
				self.tp.execute_plan([q, v])
				self.rexarm.pause(2)
				self.rexarm.open_gripper()

			x = self.kinect.blocks[color]["centroid"][0]
			y = self.kinect.blocks[color]["centroid"][1]

			x, y, z = self.worldCoordinates(x, y)

			theta = kinematics.IK([x * 1000, y * 1000, 50])
			# theta = kinematics.IK([166.57, -55.87])
			self.rexarm.open_gripper()
			[q, v] = self.tp.generate_cubic_spline(self.rexarm.get_positions(), theta, 3)
			self.tp.execute_plan([q, v])
			self.rexarm.pause(2)

			theta = kinematics.IK([x * 1000, y * 1000, 0])
			# theta = kinematics.IK([166.57, -55.87])
			[q, v] = self.tp.generate_cubic_spline(self.rexarm.get_positions(), theta, 3)
			self.tp.execute_plan([q, v])
			self.rexarm.pause(2)
			self.rexarm.close_gripper()

			# move up
			# print([x*1000, y*1000, z*1000 + 30])
			theta = kinematics.IK([x * 1000, y * 1000, 40*i+ 50])
			# print(theta)
			[q, v] = self.tp.generate_cubic_spline(self.rexarm.get_positions(), theta, 3)
			self.tp.execute_plan([q, v])
			self.rexarm.pause(2)

			# move to predetermined position
			theta = kinematics.IK([x_pred, y_pred, 40*i + 50])
			# theta = kinematics.IK([166.57, -55.87])
			[q, v] = self.tp.generate_cubic_spline(self.rexarm.get_positions(), theta, 3)
			self.tp.execute_plan([q, v])
			self.rexarm.pause(2)
			self.rexarm.open_gripper()



		print("stack em high")

		self.set_next_state("idle")


	def blockslider(self):
		print("block slider")
		self.set_next_state("idle")

	def hotswap(self):
		print("hot swap")
		self.set_next_state("idle")



	def clickgrabdrop(self):
		self.status_message = "State: Click and Grab and Click and Drop"
		self.current_state = "click grab drop"
		while(not self.grab_position):
			self.status_message = "Click on block to grab"
		while(not self.drop_position):
			self.status_message = "Click on drop position"
		print("grab and drop are : ",self.grab_position,self.drop_position)
		#Convert pixel to world coordinates
		y = self.grab_position[1] -45
		x = self.grab_position[0] -250
		z = self.kinect.currentDepthFrame[y][x]
		print(y,x,z)
		camera_coordinates = np.array([[x],[y],[1]]).astype(np.float32)
		xy_world = np.matmul(self.kinect.workcamera_affine,camera_coordinates)
		z_w = 0.94 - .1236*np.tan(z/2842.5 + 1.1863)

		#move to block 1
		theta =kinematics.IK([xy_world[0][0]*1000, -xy_world[1][0]*1000, z_w*1000 - 20])
		print(theta)


		#theta = kinematics.IK([166.57, -55.87])
		self.rexarm.open_gripper()
		[q, v]= self.tp.generate_cubic_spline(self.rexarm.get_positions(), theta, 3)
		self.tp.execute_plan([q,v])
		self.rexarm.pause(2)
		self.rexarm.close_gripper()

		#move up
		theta =kinematics.IK([xy_world[0][0]*1000, -xy_world[1][0]*1000, z_w*1000 + 30])
		#print(theta)
		[q, v]= self.tp.generate_cubic_spline(self.rexarm.get_positions(), theta, 3)
		self.tp.execute_plan([q,v])
		self.rexarm.pause(2)

		#drop block
		#Convert pixel to world coordinates
		y = self.drop_position[1] -45
		x = self.drop_position[0] -250
		z = self.kinect.currentDepthFrame[y][x]
		#print(y,x,z)
		camera_coordinates = np.array([[x],[y],[1]]).astype(np.float32)
		xy_world = np.matmul(self.kinect.workcamera_affine,camera_coordinates)
		z_w = 0.94 - .1236*np.tan(z/2842.5 + 1.1863)
		theta =kinematics.IK([xy_world[0][0]*1000, -xy_world[1][0]*1000, z_w*1000 + 30])
		#theta = kinematics.IK([166.57, -55.87])
		[q, v]= self.tp.generate_cubic_spline(self.rexarm.get_positions(), theta, 3)
		self.tp.execute_plan([q,v])
		self.rexarm.pause(2)
		self.rexarm.open_gripper()


		#print(xy_world, z_w)
		self.grab_position = ()
		self.drop_position = ()
		if self.prev_state == "manual":
			self.set_next_state("manual")
		else:
			self.set_next_state("idle")

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
		self.set_next_state("idle")

	def record(self):
		self.current_state = "record"
		self.waypoints.append(self.rexarm.get_positions()[:])
		self.status_message = "Recording waypoint: " + str(self.rexarm.get_positions())
		self.set_next_state("manual")

	def block_detection(self):
		self.current_state = "block detection"
		self.kinect.blockDetector()
		self.set_next_state("idle")

	def color_buckets(self):
		self.current_state = "color buckets"
		self.kinect.colorbuckets()
		self.set_next_state("idle")

	def calibrate(self):
		self.current_state = "calibrate"
		if self.prev_state == "manual":
			self.set_next_state("manual")
		else:
			self.set_next_state("idle")

		self.status_message = "Calibrating"
		self.tp.go(max_speed=2.0)
		location_strings = ["lower left corner of board",
							"upper left corner of board",
							"upper right corner of board",
							"lower right corner of board",
							"center of shoulder motor"]



		try:
			self.status_message = "Attempting Automated calibration"
			print(self.status_message)
			self.kinect.houghlines()
			for i in range(4):
				self.kinect.depth_click_points[i] = self.kinect.corners_depth[i]
				self.kinect.rgb_click_points[i] = self.kinect.corners_rgb[i]

		except:
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
		"""TODO Perform camera calibration here"""
		affine_transform = self.kinect.getAffineTransform(self.kinect.rgb_click_points,self.kinect.depth_click_points)

		#print(self.kinect.rgb_click_points)
		#print(self.kinect.depth_click_points)

		#self.kinect.affineworkspace(self.kinect.rgb_click_points)
		print("affine",self.kinect.workcamera_affine)
		print("solve PNP",self.kinect.workspaceTransform(self.kinect.rgb_click_points))
		self.kinect.kinectCalibrated = True
		self.status_message = "Calibration - Completed Calibration"
		time.sleep(1)
