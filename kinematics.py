import numpy as np
import math
from math import pi,sin,cos,pow,atan2
from math import *
#expm is a matrix exponential function
from scipy.linalg import expm

""" 
TODO: Here is where you will write all of your kinematics functions 
There are some functions to start with, you may need to implement a few more

"""

#link lengths in mm
L1 = 42.8
L2 = 99.29
L3 = 67.2
L4 = 42.3
L5 = 48.42
L6 = 35.88


def FK_dh(joint_angles, link):
	"""
    TODO: implement this function

    Calculate forward kinematics for rexarm using DH convention

    return a transformation matrix representing the pose of the
    desired link

    note: phi is the euler angle about the y-axis in the base frame

    """
	dhtable = np.array([[0, math.pi / 2, L1, math.pi + joint_angles[0]],
						[L2, 0, 0, math.pi / 2 + joint_angles[1]],
						[0, math.pi / 2, 0, math.pi / 2 + joint_angles[2]],
						[0, math.pi / 2, L3 + L4, math.pi + joint_angles[3]],
						[0, math.pi / 2, 0, math.pi + joint_angles[4]],
						[0, 0, L5 + L6,joint_angles[5]]])

	# link thetas
	th1 = dhtable[0, 3];
	th2 = dhtable[1, 3];
	th3 = dhtable[2, 3];
	th4 = dhtable[3, 3];
	th5 = dhtable[4, 3];
	th6 = dhtable[5, 3];

	# link alphas
	al1 = dhtable[0, 1];
	al2 = dhtable[1, 1];
	al3 = dhtable[2, 1];
	al4 = dhtable[3, 1];
	al5 = dhtable[4, 1];
	al6 = dhtable[5, 1];

	A1 = np.array([[math.cos(th1), -math.sin(th1) * math.cos(al1), math.sin(th1) * math.sin(al1), 0],
				   [math.sin(th1), math.cos(th1) * math.cos(al1), -math.cos(th1) * math.sin(al1), 0],
				   [0, math.sin(al1), math.cos(al1), dhtable[0, 2]],
				   [0, 0, 0, 1]])

	A2 = np.array([[math.cos(th2), -math.sin(th2) * math.cos(al2), math.sin(th2) * math.sin(al2), L2 * math.cos(th2)],
				   [math.sin(th2), math.cos(th2) * math.cos(al2), -math.cos(th2) * math.sin(al2), L2 * math.sin(th2)],
				   [0, math.sin(al2), math.cos(al2), dhtable[1, 2]],
				   [0, 0, 0, 1]])

	A3 = np.array([[math.cos(th3), -math.sin(th3) * math.cos(al3), math.sin(th3) * math.sin(al3), 0],
				   [math.sin(th3), math.cos(th3) * math.cos(al3), -math.cos(th3) * math.sin(al3), 0],
				   [0, math.sin(al3), math.cos(al3), dhtable[2, 2]],
				   [0, 0, 0, 1]])

	A4 = np.array([[math.cos(th4), -math.sin(th4) * math.cos(al4), math.sin(th4) * math.sin(al4), 0],
				   [math.sin(th4), math.cos(th4) * math.cos(al4), -math.cos(th4) * math.sin(al4), 0],
				   [0, math.sin(al4), math.cos(al4), dhtable[3, 2]],
				   [0, 0, 0, 1]])
	A5 = np.array([[math.cos(th5), -math.sin(th5) * math.cos(al5), math.sin(th5) * math.sin(al5), 0],
				   [math.sin(th5), math.cos(th5) * math.cos(al5), -math.cos(th5) * math.sin(al5), 0],
				   [0, math.sin(al5), math.cos(al5), dhtable[4, 2]],
				   [0, 0, 0, 1]])

	A6 = np.array([[math.cos(th6), -math.sin(th6) * math.cos(al6), math.sin(th6) * math.sin(al6), 0],
				   [math.sin(th6), math.cos(th6) * math.cos(al6), -math.cos(th6) * math.sin(al6), 0],
				   [0, math.sin(al6), math.cos(al6), dhtable[5, 2]],
				   [0, 0, 0, 1]])

	H1 = A1
	H2 = np.matmul(H1, A2)
	H3 = np.matmul(H2, A3)
	H4 = np.matmul(H3, A4)
	H5 = np.matmul(H4, A5)
	H6 = np.matmul(H5, A6)
	#print(H6)

	if link == 1:
		return H1
	elif link == 2:
		return H2
	elif link == 3:
		return H3
	elif link == 4:
		return H4
	elif link == 5:
		return H5
	elif link == 6:
		return H6


def kf_test():

	print("Elbow at 90 deg gives: ", np.matmul(FK_dh([0,0,math.pi/2,0,0,0], 6), np.array([0,0,0,1]))) # elbow at 90
	print("Elbow at -90 deg gives: ", np.matmul(FK_dh([0,0,-math.pi/2,0,0,0], 6), np.array([0,0,0,1]))) # elbow at -90
	print("Elbow at 45 deg gives: ", np.matmul(FK_dh([0,0,math.pi/4,0,0,0], 6), np.array([0,0,0,1]))) # elbow at 45
	print("Elbow at -45 deg gives: ", np.matmul(FK_dh([0,0,-math.pi/4,0,0,0], 6), np.array([0,0,0,1]))) # elbow at -45
	print("Shoulder at 90 deg gives: ", np.matmul(FK_dh([0,math.pi/2,0, 0,0,0], 6), np.array([0,0,0,1]))) # shoulder at 90
	print("Shoulder at -90 deg gives: ", np.matmul(FK_dh([0,-math.pi/2,0,0,0,0], 6), np.array([0,0,0,1]))) # shoulder at -90
	print("Shoulder at 45 deg gives: ", np.matmul(FK_dh([0,math.pi/4,0,0,0,0], 6), np.array([0,0,0,1]))) # shoulder at 45
	print("Shoulder at -45 deg gives: ", np.matmul(FK_dh([0,-math.pi/4,0,0,0,0], 6), np.array([0,0,0,1]))) # shoulder at -45
	print("Shoulder at -45 deg, base at 90, elboow at -45 gives: ", np.matmul(FK_dh([math.pi/2,-math.pi/4,-math.pi/4,0,0,0], 4), np.array([0,0,0,1]))) # shoulder at -45

kf_test()

def FK_pox(joint_angles):
	pass
	"""
	TODO: implement this function

	Calculate forward kinematics for rexarm
	use math.sing product of exponential formulation

	return a 4-tuple (x, y, z, phi) representing the pose of the 
	desired link

	note: phi is the euler angle about y in the base frame

	"""

def IK(o ,R = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])):
	""" TODO: Calculate inverse kinematics for rexarm return the required joint angles """

	print(o)
	oc = np.round(np.array([[o[0] - (L6+L5)*R[0][2]], [o[1] - (L6 + L5)*R[1][2]], [o[2] - (L6 + L5)*R[2][2]]]),6)

	#if np.round(oc[0]**2 + oc[1]**2 + (oc[2]-L1)**2,4) > np.round((L2+ L3+ L4)**2,4) or np.round(oc[0]**2 + oc[1]**2 + (oc[2]-L1)**2,4) < np.round((L2- L3- L4)**2,4):
	#	print("out_of_range")
#		R = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])

	theta1 = atan2(oc[1], oc[0])  #two possibilities
	print(np.round((oc[0]**2 + oc[1]**2 + (oc[2]-L1)**2 - (L3+L4)**2 - L2**2)/(2*(L3+L4)*L2), 4))
	try:
		theta3 = acos(np.round((oc[0]**2 + oc[1]**2 + (oc[2]-L1)**2 - (L3+L4)**2 - L2**2)/(2*(L3+L4)*L2), 4)) #two possibilities
	except:
		print("Location and orientation not reacheable upwards! Trying sideways")
		R = np.array([[0,0,1], [0, 1, 0], [-1, 0, 0]])
		oc = np.round(np.array([[o[0] - (L6+L5)*R[0][2]], [o[1] - (L6 + L5)*R[1][2]], [o[2] - (L6 + L5)*R[2][2]]]),6)
		theta3 = acos(np.round((oc[0]**2 + oc[1]**2 + (oc[2]-L1)**2 - (L3+L4)**2 - L2**2)/(2*(L3+L4)*L2), 4)) #two possibilities
		
	theta2 = pi/2 - (atan2(oc[2]-L1, sqrt(oc[0]**2 + oc[1]**2)) - atan2((L3+L4)*sin(-theta3), L2 + (L3+L4)*cos(-theta3))) #two possibilities because of theta 3

	R03 = FK_dh([theta1, theta2, theta3, 0, 0, 0], 3)[0:3, 0:3]
	#R03 = np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]])
	R36 = np.matmul(np.transpose(R03),R)
	theta5 = atan2(sqrt(1 - R36[2][2] ** 2), R36[2][2])

	if round(R36[0][2],2) == 0.0 and round(R36[1][2],2) == 0.0:
		#infinitely many solutions
		theta4 = 0.0
		theta6 = 0.0
	elif sin(theta5) >= 0:
		theta4 = atan2(R36[1][2], R36[0][2])
		theta6 = atan2(R36[2][1], -R36[2][0])
	else:
		theta4 = atan2(-R36[1][2], -R36[0][2])
		theta6 = atan2(-R36[2][1], R36[2][0])




	return [theta1, theta2, theta3, theta4, theta5, theta6]

def ik_test():

	#testing elbow at 90 deg
	Rfull = FK_dh([0,0,pi/2,0,0,0], 6)
	R = Rfull[0:3, 0:3]
	o = np.array([Rfull[0][3], Rfull[1][3], Rfull[2][3]])
	print("Elbow at 90 gives: ",IK(o,R))

	#testing elbow at -90
	Rfull = FK_dh([0,0,-math.pi/2,0,0,0], 6)
	R = Rfull[0:3, 0:3]
	o = np.array([Rfull[0][3], Rfull[1][3], Rfull[2][3]]) 
	print("Elbow at -90 gives: ",IK(o,R))

	#testing elbow at 45
	Rfull = FK_dh([0,0,math.pi/4,0,0,0], 6)
	R = Rfull[0:3, 0:3]
	o = np.array([Rfull[0][3], Rfull[1][3], Rfull[2][3]]) 
	print("Elbow at 45 gives: ",IK(o,R))

	# testing elbow at 45
	Rfull = FK_dh([0, 0, -math.pi / 4, 0, 0, 0], 6)
	R = Rfull[0:3, 0:3]
	o = np.array([Rfull[0][3], Rfull[1][3], Rfull[2][3]])
	print("Elbow at -45 gives: ", IK(o, R))

	#testing shoulder at 90
	Rfull = FK_dh([0,math.pi/2,0,0,0,0], 6)
	R = Rfull[0:3, 0:3]
	o = np.array([Rfull[0][3], Rfull[1][3], Rfull[2][3]]) 
	print("Shoulder at 90 gives: ",IK(o,R))

	# testing shoulder at -90
	Rfull = FK_dh([0, -math.pi / 2, 0, 0, 0, 0], 6)
	R = Rfull[0:3, 0:3]
	o = np.array([Rfull[0][3], Rfull[1][3], Rfull[2][3]])
	print("Shoulder at -90 gives: ", IK(o, R))

	# testing shoulder at 45
	Rfull = FK_dh([0, math.pi / 4, 0, 0, 0, 0], 6)
	R = Rfull[0:3, 0:3]
	o = np.array([Rfull[0][3], Rfull[1][3], Rfull[2][3]])
	print("Shoulder at 45 gives: ", IK(o, R))

	# testing shoulder at -45
	Rfull = FK_dh([0, -math.pi / 4, 0, 0, 0, 0], 6)
	R = Rfull[0:3, 0:3]
	o = np.array([Rfull[0][3], Rfull[1][3], Rfull[2][3]])
	print("Shoulder at -45 gives: ", IK(o, R))

	# testing base at 90, shoulder at 45
	Rfull = FK_dh([math.pi/2, math.pi / 4, 0, 0, 0, 0], 6)
	R = Rfull[0:3, 0:3]
	o = np.array([Rfull[0][3], Rfull[1][3], Rfull[2][3]])
	print("Base at 90 Shoulder at 45 gives: ", IK(o, R))

	# testing base at 90, shoulder at -45
	Rfull = FK_dh([math.pi / 2, -math.pi / 4, 0, 0, 0, 0], 6)
	R = Rfull[0:3, 0:3]
	o = np.array([Rfull[0][3], Rfull[1][3], Rfull[2][3]])
	print("Base at 90 Shoulder at -45 gives: ", IK(o, R))

	# testing base at 90, elbow at 60, shoulder at 30
	Rfull = FK_dh([math.pi / 2, math.pi / 3, math.pi / 6, 0, 0, 0], 6)
	R = Rfull[0:3, 0:3]
	o = np.array([Rfull[0][3], Rfull[1][3], Rfull[2][3]])
	print("Base at 90 Elbow at 60 Shoulder at 30 gives: ", IK(o, R))

	# testing elbow at 90 deg, W2 at -90
	Rfull = FK_dh([0, 0, pi / 2, 0, -pi/2, 0], 6)
	R = Rfull[0:3, 0:3]
	o = np.array([Rfull[0][3], Rfull[1][3], Rfull[2][3]])
	print("Elbow at 90 gives, W2 at -90: ", IK(o, R))

	# testing elbow at 90 deg, W2 at 90
	Rfull = FK_dh([0, 0, pi / 2, 0, pi / 2, 0], 6)
	R = Rfull[0:3, 0:3]
	o = np.array([Rfull[0][3], Rfull[1][3], Rfull[2][3]])
	print("Elbow at 90 gives, W2 at 90: ", IK(o, R))

	# testing elbow at 90 deg, W1 at -45, W2 at 90
	Rfull = FK_dh([0, 0, pi / 2, -pi/4, pi / 2, 0], 6)
	R = Rfull[0:3, 0:3]
	o = np.array([Rfull[0][3], Rfull[1][3], Rfull[2][3]])
	print("Elbow at 90 gives,W1 at -45,  W2 at 90: ", IK(o, R))

	# testing elbow at 90 deg, W1 at -45, W2 at 90, W3 at 45
	Rfull = FK_dh([0, 0, pi / 2, -pi / 4, pi / 2, pi/4], 6)
	R = Rfull[0:3, 0:3]
	o = np.array([Rfull[0][3], Rfull[1][3], Rfull[2][3]])
	print("Elbow at 90 gives,W1 At -45 W2 at 90 W3 at 45: ", IK(o, R))

	#Real world tests

	angles = np.zeros((15, 6))

	#LEVEL1

	# Block at 166.57, -55.87, 17.27 with same orientation as frame 0
	R = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])
	o = np.array([166.57, -55.87, 17.27-L1])
	angles[0] = IK(o, R)

	R = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])
	o = np.array([-6.28, -180, 17.27 - L1])
	angles[1] = IK(o, R)

	R = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])
	o = np.array([154.82, -101.84, 17.27 - L1])
	angles[2] = IK(o, R)

	R = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])
	o = np.array([-118.14, 150.83, 17.27 - L1])
	angles[3] = IK(o, R)

	"""
	R = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])
	o = np.array([202.2, 136.45, 17.27 - L1])
	angles[4] = IK(o, R)"""

	#LEVEL2

	R = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])
	o = np.array([166.57, -55.87, 17.27*2 - L1])
	angles[5] = IK(o, R)

	R = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])
	o = np.array([-6.28, -180, 17.27*2 - L1])
	angles[6] = IK(o, R)

	R = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])
	o = np.array([154.82, -101.84, 17.27*2 - L1])
	angles[7] = IK(o, R)

	R = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])
	o = np.array([-118.14, 150.83, 17.27*2 - L1])
	angles[8] = IK(o, R)

	"""R = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])
	o = np.array([202.2, 136.45, 17.27*2 - L1])
	angles[9] = IK(o, R)"""

	#LEVEL3

	R = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])
	o = np.array([166.57, -55.87, 17.27*3 - L1])
	angles[10] = IK(o, R)

	R = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])
	o = np.array([-6.28, -180, 17.27*3 - L1])
	angles[11] = IK(o, R)

	R = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])
	o = np.array([154.82, -101.84, 17.27*3 - L1])
	angles[12] = IK(o, R)

	R = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])
	o = np.array([-118.14, 150.83, 17.27*3 - L1])
	angles[13] = IK(o, R)

	"""R = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])
	o = np.array([202.2, 136.45, 17.27*3 - L1])
	angles[14] = IK(o, R)"""

	#extra tests
	#[-0.6397397664270065, 0.4260223494513975, 1.4493983633909848, -8.343964209548854e-17, 1.266171940747411, -0.6397397664270064]
	o = np.array([116.72079563140869, -86.85654401779175, 16.072748008628356])
	print(IK(o))

ik_test()

# def get_euler_angles_from_T(T):
# 	""" TODO: implement this function return the Euler angles from a T matrix """
# 	theta = 0.0
# 	phi = 0.0
# 	beta = 0.0

# 	if T[1,3] or T[2,3]:
# 		theta = atan2(T[3,3], pow((1 - pow(T[3,3],2), 0.5))
# 		if sin(theta) > 0:
# 			phi = atan2(T[1,3], T[2,3])
# 			beta = atan2(-T[3,1], T[3,2])
# 		else:
# 			phi = atan2(-T[1,3], T[2,3])
# 			beta = atan2(-T[3,1], -T[3,2])

# 	else:
# 		#Determine theta
# 		if T[3,3] == 0:
# 			theta = 0  
# 		#Need to determine phi + beta assuming phi = 0 by convention: Determine beta 
# 			beta =  atan2(T[1,1], -T[1,2])

# 		else:
# 			theta = pi
# 			#Need to determine phi + beta assuming phi = 0 by convention: Determine beta 
# 			beta =  atan2(-T[1,1], -T[1,2])

# 	return (theta, phi, beta) 

# def get_pose_from_T(T):
# 	""" TODO: return the joint pose from a T matrix of the form (x,y,z,phi) where phi is rotation about base frame y-axis """
# 	x = T[0]
# 	y = T[1]
# 	z = T[2]
# 	phi = T[3]

# 	matrix = np.array([[ cos(phi), 0, sin(phi), x],
# 					   [      0, 1,      0, y],
# 					   [-sin(phi), 0, cos(phi), z],
# 					   [      0, 0,      0, 1]])
# 	pass


# def to_s_matrix(w,v):
# 	""" TODO: implement this function Find the [s] matrix for the POX method e^([s]*theta) """
# 	pass