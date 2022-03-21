import os
import csv
from signal import pthread_kill
from tkinter import E
from matplotlib.pyplot import thetagrids
import serial
import keyboard
from time import time
from numpy import sin, cos, arctan2, arcsin, pi, size, power, dot, array, zeros , copysign
import numpy as np 
from numpy.linalg import norm
import math 
SERIAL_PORT = 'COM4'	# this port address is for the serial tx/rx pins on the GPIO header
SERIAL_RATE = 115200	# be sure to set this to the same rate used on the Arduino
RAD_TO_DEG = 180/pi
DEG_TO_RAD = pi/180

save_dataset = 0		# if =1 creates a file .csv (excel) with a captured dataset

#---------------------------------------------------------------------------------------------------#
#--------------------------------------- FUNZIONI DI STAMPA ----------------------------------------#
#---------------------------------------------------------------------------------------------------#

def space_change(value):
	if (value > -10 and value < 0) or (value >= 10 and value < 100):
		return "  "
	elif value >= 0 and value < 10:
		return "   "
	elif (value > -100 and value <= -10) or (value >= 100):
		return " "
	else:
		return ""

def print_pose(q):
	q_x, q_y, q_z = q[0,0], q[1,0], q[2,0]
	s1, s2, s3 = space_change(q_x), space_change(q_y), space_change(q_z)
	print(f"pose=[{s1}{format(q_x,'.3f')}   {s2}{format(q_y,'.3f')}   {s3}{format(q_z,'.3f')} ]", end="")

def print_one(q):
	print_pose(q)
	print("") # newline

#---------------------------------------------------------------------------------------------------#
#------------------------------------ MATRICI E VETTORI UTILI --------------------------------------#
#---------------------------------------------------------------------------------------------------#

# def Rx(a):
# 	R = array([	[		1,			0,			0	],
# 				[		0,		cos(a),		-sin(a)	],
# 				[		0,		sin(a),		cos(a)	]])
# 	return R

# def Ry(a):
# 	R = array([	[	cos(a),			0,		sin(a)	],
# 				[		0,			1,		0		],
# 				[	-sin(a),		0,		cos(a)	]])
# 	return R

# def Rz(a):
# 	R = array([	[	cos(a),		-sin(a),	0	],
# 				[	sin(a),		cos(a),		0	],
# 				[		0,			0,		1	]])
# 	return R

def skew_symmetric_4x4(a, b, c):
	Omega = array([	[	0,		-a,		-b,		-c	],
					[	a,		0,		c,		-b	],
					[	b,		-c,		0,		a	],
					[	c,		b,		-a,		0	]])
	return Omega

def get_quaternion_from_euler(roll, pitch, yaw):
	quat = array([	[cos(roll/2)*cos(pitch/2)*cos(yaw/2) + sin(roll/2)*sin(pitch/2)*sin(yaw/2)],
					[sin(pitch/2)*cos(roll/2)*cos(yaw/2) - cos(pitch/2)*sin(roll/2)*sin(yaw/2)],
					[cos(pitch/2)*sin(roll/2)*cos(yaw/2) + sin(pitch/2)*cos(roll/2)*sin(yaw/2)],
					[cos(pitch/2)*cos(roll/2)*sin(yaw/2) - sin(pitch/2)*sin(roll/2)*cos(yaw/2)]])
	return np.reshape(quat,(1,4))

def get_euler_from_quaternion(quat):
	quat = np.reshape(quat,(4,1))
	q0 = quat[0]
	q1 = quat[1]
	q2 = quat[2]
	q3 = quat[3]

	roll = np.arctan2(2*(q0*q1 + q2*q3), 1 - 2*(power(q1,2) + power(q2,2)))
	check = 2*(q1*q3 - q0*q2)
	if check >= 1:	pitch = arcsin(check-1) # use 90 degrees if out of range
	if check <= - 1:	pitch = arcsin(check+1) # use 90 degrees if out of range		
	if -1 < check < 1 :	pitch = arcsin(check) # use 90 degrees if out of range	
	pitch = arcsin(check)  #RICONTROLLARE MENO
	yaw = arctan2(2*(q0*q3 + q1*q2), 1 - 2*(power(q2,2) + power(q3,2)))
	pose = np.asarray([roll, pitch, yaw])
	pose = pose.reshape((3))
	return pose

#---------------------------------------------------------------------------------------------------#
#-------------------------------------- FILTRO COMPLEMENTARE ---------------------------------------#
#---------------------------------------------------------------------------------------------------#

# def quaternion_coniugate(q):
# 	q_conj = [ q[0] , - q[1] , - q[2] , - q[3] ]

	

# def ComplementaryFilter(q, data, dt, alpha, mag = False):
# 	a_x, a_y, a_z = data[0], data[1], data[2]
# 	g_x, g_y, g_z = data[3], data[4], data[5]
# 	# -----------------------------------------
# 	Omega_t = skew_symmetric_4x4(g_x, g_y, g_z)
# 	q_gyro_dot = 1/2 * dot(Omega_t, q)
# 	q_gyro = q + q_gyro_dot * dt								# attitude estimated from gyroscope
# 	# -----------------------------------------
# 	roll = arctan2(a_y, a_z)
# 	pitch = arctan2(-a_x, power(power(a_y,2)+power(a_z,2),1/2)) # -a_x / sqrt(a_y^2 + a_z^2)
# 	if mag == False :
# 		yaw = 0
# 	else:				# MPU-9250
# 		m_x, m_y, m_z = data[6], data[7], data[8]
# 		m_vect = array([[m_x],[m_y],[m_z]])
# 		# -----------------------------------------
# 		R = dot(Ry(pitch), Rx(roll))
# 		b = dot(R, m_vect) # rotated magnetic field
# 		b_x, b_y = b[0,0], b[1,0]
# 		yaw = arctan2(-b_y, b_x)
# 	q_acc_magn = get_quaternion_from_euler(roll, pitch, yaw)	# attitude estimated from accelerometer (and magnetometer)
# 	# -----------------------------------------
# 	q = (1 - alpha) * q_gyro + alpha * q_acc_magn.T				# estimated quaternion
# 	return q

def eul_to_quat213(roll,pitch,yaw):
	''''create quaternion based on ZXY sequence of euler angles'''
	theta = roll
	phi = pitch
	psi = yaw	
	cpsi = cos(psi/2)
	spsi = sin(psi/2)
	ctheta = cos(theta/2)
	stheta = sin(theta/2)
	cphi = cos(phi/2)
	sphi = sin(phi/2)
	q0 = cphi * ctheta * cpsi - sphi * stheta * spsi
	q1 = cphi * stheta * cpsi - sphi * ctheta * spsi
	q2 = cphi * stheta * spsi + sphi * ctheta * cpsi
	q3 = cphi * ctheta * spsi + sphi * stheta * cpsi 
	return np.asarray([q0,q1,q2,q3])

def eul_to_quatXYZ(roll,pitch,yaw):
	''''create quaternion based on XYZ sequence of euler angles'''
	theta = roll
	phi = pitch
	psi = yaw	
	cpsi = cos(psi/2)
	spsi = sin(psi/2)
	ctheta = cos(theta/2)
	stheta = sin(theta/2)
	cphi = cos(phi/2)
	sphi = sin(phi/2)
	q0 = cphi * ctheta * cpsi - sphi * stheta * spsi
	q1 = cphi * ctheta * spsi + sphi * stheta * cpsi
	q2 = cphi * stheta * cpsi - sphi * ctheta * spsi
	q3 = cphi * stheta * spsi + sphi * ctheta * cpsi 
	return np.asarray([q0,q1,q2,q3])

def eul_to_quatZYX(roll,pitch,yaw):
	''''create quaternion based on ZYX sequence of euler angles'''
	theta = roll
	phi = pitch
	psi = yaw	
	cpsi = cos(psi/2)
	spsi = sin(psi/2)
	ctheta = cos(theta/2)
	stheta = sin(theta/2)
	cphi = cos(phi/2)
	sphi = sin(phi/2)
	q0 = cphi * ctheta * cpsi + sphi * stheta * spsi
	q1 = sphi * ctheta * cpsi - cphi * stheta * spsi
	q2 = cphi * stheta * cpsi + sphi * ctheta * spsi
	q3 = cphi * ctheta * spsi - sphi * stheta * cpsi 
	return np.asarray([q0,q1,q2,q3])

def eul_to_quat212(roll,pitch,yaw):
	''''create quaternion based on YXY sequence of euler angles'''
	theta = roll
	phi = pitch
	psi = yaw	
	cpsi = cos(psi/2)
	spsi = sin(psi/2)
	ctheta = cos(theta/2)
	stheta = sin(theta/2)
	cphi = cos(phi/2)
	sphi = sin(phi/2)
	q0 = cphi * ctheta * cpsi - sphi * ctheta * spsi
	q1 = cphi * stheta * cpsi + sphi * stheta * spsi
	q2 = cphi * ctheta * spsi + sphi * ctheta * cpsi
	q3 = sphi * stheta * spsi - cphi * stheta * spsi 
	return np.asarray([q0,q1,q2,q3])

def eul_to_quat312(roll,pitch,yaw):
	''''create quaternion based on ZXY sequence of euler angles'''
	theta = roll
	phi = pitch
	psi = yaw	
	cpsi = cos(psi/2)
	spsi = sin(psi/2)
	ctheta = cos(theta/2)
	stheta = sin(theta/2)
	cphi = cos(phi/2)
	sphi = sin(phi/2)
	q0 = cphi * ctheta * cpsi + sphi * stheta * spsi
	q1 = cphi * stheta * cpsi + sphi * ctheta * spsi
	q2 = cphi * ctheta * spsi - sphi * stheta * cpsi
	q3 = sphi * ctheta * cpsi - cphi * ctheta * spsi 
	return np.asarray([q0,q1,q2,q3])

#R_XYZ_quat():


#---------------------------------------------------------------------------------------------------#
#------------------------------------ MATRICI E VETTORI UTILI --------------------------------------#
#---------------------------------------------------------------------------------------------------#

def Rx(a):
	R = array([	[		1,			0,			0	],
				[		0,		cos(a),		sin(a)	],
				[		0,		-sin(a),	cos(a)	]])
	return R

def Ry(a):
	R = array([	[	cos(a),			0,		-sin(a)	],
				[		0,			1,		0		],
				[	sin(a),			0,		cos(a)	]])
	return R

def Rz(a):
	R = array([	[	cos(a),		sin(a),	0	],
				[	-sin(a),	cos(a),		0	],
				[		0,			0,		1	]])
	return R

def asec(x):
	if x !=0 :
		a = np.arccos(1/x)
	else:
		a = 0
	return a

def CPL_FilteringStep(asset,data,dt,mag ):
	'''complementary filter based on accelerometer,gyro,mag mesures using euler angles XYZ'''
	phi = asset[0]
	theta = asset[1] 
	psi = asset[2]
	a_x, a_y, a_z = data[0], data[1], data[2]
	g_x, g_y, g_z = data[3], data[4], data[5]
	phi_acc = np.arctan2( a_y , a_z )#np.arctan2(-a_x , np.sqrt(a_y**2+a_z**2)) # #roll angle (rotation along x)
	theta_acc = np.arctan2(-a_x,a_y*sin(phi_acc)+a_z*cos(phi_acc))#np.arctan2(a_y , np.sqrt(a_x**2+a_z**2)) #arctan2(-a_x, np.sqrt(a_y**2+a_z**2)) #pitch angle (rotation along y)
	psi_acc = np.arctan2(np.sqrt(a_y**2+a_x**2),a_z)

	#if a_z > 0 and a_y > 0 : theta_acc = theta_acc 
	#if a_z < 0 and a_y > 0 : theta_acc = - theta_acc + np.pi
	#if a_z < 0 and a_y < 0 : theta_acc = - theta_acc -  np.pi
	#if a_z > 0 and a_y < 0 : theta_acc = theta_acc - np.pi

	if mag == True:
		m_x, m_y, m_z = data[6], data[7], data[8]
		My = m_x*sin(phi_acc)-m_y*cos(phi_acc)#m_x * sin(theta_acc) * sin(phi_acc) + m_y* cos(theta_acc) - m_z * sin(theta_acc) * cos(phi_acc)
		Mx = m_x*cos(theta_acc)+m_y*sin(theta_acc)*sin(phi_acc)+m_z*sin(theta_acc)*cos(psi_acc) #m_x * cos(phi_acc) + m_z * sin(phi_acc)
		psi_mag = - arctan2( My , Mx )
	else:
		psi_mag= 0

	alpha = 1 / (1 + dt)
	p = g_x
	q = g_y
	r = g_z
	psi_dot_gyro = p + q * sin(phi) * np.tan(theta) + r * cos(phi) * np.tan(theta)
	theta_dot_gyro = q * cos(phi) - r * sin(phi)
	phi_dot_gyro = q * sin(phi) * asec(theta) + r * cos(phi)* asec(theta)
	gyro_data = array( [0 * dt  , 
						0 * dt  , 
						0 * dt ] )
	accmag_data = array( [phi_acc,theta_acc,psi_mag] )
	gyroass = array( asset + gyro_data )
	angles = (1 - alpha) * gyroass + alpha * accmag_data
	return angles

def quaternion_product2(q1, q2):
	q1 = np.reshape(q1,(1,4))
	q2 = np.reshape(q2,(1,4))
	Omega = skew_symmetric_4x4_B(q2)
	return dot(q1, Omega)

def quaternion_product3(q1, q2, q3):

	q1q2 = quaternion_product2(q1, q2)
	q1q2q3 = quaternion_product2(q1q2, q3)
	return q1q2q3

def quaternion_conjugate(q):
	q = np.reshape(q,(1,4))
	q1, q2, q3, q4 = q[0,:]
	q_star = array([q1, -q2, -q3, -q4])
	return q_star

def f_g(q, a):
	q = np.reshape(q,(1,4))
	q1, q2, q3, q4 = q[0,:]
	a = np.reshape(a,(1,4))
	ax, ay, az = a[0,1], a[0,2], a[0,3]
	f = array([	[		2*(q2*q4 - q1*q3) - ax		],
				[		2*(q1*q2 + q3*q4) - ay		],
				[	2*(1/2 - q2**2 - q3**2) - az	]])
	return f

def J_g(q):
	q = np.reshape(q,(1,4))
	q1, q2, q3, q4 = q[0,:]
	J = array([	[-2*q3,	2*q4,	-2*q1,	2*q2],
				[2*q2,	2*q1,	2*q4,	2*q3],
				[	0,	-4*q2,	-4*q3,	0	]])
	return J

def f_b(q, b, m):
	q = np.reshape(q,(1,4))
	b = np.reshape(b,(1,4))
	m = np.reshape(m,(1,4))
	q1, q2, q3, q4 = q[0,:]
	bx, bz = b[0,1], b[0,3]
	mx, my, mz = m[0,1], m[0,2], m[0,3]
	f = array([	[2*bx*(1/2 - q3**2 - q4**2) + 2*bz*(q2*q4 - q1*q3) - mx	],
				[	2*bx*(q2*q3 - q1*q4) + 2*bz*(q1*q2 + q3*q4) - my	],
				[2*bx*(q1*q3 + q2*q4) + 2*bz*(1/2 - q2**2 - q3**2) - mz	]])
	return f

def J_b(q, b):
	q = np.reshape(q,(1,4))
	b = np.reshape(b,(1,4))
	q1, q2, q3, q4 = q[0,:]
	bx, bz = b[0,1], b[0,3]
	J = array([	[	-2*bz*q3,				2*bz*q4,		-4*bx*q3 - 2*bz*q1,	-4*bx*q4 + 2*bz*q2	],
				[-2*bx*q4 + 2*bz*q2,	2*bx*q3 + 2*bz*q1,	2*bx*q2 + 2*bz*q4,	-2*bx*q1 + 2*bz*q3	],
				[	2*bx*q3,			2*bx*q4 - 4*bz*q2,	2*bx*q1 - 4*bz*q3,		2*bx*q2			]])
	return J

def f_gb(q, a, b, m):
	return np.vstack( ( f_g(q, a),f_b(q, b, m) ) )

def J_gb(q, b):
	print(np.vstack( ( J_g(q),J_b(q, b) ) ))
	return np.vstack( ( J_g(q),J_b(q, b) ) )

def skew_symmetric_4x4_B(omega):
	a, b, c, d = omega[0,:]
	Omega = array([	[	a,		b,		c,		d	],
					[	-b,		a,		-d,		c	],
					[	-c,		d,		a,		-b	],
					[	-d,		-c,		b,		a	]])
	return Omega

def MadgwickFilter(q_est, data, dt, alpha, omega_error_max,mag = True):
	a_x, a_y, a_z = data[0], data[1], data[2]
	g_x, g_y, g_z = data[3], data[4], data[5]
	a = array([0, a_x, a_y, a_z])
	a = a / norm(a) # normalization
	w = array([0, g_x, g_y, g_z]) #omega
	# -----------------------------------------

	q_omega_dot = 1/2 * quaternion_product2(q_est, w)
	q_omega = q_est + q_omega_dot * dt							# orientation of the earth frame relative to the sensor frame (using gyroscope)
	
	mu = alpha * norm(q_omega_dot) * dt # alpha > 1
	beta = power(3/4, 1/2) * omega_error_max
	gamma = beta / (mu/dt + beta)

	# -----------------------------------------

	if mag == False :	# MPU-6050

		Nabla_f = dot(np.transpose(J_g(q_est)), f_g(q_est, a)) # Nabla_f = J_g' * f_g

		# -----------------------------------------

	else:				# MPU-9250

		m_x, m_y, m_z = data[6], data[7], data[8]
		m = array([0, m_x, m_y, m_z])
		m = m / norm(m) # normalization
		# -----------------------------------------

		h = quaternion_product3(q_est, m, quaternion_conjugate(q_est)) # h = q_est * m * q_est'
		h_x, h_y, h_z = h[0,1], h[0,2], h[0,3]
		b = array([0, power(h_x**2 + h_y**2, 1/2), 0, h_z])

		Nabla_f = dot(np.transpose(J_gb(q_est,b)), f_gb(q_est, a,b,m)) # Nabla_f = J_gb' * f_gb

	# -----------------------------------------

	q_nabla = q_est - mu * Nabla_f.T / norm(Nabla_f)
	
	q_est = gamma * q_nabla + (1 - gamma) * q_omega			# estimated orientation of the earth frame relative to the sensor frame
	q_est = q_est / norm(q_est) # normalization

	return q_est