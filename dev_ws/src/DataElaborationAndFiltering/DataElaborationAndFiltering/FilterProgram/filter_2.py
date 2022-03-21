from os import supports_bytes_environ
import numpy as np
import pyquaternion as pq
import ahrs
from FilterProgram.support import * 

ekf = ahrs.filters.ekf.EKF(None,None ,None , 25, frame='ENU')

cf = ahrs.filters.Complementary(None,None ,None , 25, str = 'NED')


def compute_new_asset_quat2(q, acc,gyro,mag, dt):
    q = ekf.update(q, gyro, acc,mag)
    return q

def Complementary2Eul(asset,acc,gyro,mag,dt):
	'''complementary filter based on accelerometer,gyro,mag mesures using euler angles XYZ'''

	alpha = 0.98
	phi = asset[0]
	theta = asset[1] 
	psi = asset[2]
	a_x = acc[0] 
	a_y = acc[1]
	a_z = acc[2]
	g_x = gyro[0] 
	g_y = gyro[1] 
	g_z = gyro[2] 


	p = g_x
	q = g_y
	r = g_z

	phi_dot_gyro = p + q * np.sin(phi) * np.tan(theta) + r * np.cos(phi) * np.tan(theta)
	theta_dot_gyro = q * np.cos(phi) - r * np.sin(phi)
	psi_dot_gyro = q * np.sin(phi) * np.cos(theta)**-1 + r * np.cos(phi) * np.cos(theta)**-1

	phi_acc = np.arctan2( a_y , a_z )
	theta_acc = np.arctan2(-a_x,a_y*np.sin(phi_acc)+a_z*np.cos(phi_acc))
	psi_acc = np.arctan2(np.sqrt(a_y**2+a_x**2),a_z)

	if all(mag) != 0:
		# m_x = mag[0]
		# m_y = mag[1]
		# m_z = mag[2]
		# My = m_x * np.sin(phi_acc) - m_y * np.cos(phi_acc)
		# Mx = m_x * np.cos(theta_acc) + m_y*np.sin(theta_acc) * np.sin(phi_acc) + m_z * np.sin(theta_acc) * np.cos(psi_acc) 
		# psi_mag = 0.1 * np.arctan2( - My , Mx )
		m_x = mag[0]
		m_y = mag[1]
		m_z = mag[2]
		m_vect = np.array([[m_x],[m_y],[m_z]])
		# -----------------------------------------
		R = np.dot(Ry(theta_acc), Rx(phi_acc))
		b = np.dot(R, m_vect) # rotated magnetic field
		b_x, b_y = b[0,0], b[1,0]
		psi_mag = np.arctan2(-b_y, b_x)
	else:
		psi_mag= 0

	gyro = np.array( [	phi_dot_gyro 	* dt  , 
						theta_dot_gyro	* dt  , 
						psi_dot_gyro 	* dt ] )

	accmag = np.array( [phi_acc,theta_acc,psi_mag] )
	#angles = alpha * ( gyro + asset ) + (1 - alpha) * (accmag )
	asset += 0.9* gyro + 0.1 * (accmag - asset)
	return asset

def ComplementaryFilterQuat_2(q, acc,gyro,mag, dt):
	alpha = 0.1
	a_x = acc[0] 
	a_y = acc[1]
	a_z = acc[2]
	g_x = gyro[0] 
	g_y = gyro[1] 
	g_z = gyro[2] 
	# -----------------------------------------
	Omega_t = skew_symmetric_4x4(g_x, g_y, g_z)
	q_gyro_dot = 1/2 * np.dot(Omega_t, q)
	q_gyro = q + q_gyro_dot * dt								# attitude estimated from gyroscope
	# -----------------------------------------
	roll = np.arctan2( a_y , a_z ) #roll angle (rotation along x)
	pitch = np.arctan2(-a_x,a_y*np.sin(roll)+a_z*np.cos(roll))
	if all(mag) != 0:
		m_x = mag[0]
		m_y = mag[1]
		m_z = mag[2]
		m_vect = np.array([[m_x],[m_y],[m_z]])
		# -----------------------------------------
		R = np.dot(Ry(pitch), Rx(roll))
		b = np.dot(R, m_vect) # rotated magnetic field
		b_x, b_y = b[0,0], b[1,0]
		yaw = np.arctan2(-b_y, b_x)
	else:	
		yaw = 0 #get_euler_from_quaternion(q)[2]
	q_acc_magn = get_quaternion_from_euler(pitch, roll, yaw)	# attitude estimated from accelerometer (and magnetometer)
	# -----------------------------------------
	q =  (1 - alpha) * q_gyro + alpha * q_acc_magn			# estimated quaternion
	return q
