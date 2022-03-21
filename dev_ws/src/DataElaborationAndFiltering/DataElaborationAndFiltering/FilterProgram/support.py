#!/usr/bin/env python
# encoding: utf-8
import numpy as np 
from FilterProgram.constants import * 


def remap(data):
    #align mpu axis to NED
    r = np.zeros(10)
    r[0] = data[0]
    r[1] = data[1]
    r[2] = data[2]
    r[3] = data[3] 
    r[4] = data[4] 
    r[5] = data[5] 
    r[6] = data[7]
    r[7] = data[6]
    r[8] = -data[8]    

    r[9] = data[9]  
    return r

def get_data(msg) :
    data = msg.data
    data = remap(data)
    acc = np.zeros(3)
    gyro = np.zeros(3)
    mag = np.zeros(3)
    acc[0] = data[0]
    acc[1] = data[1]
    acc[2] = data[2]
    gyro[0] = data[3] * DEG_TO_RAD
    gyro[1] = data[4] * DEG_TO_RAD
    gyro[2] = data[5] * DEG_TO_RAD
    mag[0] = data[6]
    mag[1] = data[7]
    mag[2] = data[8]
    dt = msg.data[9]
    return [ acc, gyro, mag , dt]

def Rx(a):
	R = np.array([	[		1,			0,			0	],
				[		0,		np.cos(a),		np.sin(a)	],
				[		0,		-np.sin(a),	np.cos(a)	]])
	return R

def Ry(a):
	R = np.array([	[	np.cos(a),			0,		-np.sin(a)	],
				[		0,			1,		0		],
				[	np.sin(a),			0,		np.cos(a)	]])
	return R

def Rz(a):
	R = np.array([	[	np.cos(a),		np.sin(a),	0	],
				[	-np.sin(a),	np.cos(a),		0	],
				[		0,			0,		1	]])
	return R

def skew_symmetric_4x4(a, b, c):
	Omega = np.array([	[	0,		-a,		-b,		-c	],
					[	a,		0,		c,		-b	],
					[	b,		-c,		0,		a	],
					[	c,		b,		-a,		0	]])
	return Omega

def get_euler_from_quaternion(quat):
	quat = np.reshape(quat,(4,1))
	q0 = quat[0]
	q1 = quat[1]
	q2 = quat[2]
	q3 = quat[3]

	roll = np.arctan2(2*(q0*q1 + q2*q3), 1 - 2*(np.power(q1,2) + np.power(q2,2)))
	check = 2*(q1*q3 - q0*q2)
	if check >= 1:	pitch = np.arcsin(check-1) # use 90 degrees if out of range
	if check <= - 1:	pitch = np.arcsin(check+1) # use 90 degrees if out of range		
	if -1 < check < 1 :	pitch = np.arcsin(check) # use 90 degrees if out of range	
	pitch = np.arcsin(check)  #RICONTROLLARE MENO
	yaw = np.arctan2(2*(q0*q3 + q1*q2), 1 - 2*(np.power(q2,2) + np.power(q3,2)))
	pose = np.asarray([roll, pitch, yaw])
	pose = pose.reshape((3))
	return pose

def get_quaternion_from_euler(roll, pitch, yaw):
	a = np.cos(roll/2)
	b = np.cos(pitch/2)
	c = np.cos(yaw/2)
	d = np.sin(roll/2)
	e = np.sin(pitch/2)
	f = np.sin(yaw/2)
	quat = np.array(	[a*b*c + d*e*f,
						e*a*c - b*d*f,
						b*d*c + e*a*f,
						b*a*f - e*d*c])
	return quat