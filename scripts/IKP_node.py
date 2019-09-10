#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import numpy as np
import math

ERROR_MARGIN = 0.00001

Q = np.array([[-0.7071,7.707 *0.000001,0.7072],[-0.4083,0.8165, -0.4082],[-0.5774,-0.5773,-0.5773]])

Q_inv = np.array([[-0.706993872686615 , -0.408247422194984 , -0.577411171009842],
		    [-0.0000169093916079563 ,0.816516732104659 , -0.577367206594261],
		    [0.707133247671967,-0.4081998593224728,-0.577323231408368]])

class IKP():

	def __init__(self, rotationAngles_deg):
		self.rotationAngles_deg = rotationAngles_deg
		# self.Q = np.array([[-0.7071,7.707 *0.000001,0.7072],[-0.4083,0.8165, -0.4082],[-0.5774,-0.5773,-0.5773]])
		# self.Q_inv = np.array([[-0.706993872686615 , -0.408247422194984 , -0.577411171009842],
		#     [-0.0000169093916079563 ,0.816516732104659 , -0.577367206594261],
		#     [0.707133247671967,-0.4081998593224728,-0.577323231408368]])
        

	def run(self):
		global Q,Q_inv
		rotationAngles_rad = self.rotationAngles_deg*math.pi/180
		RotationMatrixCam_Frame = self.cal_rotationMatrix(rotationAngles_rad) 
		RotationMatrixEEF_Frame = Q_inv.dot(RotationMatrixCam_Frame.dot( Q))
		motorAngles_deg = self.cal_angles(RotationMatrixEEF_Frame)
		# print(motorAngles_deg)
		return motorAngles_deg

	def cal_rotationMatrix(self, rotationAngles_rad):
		cosS = math.cos(rotationAngles_rad[0])
		sinS = math.sin(rotationAngles_rad[0])
		cost = math.cos(rotationAngles_rad[1])
		sint = math.sin(rotationAngles_rad[1])
		cosf = math.cos(rotationAngles_rad[2])
		sinf = math.sin(rotationAngles_rad[2])
		return np.array([[cosf*cost , -sinf*cosS+cosf*sint*sinS , sinf*sinS+cosf*sint*cosS],
				[sinf*cost ,cosf*cosS+sinf*sint*sinS , -cosf*sinS+sinf*sint*cosS],
				[-sint , cost*sinS , cost*cosS]])

	def cal_angles(self,RotationMatrixEEF_Frame):
		motorAngles = np.array([0.0,0.0,0.0]) 
		if abs(RotationMatrixEEF_Frame[0][2]) < ERROR_MARGIN :
			motorAngles[0] = 0.0
		else :
			motorAngles[0] = math.atan(RotationMatrixEEF_Frame[0][1]/RotationMatrixEEF_Frame[0][2])
		if abs(RotationMatrixEEF_Frame[1][0]) < ERROR_MARGIN :
			motorAngles[1] = 0.0
		else :
			motorAngles[1] = math.atan(RotationMatrixEEF_Frame[1][2]/RotationMatrixEEF_Frame[1][0])
		if abs(RotationMatrixEEF_Frame[2][1]) < ERROR_MARGIN  :
			motorAngles[2] = 0.0
		else :
			motorAngles[2] = math.atan(RotationMatrixEEF_Frame[2][0]/RotationMatrixEEF_Frame[2][1])
		return motorAngles * 180.0/math.pi



def calculate_IKP(rotationAngles_deg):
    ikp = IKP(rotationAngles_deg)
    motorAngles_deg = ikp.run()
    return motorAngles_deg

def rotation_maker(data):
    data = data.split(',')
    # print(data)
    roll = float(data[0])
    pitch = float(data[1])
    ya = float(data[2])
    if len(data) is not 3:
        raise Exception("data size error")
    rotation = np.array([roll, pitch, ya])
    return rotation


def talker_motors_tetas(motors_tetas):
    pub = rospy.Publisher('Motors_theta', String, queue_size=20)
    if not rospy.is_shutdown():
        motors_teta = str(motors_tetas[0]) +','+ str(motors_tetas[1]) + ','+ str(motors_tetas[2])
        print ( motors_teta , "published")
        pub.publish(motors_teta)


def callback(req):
    try:
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", req.data)
        rotationAngles_deg = rotation_maker(req.data)
        motors_tetas = calculate_IKP(rotationAngles_deg)
        talker_motors_tetas(motors_tetas)
        
    except Exception,e :
        print(e)
        pass

def listener():
    global x,y
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('IKP_node', anonymous=False)

    rospy.Subscriber("roll_pitch_Ya", String, callback)

    # rotationAngles_deg = np.array([4.0, 4.0 , 4.0])
    #  pub = rospy.Publisher('topic1', String, queue_size=10)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
      listener()