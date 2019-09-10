#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import numpy
import math

x = 0 
y = 0
e_x = 0
e_y = 0


def talker_roll_pitch(roll,pitch):
    pub = rospy.Publisher('roll_pitch_Ya', String, queue_size=20)
    if not rospy.is_shutdown():
        ya = 1
        roll_pitch_ya = str(roll) +','+ str(pitch) +',' + str(ya)
        pub.publish(roll_pitch_ya)


def zero_offset(x,y):
    # make in center for width = 600
    x = x - 300
    y = y - 150
    x = x / 16 # cm
    y = y / 16 # cm
    return (x,y)

def roll_pitch(x,y):
    # x, y should be in cm
    z = 30 
    roll_ratio = x / z
    pitch_ratio = y / z
    roll = float( math.degrees(math.atan(roll_ratio)) )
    pitch = float(math.degrees(math.atan(pitch_ratio)))

    return (roll,pitch)




def callback(req):
    try:
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", req.data)
        global x,y
        data = req.data.split(',')
        x = float(data[0])
        y = float(data[1])
        (x,y) = zero_offset(x,y)
        print("x,y is :",x,y)
        (roll, pitch) = roll_pitch(x,y)
        print("roll,pitch is :",roll,pitch)
        talker_roll_pitch(roll,pitch)
    except: 
        print('exception catched.')
        pass
        
    
def listener():
    global x,y
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('point_listener', anonymous=False)

    rospy.Subscriber("point_data", String, callback)

    #  pub = rospy.Publisher('topic1', String, queue_size=10)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()