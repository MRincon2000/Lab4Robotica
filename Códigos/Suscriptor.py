#!/usr/bin/env python
import rospy
import numpy as np
import roboticstoolbox as rtb
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import time as t

robot= rtb.DHRobot([
    rtb.RevoluteDH(d=0.045, alpha= -np.pi/2),
    rtb.RevoluteDH(a= 0.105),
    rtb.RevoluteDH(a=0.105),
    rtb.RevoluteDH(a=0.110)
], name= "Pincher")


def callback(data):
    art1=(data.position[0])*180/np.pi
    art2=-(data.position[1])*180/np.pi
    art3=-(data.position[2]+np.pi/2)*180/np.pi
    art4=-(data.position[3]+np.pi/2)*180/np.pi
    art5=-data.position[4]
    print("art1= ", art1, ", art2= ", art2, ", art3= ", art3, ", art4= ", art4,", art5= ", art5)


    
def listener():
    rospy.init_node('joint_listener', anonymous=True)
    rospy.Subscriber("/dynamixel_workbench/joint_states", JointState, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            listener()
    except rospy.ROSInterruptException:
        pass