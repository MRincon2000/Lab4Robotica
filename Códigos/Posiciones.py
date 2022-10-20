"""
Allows to use the service dynamixel_command 
"""
import rospy
import time
import roboticstoolbox as rtb
import numpy as np
# from std_msgs.msg import String
from dynamixel_workbench_msgs.srv import DynamixelCommand

robot= rtb.DHRobot([
    rtb.RevoluteDH(d=0.045, alpha= -np.pi/2, offset= 0),
    rtb.RevoluteDH(a= 0.105, offset= -np.pi/2),
    rtb.RevoluteDH(a=0.105, offset= np.pi/2),
    rtb.RevoluteDH(a=0.110, offset= np.pi/2)
], name= "Pincher")


__author__ = "F Gonzalez, S Realpe, JM Fajardo"
__credits__ = ["Felipe Gonzalez", "Sebastian Realpe", "Jose Manuel Fajardo", "Robotis"]
__email__ = "fegonzalezro@unal.edu.co"
__status__ = "Test"

def jointCommand(command, id_num, addr_name, value, time):
    #rospy.init_node('joint_node', anonymous=False)
    rospy.wait_for_service('dynamixel_workbench/dynamixel_command')
    try:        
        dynamixel_command = rospy.ServiceProxy(
            '/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        result = dynamixel_command(command,id_num,addr_name,value)
        rospy.sleep(time)
        return result.comm_result
    except rospy.ServiceException as exc:
        print(str(exc))

def calculaAngulo(angulo, ID):
    if ID==1:
        salidaA=int(512+angulo/180*512)
    elif ID==2:
        salidaA=int(512-angulo/130*512)
    elif ID==3:
        salidaA=int(220-angulo/130*512)
    elif ID==4:
        salidaA=int(230-angulo/1301*512)
        if salidaA <200:
            salidaA=200
    return salidaA




if __name__ == '__main__':
    try:
        # Goal_Position (0,1023)
        # Torque_Limit (0,1023)
        jointCommand('', 1, 'Torque_Limit', 500, 0)
        jointCommand('', 2, 'Torque_Limit', 500, 0)
        jointCommand('', 3, 'Torque_Limit', 400, 0)
        jointCommand('', 4, 'Torque_Limit', 400, 0)
        
        
        while(True):
            posicion=int(input("Ingrese un numero de posicion: "))
            if(posicion== 1):
                jointCommand('',1, 'Goal_Position', calculaAngulo(0,1), 0)
                jointCommand('',2, 'Goal_Position', calculaAngulo(0,2), 0)
                jointCommand('',3, 'Goal_Position', calculaAngulo(0,3), 0)
                jointCommand('',4, 'Goal_Position', calculaAngulo(0,4), 0)
                robot.plot([0, 0, 0, 0])
            elif(posicion==2):
                jointCommand('',1, 'Goal_Position', calculaAngulo(-20,1), 0)
                jointCommand('',2, 'Goal_Position', calculaAngulo(20,2), 0)
                jointCommand('',3, 'Goal_Position', calculaAngulo(-20,3), 0)
                jointCommand('',4, 'Goal_Position', calculaAngulo(20,4), 0)
                robot.plot([np.pi/180*-20, np.pi/180*20, np.pi/180*-20, np.pi/180*20])
            elif(posicion==3):
                jointCommand('',1, 'Goal_Position', calculaAngulo(30,1), 0)
                jointCommand('',2, 'Goal_Position', calculaAngulo(-30,2), 0)
                jointCommand('',3, 'Goal_Position', calculaAngulo(30,3), 0)
                jointCommand('',4, 'Goal_Position', calculaAngulo(-30,4), 0)
                robot.plot([np.pi/180*30, np.pi/180*-30, np.pi/180*30, np.pi/180*-30])
            elif(posicion==4):
                jointCommand('',1, 'Goal_Position', calculaAngulo(-90,1), 0)
                jointCommand('',2, 'Goal_Position', calculaAngulo(15,2), 0)
                jointCommand('',3, 'Goal_Position', calculaAngulo(-55,3), 0)
                jointCommand('',4, 'Goal_Position', calculaAngulo(10,4), 0)
                robot.plot([np.pi/180*-90, np.pi/180*15, np.pi/180*-55, np.pi/180*10])
            elif(posicion==5):
                jointCommand('',1, 'Goal_Position', calculaAngulo(-90,1), 0)
                jointCommand('',2, 'Goal_Position', calculaAngulo(45,2), 0)
                jointCommand('',3, 'Goal_Position', calculaAngulo(-55,3), 0)
                jointCommand('',4, 'Goal_Position', calculaAngulo(10,4), 0)
                robot.plot([np.pi/180*-90, np.pi/180*45, np.pi/180*-55, np.pi/180*10])


            else: 
                break
        
    except rospy.ROSInterruptException:
        pass