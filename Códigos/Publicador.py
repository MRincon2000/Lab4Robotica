import rospy
import numpy as np
import time
import roboticstoolbox as rtb
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

robot= rtb.DHRobot([
    rtb.RevoluteDH(d=0.045, alpha= -np.pi/2),
    rtb.RevoluteDH(a= 0.105),
    rtb.RevoluteDH(a=0.105),
    rtb.RevoluteDH(a=0.110)
], name= "Pincher")



def joint_publisher():
    pub = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=0)
    rospy.init_node('joint_publisher', anonymous=False)


    posicionActual=[0, 0, -np.pi/2, -np.pi/2, 0]

    while True:


        state = JointTrajectory()
        state.header.stamp = rospy.Time.now()
        state.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
        point = JointTrajectoryPoint()
        actual=int(input("Ingrese la articulación que desea manipular: "))
        if actual<1 or actual>5:
            break
        else:
            angDes=int(input("Ingrese el angulo deseado para la articulacion: "))

            if actual==1:
                ang=angDes+0
                if ang>-180 and ang<180:
                    posicionActual[0]=ang*np.pi/180
                elif ang<=-180:
                    angDes=-179
                    print("Posicion no admitida, moviendo a la mas cercana posible.")
                    posicionActual[0]=-179*np.pi/180
                else:
                    angDes=179
                    print("Posicion no admitida, moviendo a la mas cercana posible.")
                    posicionActual[0]=179*np.pi/180
            elif actual==2:
                ang=angDes+0
                if ang>-100 and ang<100:
                    posicionActual[1]=-ang*np.pi/180
                elif ang<=-100:
                    angDes=-99
                    print("Posicion no admitida, moviendo a la mas cercana posible.")
                    posicionActual[1]=99*np.pi/180
                else:
                    angDes=100
                    print("Posicion no admitida, moviendo a la mas cercana posible.")
                    posicionActual[1]=-99*np.pi/180
            elif actual==3:
                ang=angDes+90
                if ang>-130 and ang<130:
                    posicionActual[2]=-ang*np.pi/180
                elif ang<=-130:
                    angDes=-219
                    print("Posicion no admitida, moviendo a la mas cercana posible.")
                    posicionActual[2]=129*np.pi/180
                else:
                    angDes=40
                    print("Posicion no admitida, moviendo a la mas cercana posible.")
                    posicionActual[2]=-129*np.pi/180
            elif actual==4:
                ang=angDes+90
                if ang>-100 and ang<100:
                    posicionActual[3]=-ang*np.pi/180
                elif ang<=-100:
                    angDes=-190
                    print("Posicion no admitida, moviendo a la mas cercana posible.")
                    posicionActual[3]=89*np.pi/180
                else:
                    angDes=10
                    print("Posicion no admitida, moviendo a la mas cercana posible.")
                    posicionActual[3]=-99*np.pi/180
            elif actual==5:
                if ang>-90 and ang<90:
                    posicionActual[4]=-ang*np.pi/180
                elif ang<=-90:
                    ang=-89
                    print("Posicion no admitida, moviendo a la mas cercana posible.")
                    posicionActual[4]=89*np.pi/180
                else:
                    ang=89
                    print("Posicion no admitida, moviendo a la mas cercana posible.")
                    posicionActual[4]=-89*np.pi/180


            robot.plot([posicionActual[0],-np.pi/2+posicionActual[1],posicionActual[2],posicionActual[3]])
            point.positions = posicionActual  
            point.time_from_start = rospy.Duration(0.5)
            state.points.append(point)
            pub.publish(state)
            print('Articulacion editada: ', actual, ", ", "Nuevo angulo: ", angDes)
            rospy.sleep(1)


if __name__ == '__main__':
    try:
        time.sleep(1)
        joint_publisher()
    except rospy.ROSInterruptException:
        pass