#!/usr/bin/env python
""" arucoNav.py - Version 1.0 10-8-2020
    Autor: David Barrera
    Codigo modificado a partir de: https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/
"""

import rospy
import math
import time
import numpy
import tf
# Brings in the SimpleActionClient
import actionlib

from geometry_msgs.msg import Quaternion, PoseWithCovarianceStamped, PoseStamped
from tf.transformations import quaternion_from_euler
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class arucoNavigation():
    def __init__(self):
        # Para el action
        self.goal = MoveBaseGoal() #Este dato es equivalente al PoseStamped: 3 coordenadas de posicion (x,y,z) y un cuaternion para la orientacion
        self.goal_x=0.0
        self.goal_y=0.0
        self.goal_theta=0.0

        # Para la posicion de la marca
        self.mark_x=0.0
        self.mark_y=0.0
        self.mark_z=0.0
        self.mark_alpha=0.0
        self.mark_beta=0.0
        self.mark_theta=0.0

        self.mark_theta_list=[]
        self.mark_theta_median=0.0



# Funciones para la navegacion
def goal2MoveBaseGoal(x=0.0, y=0.0, theta=0): #Funcion que transforma una posicion dada por x, y e angulo en z (en grados), en formato MoveBaseGoal

    arucoNav.goal.target_pose.header.frame_id = "campero_map"
    arucoNav.goal.target_pose.header.stamp = rospy.Time.now()

    #Definimos la posicion
    arucoNav.goal.target_pose.pose.position.x = x
    arucoNav.goal.target_pose.pose.position.y = y
    arucoNav.goal.target_pose.pose.position.z = 0.0

    #Definimos la orientacion
    q=quaternion_from_euler(0.0, 0.0, numpy.deg2rad(theta))
    arucoNav.goal.target_pose.pose.orientation=Quaternion(*q)



def movebase_client():

   # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('/campero/move_base',MoveBaseAction)

   # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

    if arucoNav.mark_x==0.0 and arucoNav.mark_y==0.0 and arucoNav.mark_theta==0.0: #Si los valores no se han actualizado es que la camara no ve la marca
        marca_detectada=0
        print('No detecto a la marca, por favor cancela el programa')
    else:
        marca_detectada=1
        print('Marca detectada correctamente')
    
    if marca_detectada:
        arucoNav.goal_x = arucoNav.mark_x
        arucoNav.goal_y = arucoNav.mark_y
        arucoNav.goal_theta = arucoNav.mark_theta_median

        # Mostramos la informacion por pantalla
        rospy.loginfo(' ---------------------- ')

        rospy.loginfo('mark_x= %f ', arucoNav.mark_x)
        rospy.loginfo('mark_y= %f ', arucoNav.mark_y)
        rospy.loginfo('mark_z= %f ', arucoNav.mark_z)
        rospy.loginfo('mark_theta= %f ', arucoNav.mark_theta)
        rospy.loginfo('mark_theta_median= %f ', arucoNav.mark_theta_median)

        rospy.loginfo(' ---------------------- ')

        rospy.loginfo('goal_x= %f ', arucoNav.goal_x)
        rospy.loginfo('goal_y %f ', arucoNav.goal_y)
        rospy.loginfo('goal_theta= %f ', arucoNav.goal_theta)

        rospy.loginfo(' ---------------------- ')


        goal2MoveBaseGoal(arucoNav.goal_x,arucoNav.goal_y,arucoNav.goal_theta)   #Creamos el goal en el formato MoveBaseGoal

    # Sends the goal to the action server.
        client.send_goal(arucoNav.goal)
        print("Yendo hacia la marca")

    # Waits for the server to finish performing the action.
        wait = client.wait_for_result()
        
    # If the result doesn't arrive, assume the Server is not available
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
        # Result of executing the action
            print("Estoy frente a la marca")
            return client.get_result()
     


# Averiguamos la posicion del robot
def arucoCallback(aruco_pose_message):
    arucoNav.mark_x=aruco_pose_message.pose.position.x
    arucoNav.mark_y=aruco_pose_message.pose.position.y
    arucoNav.mark_z=aruco_pose_message.pose.position.z
    rotation=aruco_pose_message.pose.orientation
    angles=tf.transformations.euler_from_quaternion(quaternion=(rotation.x, rotation.y, rotation.z, rotation.w))
    arucoNav.mark_alpha=numpy.rad2deg(angles[0])
    arucoNav.mark_beta=numpy.rad2deg(angles[1])
    arucoNav.mark_theta=numpy.rad2deg(angles[2])

    # Realizamos un filtrado del angulo theta, para ello guardamos 10 valores y sacamos la mediana
    if len(arucoNav.mark_theta_list)>9:
        arucoNav.mark_theta_list.remove(arucoNav.mark_theta_list[0])
    if not len(arucoNav.mark_theta_list)==0:
        arucoNav.mark_theta_median=numpy.median(arucoNav.mark_theta_list)
    else:
        arucoNav.mark_theta_median=arucoNav.mark_theta
    
    # Si queremos ver la localizacion de la marca en todo momento descomentar esta parte 
    #rospy.loginfo('La marca se encuentra la posicion: x= %f , y= %f ,z= %f con la orientacion: alpha= %f , beta= %f , theta= %f', arucoNav.mark_x,arucoNav.mark_y,arucoNav.mark_z,arucoNav.mark_alpha,arucoNav.mark_beta,arucoNav.mark_theta)
    #rospy.loginfo('x= %f ', arucoNav.mark_x)
    #rospy.loginfo('y= %f ', arucoNav.mark_y)
    #rospy.loginfo('z= %f ', arucoNav.mark_z)
    #rospy.loginfo('alpha= %f ', arucoNav.mark_alpha)
    #rospy.loginfo('beta= %f ',arucoNav. mark_beta)
    #rospy.loginfo('theta= %f ', arucoNav.mark_theta)
    #rospy.loginfo('theta_median= %f ', arucoNav.mark_theta_median)

def listener():
    rospy.Subscriber('aruco_pose', PoseStamped, arucoCallback)

def normpi(angle): # Normaliza el angulo para que solo tenga valores entre -180 y 180 grados
    if angle > 180.0:
        angle = angle - 360.0
    elif angle <= -180.0:
        angle = angle + 360.0
    return angle    
    


    # If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
        rospy.init_node('aruco_navigation') 
        arucoNav=arucoNavigation()
        listener()
        result = movebase_client()
        if not result:
            rospy.spin()
        if rospy.is_shutdown():
            rospy.loginfo("Se ha cancelado el programa")
        elif result:
            rospy.loginfo("Programa finalizado")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
