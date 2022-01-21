#!/usr/bin/env python
""" actionGoal.py - Version 1.0 10-8-2020
    Autor: David Barrera
    Codigo modificado a partir de: https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/
"""

import rospy
import math
import time
import numpy
# Brings in the SimpleActionClient
import actionlib

from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


goal = MoveBaseGoal() #Este dato es equivalente al PoseStamped: 3 coordenadas de posicion (x,y,z) y un cuaternion para la orientacion

# Coordenadas a las que se dirigira el robot
x_goal=-1.6 # ref. pos = -1.6
y_goal=1.2 # ref. pos = 1.2
theta_goal=0.0

def goal2MoveBaseGoal(x=0.0, y=0.0, theta=0): #Funcion que transforma una posicion dada por x, y e angulo en z (en grados), en formato MoveBaseGoal

    goal.target_pose.header.frame_id = "campero_map"
    goal.target_pose.header.stamp = rospy.Time.now()

    #Definimos la posicion
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = 0.0

    #Definimos la orientacion
    q=quaternion_from_euler(0.0, 0.0, numpy.deg2rad(theta))
    goal.target_pose.pose.orientation=Quaternion(*q)


def movebase_client():

   # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('/campero/move_base',MoveBaseAction)

   # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

    goal2MoveBaseGoal(x_goal,y_goal,theta_goal)   #Creamos el goal en el formato MoveBaseGoal

   # Sends the goal to the action server.
    client.send_goal(goal)
    print("Yendo al goal")

   # Waits for the server to finish performing the action.
    wait = client.wait_for_result()
    
   # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Result of executing the action
        return client.get_result()   


    # If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
        # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('movebase_client_py')
        result = movebase_client()
        if rospy.is_shutdown():
            rospy.loginfo("Se ha cancelado la ruta")
        elif result:
            rospy.loginfo("He llegado al goal")

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
