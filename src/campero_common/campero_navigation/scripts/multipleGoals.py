#!/usr/bin/env python
""" multipleGoals.py - Version 1.0 10-8-2020
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

class goal:
	def __init__(self, x, y, theta):
		self.x=x			
		self.y=y
		self.theta=theta

		self.createMoveBaseGoal()	#El atributo que usaremos para la navegacion sera del tipo MoveBaseGoal


	def createMoveBaseGoal(self):	#Damos valor al atributo MoveBaseGoal en funcion de x, y, theta
		self.MoveBaseGoal=MoveBaseGoal()

		self.MoveBaseGoal.target_pose.header.frame_id = "campero_map"
		self.MoveBaseGoal.target_pose.header.stamp = rospy.Time.now()

		#Definimos la posicion
		self.MoveBaseGoal.target_pose.pose.position.x = self.x
		self.MoveBaseGoal.target_pose.pose.position.y = self.y
		self.MoveBaseGoal.target_pose.pose.position.z = 0.0

		#Definimos la orientacion
		q=quaternion_from_euler(0.0, 0.0, numpy.deg2rad(self.theta))
		self.MoveBaseGoal.target_pose.pose.orientation=Quaternion(*q)


def movebase_client():

   # Create an action client called "move_base" with action definition file "MoveBaseAction"
	client = actionlib.SimpleActionClient('/campero/move_base',MoveBaseAction)

   # Waits until the action server has started up and started listening for goals.
	client.wait_for_server()

	goalSequence=[]			#Creamos el array que contiene la secuencia de goals
	goal1=goal(-1.9,0.5,0)   #Creamos el goal en el formato MoveBaseGoal
	goal2=goal(1.5,-1.5,180)
	goal3=goal(-1.9,0.5,0)
	goal4=goal(1.5,-1.5,180)
	goal5=goal(-1.9,0.5,0)
        goal6=goal(1.5,-1.5,0)

	goalSequence=[goal1, goal2, goal3, goal4, goal5, goal6]
	n=1
	
	while not len(goalSequence)==0 and not rospy.is_shutdown():
	# Sends the goal to the action server.
		client.send_goal(goalSequence[0].MoveBaseGoal)
		print("Yendo al goal"+str(n))

	# Waits for the server to finish performing the action.
		wait = client.wait_for_result()
		
	# If the result doesn't arrive, assume the Server is not available
		if not wait:
			rospy.logerr("Action server not available!")
			rospy.signal_shutdown("Action server not available!")
		else:
			# Result of executing the action
			print("He llegado al goal"+str(n))
			goalSequence.pop(0)		#Eliminamos el punto al que el robot acaba de llegar
			n=n+1
	
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
			rospy.loginfo("Se ha completado la ruta")

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
