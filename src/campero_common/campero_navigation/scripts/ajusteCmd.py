#!/usr/bin/env python
""" arucoCmd.py - Version 1.0 4-9-2020
    Autor: David Barrera
"""

import rospy
import math
import time
import numpy
import tf


from geometry_msgs.msg import Quaternion, PoseWithCovarianceStamped, PoseStamped
from tf.transformations import quaternion_from_euler

from geometry_msgs.msg import Twist


class arucoNavigation():
    def __init__(self):
		# Para la velocidad del robot
		self.vel = Twist() 
		self.vel.linear.x=0.0
		self.vel.linear.y=0.0
		self.vel.linear.z=0.0

		self.vel.angular.x=0.0
		self.vel.angular.y=0.0
		self.vel.angular.z=0.0

		# Para la posicion de la marca
		self.mark_x=0.0
		self.mark_y=0.0
		self.mark_z=0.0
		self.mark_alpha=0.0
		self.mark_beta=0.0
		self.mark_theta=0.0

		self.mark_theta_list=[]
		self.mark_theta_median=0.0

		# Para saber si esta bien posicionado
		self.orientado=0
		self.desplazado=0


		# Para saber si el robot ha llegado a la posicion objetivo
		self.fin=0


def navigation():

	#Orientacion
	if (arucoNav.orientado==0):
		if (arucoNav.mark_theta_median<-0.05):
			arucoNav.vel.angular.z=-0.05
			print("Girando derecha")
		elif (arucoNav.mark_theta_median>0.05):
			arucoNav.vel.angular.z=0.05
			print("Girando izquierda")
		elif (arucoNav.mark_theta_median>-0.05) and (arucoNav.mark_theta_median<0.05):
			arucoNav.vel.angular.z=0.0	
			arucoNav.orientado=1
			print("Robot orientado")
		


	# Desplazamiento
	if (arucoNav.orientado==1):
		if (arucoNav.mark_x>1.02):				#Si esta lejos avanza en x
			arucoNav.vel.linear.x=0.05
			arucoNav.vel.linear.y=0.0
			print("Voy hacia delante")
			if (arucoNav.mark_y>0.05):			#Si no esta centrado se desplaza lateralmente
				arucoNav.vel.linear.y=-0.05
			elif (arucoNav.mark_y<-0.05):
				arucoNav.vel.linear.y=0.05
		elif (arucoNav.mark_x<0.98):				#Si se ha pasado retrocede en x
			arucoNav.vel.linear.x=-0.05
			arucoNav.vel.linear.y=0.0
			print("Voy hacia detras")
			if (arucoNav.mark_y>0.05):			#Si no esta centrado se desplaza lateralmente
				arucoNav.vel.linear.y=0.05
			elif (arucoNav.mark_y<-0.05):
				arucoNav.vel.linear.y=-0.05
		elif (arucoNav.mark_x>0.98) and (arucoNav.mark_x<1.02):				#Si esta a la distancia correcta se detiene en x
			arucoNav.vel.linear.x=0.0
			arucoNav.vel.linear.y=0.0
			print("Orientado y centrado en x")
			if (arucoNav.mark_y>0.05):			#Si no esta centrado se desplaza lateralmente
				arucoNav.vel.linear.y=0.05
			elif (arucoNav.mark_y<-0.05):
				arucoNav.vel.linear.y=-0.05
			else:								#Si esta centrado y cerca, esta en la posicion deseada
				arucoNav.desplazado=1
				print("Robot en posicion correcta")



	# Si el robot esta bien posicionado en x e y y ademas esta orientado en z, ha cumplido el objetivo
	if (arucoNav.orientado and arucoNav.desplazado):
		arucoNav.vel.angular.z = 0.0
		arucoNav.vel.linear.x = 0.0
		arucoNav.vel.linear.y = 0.0
		arucoNav.fin=1
		print("Fin de la navegacion")
	pub.publish(arucoNav.vel)

	#Si queremos ver las velocidades que se publican en cada momento descomentar esta parte
	#rospy.loginfo('velocidad lineal en x: %f ', arucoNav.vel.linear.x)
	#rospy.loginfo('velocidad lineal en y: %f ', arucoNav.vel.linear.y)
	#rospy.loginfo('velocidad angular en z: %f ', arucoNav.vel.angular.z)
	#rospy.loginfo('theta_median= %f ', arucoNav.mark_theta_median)





# Averiguamos la posicion del robot respecto a la marca
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
    rospy.Subscriber('aruco_distance', PoseStamped, arucoCallback)  


    # If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
	try:
		rospy.init_node('aruco_navigation') 
		pub = rospy.Publisher('/campero/cmd_vel',Twist,queue_size=10)
		arucoNav=arucoNavigation()
		listener()
		rospy.sleep(1)
		print("Inicio de la navegacion")
		while not arucoNav.fin and not rospy.is_shutdown():
			navigation()
		#if rospy.is_shutdown():
		if rospy.is_shutdown():
			rospy.loginfo("Se ha cancelado el programa")
		else:
			rospy.loginfo("El robot ha llegado a su destino")
	except rospy.ROSInterruptException:
		rospy.loginfo("Navigation test finished.")
