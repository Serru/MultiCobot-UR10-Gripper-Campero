#!/usr/bin/env python
""" arucoNav.py - Version 0.3 17-9-2020
    Autor: David Barrera
    Esta version realiza el giro con rospy.Timer()
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
		self.cerca=0
		self.centrado=0

		# Para saber si el robot ha llegado a la posicion objetivo
		self.fin=0
		self.girocompleto=0


def navigation():

	if not rospy.is_shutdown():
		# Avance del robot
		if (arucoNav.mark_x<-0.25):	# Si ya esta lo suficientemente cerca no avanza
			arucoNav.vel.linear.x=0.0
			arucoNav.cerca=1
		#Fase de avance rapido
		elif (arucoNav.mark_x>0.3):
			arucoNav.vel.linear.x=0.15	#Simulacion 0.3
			arucoNav.vel.angular.z=0.0
		#Fase de avance lento
		elif (arucoNav.mark_x>=0.1):
			arucoNav.vel.linear.x=0.10	#Simulacion: 0.15
			arucoNav.vel.angular.z=0.0


		# Desplazamiento lateral del robot

		if (arucoNav.mark_y<0.3) and (arucoNav.mark_y>-0.3): # Si esta centrado en la marca no se desplaza
			arucoNav.vel.linear.y=0.0
			arucoNav.centrado=1

		# Fase de esplazamiento rapido
		elif (arucoNav.mark_y>0.3):
			arucoNav.vel.linear.y=0.1		#Simulacion: 0.3
			arucoNav.vel.angular.z=0.0
		elif (arucoNav.mark_y<-0.3):			
			arucoNav.vel.linear.y=-0.1		#Simulacion: 0.3
			arucoNav.vel.angular.z=0.0	

		#Fase de desplazamiento lento
		#elif (arucoNav.mark_y>0.05):
		#	arucoNav.vel.linear.y=0.1
		#	arucoNav.vel.angular.z=0.0
		#elif (arucoNav.mark_y<-0.05):
		#	arucoNav.vel.linear.y=-0.1
		#	arucoNav.vel.angular.z=0.0


		# Rotacion del robot

		if((arucoNav.mark_theta_median<2.0)and(arucoNav.mark_theta_median>-2.0)): # Si el robot esta bien orientado no gira
			arucoNav.vel.angular.z=0.0
			arucoNav.orientado=1
		#Fase de giro rapido
		elif (arucoNav.mark_theta_median>10.0):
			arucoNav.vel.linear.x=0.0
			arucoNav.vel.linear.y=0.0
			arucoNav.vel.angular.z=0.3
		elif(arucoNav.mark_theta_median<-10.0):
			arucoNav.vel.linear.x=0.0
			arucoNav.vel.linear.y=0.0
			arucoNav.vel.angular.z=-0.3
		# Fase de giro lento
		elif (arucoNav.mark_theta_median>2.0) and arucoNav.mark_x<0.5:
			arucoNav.vel.linear.x=0.0
			arucoNav.vel.linear.y=0.0
			arucoNav.vel.angular.z=0.10		#Simulacion: 0.15
		elif(arucoNav.mark_theta_median<-2.0) and arucoNav.mark_x<0.5:
			arucoNav.vel.linear.x=0.0
			arucoNav.vel.linear.y=0.0
			arucoNav.vel.angular.z=-0.10		#Simulacion: 0.15

		# Si el robot esta bien posicionado en x e y y ademas esta orientado en z, ha cumplido el objetivo
		if (arucoNav.orientado and arucoNav.centrado and arucoNav.cerca):
			arucoNav.fin=1
		pub.publish(arucoNav.vel)

		#print("arucoNav.cerca: "+str(arucoNav.cerca))
		#print("arucoNav.centrado: "+str(arucoNav.centrado))
		#print("arucoNav.orientado: "+str(arucoNav.orientado))

		#if (arucoNav.mark_theta_median>1):
		#arucoNav.vel.linear.x=0.3

		#rospy.loginfo('velocidad lineal en x: %f ', arucoNav.vel.linear.x)
		#rospy.loginfo('velocidad lineal en y: %f ', arucoNav.vel.linear.y)
		#rospy.loginfo('velocidad angular en z: %f ', arucoNav.vel.angular.z)
		#rospy.loginfo('theta_median= %f ', arucoNav.mark_theta_median)
		#print(arucoNav.orientado)



def giro():
	print("Empiezo a girar durante 6 segundos")			#Simulacion 7.5 segundos

	rospy.Timer(rospy.Duration(6),my_callback,oneshot=True)

	while (not arucoNav.girocompleto==1) and not rospy.is_shutdown():
		arucoNav.vel.angular.z=-0.5
		pub.publish(arucoNav.vel)
		
		#print("Estoy girando")
	
	print("Giro terminado")
	arucoNav.girocompleto=0
	arucoNav.vel.angular.z=0.0
	pub.publish(arucoNav.vel)


def my_callback(event):
	print("Han pasado 6 segundos")
	arucoNav.girocompleto=1


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
		pub = rospy.Publisher('/campero/cmd_vel',Twist,queue_size=10)
		arucoNav=arucoNavigation()
		listener()
		rospy.sleep(1)
		while not rospy.is_shutdown():
			print("Voy hacia la marca")
			
			arucoNav.orientado=0
			arucoNav.cerca=0
			arucoNav.centrado=0
			arucoNav.fin=0
			while not arucoNav.fin and not rospy.is_shutdown():
				navigation()
			print("Acercamiento a la marca completado")
			giro()
			rospy.sleep(2)
			
		#if rospy.is_shutdown():
		if rospy.is_shutdown():
			rospy.loginfo("Se ha cancelado el programa")
	except rospy.ROSInterruptException:
		rospy.loginfo("Navigation test finished.")
