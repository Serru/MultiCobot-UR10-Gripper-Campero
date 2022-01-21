#!/usr/bin/env python
import rospy
import math
import tf
import numpy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped

x=0.0
y=0.0
z=0.0
q_x=0.0
q_y=0.0
q_z=0.0
q_w=0.0
pose=PoseStamped()	

def tfListener():
	rospy.init_node('tf_listener',anonymous=True)
	pub = rospy.Publisher('campero_odometry', PoseStamped, queue_size=10)
	tf_listener=tf.TransformListener()
	tf_listener.waitForTransform('campero_map','campero_base_footprint',rospy.Time(), rospy.Duration(2.5))
	(trans,rot)=tf_listener.lookupTransform('campero_map','campero_base_footprint',rospy.Time(0))
	
	x=trans[0]
	y=trans[1]
	z=trans[2]
	q_x=rot[0]
	q_y=rot[1]
	q_z=rot[2]
	q_w=rot[3]
	angles=tf.transformations.euler_from_quaternion(quaternion=(q_x, q_y, q_z, q_w))
	alpha=numpy.rad2deg(angles[0])
	beta=numpy.rad2deg(angles[1])
	theta=numpy.rad2deg(angles[2])
	pose.pose.position.x=x
	pose.pose.position.y=y
	pose.pose.position.z=z
	pose.pose.orientation.x=q_x
	pose.pose.orientation.y=q_y
	pose.pose.orientation.z=q_z
	pose.pose.orientation.w=q_w
	rospy.loginfo('El robot se encuentra la posicion: x= %f , y= %f ,z= %f con la orientacion: alpha= %f , beta= %f , theta= %f', x,y,z,alpha,beta,theta)
	rospy.loginfo('x= %f ', x)
	rospy.loginfo('y= %f ', y)
	rospy.loginfo('z= %f ', z)
	rospy.loginfo('alpha= %f ', alpha)
	rospy.loginfo('beta= %f ', beta)
	rospy.loginfo('theta= %f ', theta)
	pub.publish(pose)
	


if __name__ == '__main__':
	try:
		while not rospy.is_shutdown():
			tfListener()
		rospy.loginfo('Programa cancelado')
	except rospy.ROSInterruptException:
		rospy.loginfo('Programa finalizado')
