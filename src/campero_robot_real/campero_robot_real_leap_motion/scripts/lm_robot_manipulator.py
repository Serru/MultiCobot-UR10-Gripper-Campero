#!/usr/bin/env python

import sys
import copy
import rospy

from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import geometry_msgs.msg
from sensor_msgs.msg import JointState

import tf.transformations as tf
from geometry_msgs.msg import Pose, Quaternion
from kinematics_utils import *
from math import *

from leap_motion.msg import leapcobotright

class CmdTrajectory():
    def __init__(self):
        self.send_trajectory_pub = rospy.Publisher('/pub_ik_trajectory', JointTrajectory, queue_size=10)
        self.send_gripper_cmd_pub = rospy.Publisher('/pub_gripper_control', JointTrajectory, queue_size=10)
        self.current_robot_pose = Pose()
        self.robot_pose_sub = rospy.Subscriber('/robot_pose', Pose, self.update_current_pose)
        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.update_current_joint_states)
        self.robot_pose_updated = False
        
        ## LEAP MOTION CONFIG
        self.leap_motion_right_hand_sub = rospy.Subscriber('/leapmotion/data', leapcobotright, self.send_leap_motion_trajectory, queue_size=10)

        self.set_leap_motion_reference_position = False
        self.leap_motion_reference_position = geometry_msgs.msg.Pose().position
        self.robot_reference = geometry_msgs.msg.Pose()
        self.current_joint_states = []
        self.start_leap_motion_control = False

    def send_leap_motion_trajectory(self, frame):
        # Gestos:
        # Punyo: Para de enviar nuevas instrucciones
        # Ok: Comienza a enviar instrucciones
        # Pinza: Cierra o abre el gripper
        # Movimiento de munyeca: Rota el Gripper [para la simulacion no se activan]
        # Gesto de ROCK: Configura el frame de referencia de Leap Motion
        # Antes de comenzar, es bueno situar el robot en una posicion en donde trabajara

        print("========================== COMIENZO ========================")
        print("== Leap Motion: hand position")
        print(frame.right_hand_palmpos)
        print("== Leap Motion: hand reference positions")
        print(self.leap_motion_reference_position)

        # Obtiene la posicion de la palma de la mano
        palm_pos = frame.right_hand_palmpos

        if frame.right_hand_fist:
            print("== Leap Motion -- Fist Gesture: Stop")
            self.start_leap_motion_control = False

        if frame.right_hand_thumb_up:
            print("== Leap Motion -- Thumb Up Gesture: Start")
            self.start_leap_motion_control = True

        if self.start_leap_motion_control:
            # Configura la posicion de referencia en Leap Motion,
            # cada vez que reconozca el gesto de ROCK&ROLL
            if frame.right_hand_set_origin_frame_detected:
                print("== Leap Motion -- Rock Gesture: Set hand reference positions")
                self.set_leap_motion_reference_position = True
                self.leap_motion_reference_position.x = palm_pos.x
                self.leap_motion_reference_position.y = palm_pos.y
                self.leap_motion_reference_position.z = palm_pos.z
                print("== Leap Motion: Reference Values (xyz): "+ str(self.leap_motion_reference_position.x) + ", " + str(self.leap_motion_reference_position.y) + ", " + str(self.leap_motion_reference_position.z))
                rospy.sleep(0.5)

            # Solamente si la referencia de lm esta configurada
            if self.set_leap_motion_reference_position:
    
                if frame.right_hand_pinch:
                    print("==== Leap Motion -- Gripper Gesture: Closing the gripper")
                    #self.send_gripper_cmd(frame.right_hand_pinch_value)
                    if frame.right_hand_pinch_value > 0.2:
                        self.send_gripper_cmd(0.8)
                    else:
                        self.send_gripper_cmd(0.0)
                    #self.send_gripper_cmd(0.43)
                #else:
                    #self.send_gripper_cmd(0.0)
    
                ## Se obvia las rotaciones de momento
                #r = frame.right_hand_rotate_value
                #p = frame.right_hand_turn_value
                #y = frame.right_hand_swing_value
                #rpy = tf.quaternion_from_euler(r, p, y)
            
                #print str(self.robot_reference.position.x)
                #print str(palm_pos.x*0.001)
                desired_pos = self.get_transformed_position(palm_pos)
                pos_x = self.robot_reference.position.x + desired_pos.x * 0.001
                pos_y = self.robot_reference.position.y + desired_pos.z * 0.001
                pos_z = self.robot_reference.position.z + desired_pos.y * 0.001
                if pos_x != 0.0 or pos_y != 0.0 or pos_z != 0.0:
                    print("==== UR10 -- tf: UR10 current pose positions (xyz): "+ str(self.current_robot_pose.position.x) + ", " + str(self.current_robot_pose.position.y) + ", " + str(self.current_robot_pose.position.z))
                    print("==== UR10: reference positions (xyz): "+ str(self.robot_reference.position.x) + ", " + str(self.robot_reference.position.y) + ", " + str(self.robot_reference.position.z))
                    print("==== Leap motion: positions taking reference as origin (xyz): "+ str(desired_pos.x) + ", " + str(desired_pos.y) + ", " + str(desired_pos.z))
                    print("==== Leap motion: positions taking reference as origin (xyz) * 0.001: "+ str(desired_pos.x*0.001) + ", " + str(desired_pos.y*0.001) + ", " + str(desired_pos.z*0.001))
                    print("==== UR10 -- arm: desire cartesian [end effector] position (xyz): "+ str(pos_x) + ", " + str(pos_y) + ", " + str(pos_z))            
                    rpy = tf.quaternion_from_euler(pi, 0, 0)
                    #print rpy
                    #self.send_trajectory(palm_pos.x, palm_pos.y, palm_pos.z, rpy[0], rpy[1], rpy[2], rpy[3])
                    #self.send_trajectory(round(-1*pos_x, 3), round(-1*pos_y, 3), round(pos_z, 3), ()-0.499609514494, -0.500773235822, 0.499207219805, -0.500408484147)
                    self.send_trajectory(round(-1*pos_x, 3), round(-1*pos_y, 3), round(pos_z, 3), rpy[0], rpy[1], rpy[2], rpy[3])
                
        print("========================== FIN ========================\n")

    def send_trajectory(self, pos_x, pos_y, pos_z, rot_x, rot_y, rot_z, rot_w):
        position = JointTrajectory()
        position.header.stamp=rospy.Time.now()
        position.header.frame_id = "/campero_ur10_base_link"    
        position.joint_names = ['campero_ur10_shoulder_pan_joint','campero_ur10_shoulder_lift_joint','campero_ur10_elbow_joint',
                          'campero_ur10_wrist_1_joint','campero_ur10_wrist_2_joint','campero_ur10_wrist_3_joint']
        
        rate = rospy.Rate(10)
        while not self.robot_pose_updated:
            rate.sleep()

#        (roll, pitch, yaw) = tf.euler_from_quaternion([
#                                            self.current_robot_pose.orientation.x,
#                                            self.current_robot_pose.orientation.y,
#                                            self.current_robot_pose.orientation.z,
#                                            self.current_robot_pose.orientation.w])
#
#        rcs = [ self.current_robot_pose.position.x,
#                self.current_robot_pose.position.y,
#                self.current_robot_pose.position.z,
#                roll, pitch, yaw]
        rcs = self.current_joint_states
        print("\033[91m====== UR10 -- RCS: list of reference pose [joint values]: ["+ str(rcs[0]) + ", " + str(rcs[1]) + ", " + str(rcs[2]) + ", " + str(rcs[3]) + ", " + str(rcs[4]) + ", " + str(rcs[5]) + "]\033[0m")

        ps = Pose()
        ps.position.x = pos_x
        ps.position.y = pos_y
        ps.position.z = pos_z
        ps.orientation.x = rot_x
        ps.orientation.y = rot_y
        ps.orientation.z = rot_z
        ps.orientation.w = rot_w
    
        #state = []
    
        #sol = inv_kin(ps, array_pos)0.0530511263012886  0.052499477863311765 0.051719752252101896
        #print(sol)

        
        points = JointTrajectoryPoint()
        try:            
            ik_values = inv_kin(ps, rcs)
            print("\033[91m====== UR10 -- IK: list of the new pose [joint values]: ["+ str(ik_values[0]) + ", " + str(ik_values[1]) + ", " + str(ik_values[2]) + ", " + str(ik_values[3]) + ", " + str(ik_values[4]) + ", " + str(ik_values[5]) + "]\033[0m")
            points.positions = [ik_values[0],ik_values[1],ik_values[2],ik_values[3],ik_values[4], ik_values[5]]

            distancia = max([abs(ik_values[0]-rcs[0]), abs(ik_values[1] - rcs[1]), abs(ik_values[2]-rcs[2]), abs(ik_values[3] - rcs[3]), abs(ik_values[4] - rcs[4]), abs(ik_values[5] - rcs[5])])
            velocidad = 0.05
            duration = distancia / velocidad

            print('\033[93m====== Trayectory Values ======\033[0m')
            
            print('\033[93m=== duracion: ' + str(duration) + ']\033[0m')
            print('\033[93m=== distancia_max: ' + str(distancia) + ']\033[0m')
            print('\033[93m=== velocidad: ' + str(velocidad) + ']\033[0m')
            print('\033[92m===== Diferencia de joints values\033[0m')
            print('\033[92m=== shoulder_pan_joint: ' + str(rcs[0] - ik_values[0]) + ']\033[0m')
            print('\033[92m=== shoulder_lift_joint: ' + str(rcs[1] - ik_values[1]) + ']\033[0m')
            print('\033[92m=== elbow_joint: ' + str(rcs[2] - ik_values[2]) + ']\033[0m')
            print('\033[92m=== wrist_1_joint: ' + str(rcs[3] - ik_values[3]) + ']\033[0m')
            print('\033[92m=== wrist_2_joint: ' + str(rcs[4] - ik_values[4]) + ']\033[0m')
            print('\033[92m=== wrist_3_joint: ' + str(rcs[5] - ik_values[5]) + ']\033[0m')
            print('\033[91m===== Joints values deseados\033[0m')
            print('\033[91m=== shoulder_pan_joint: ' + str(points.positions[0]) + ']\033[0m')
            print('\033[91m=== shoulder_lift_joint: ' + str(points.positions[1]) + ']\033[0m')
            print('\033[91m=== elbow_joint: ' + str(points.positions[2]) + ']\033[0m')
            print('\033[91m=== wrist_1_joint: ' + str(points.positions[3]) + ']\033[0m')
            print('\033[91m=== wrist_2_joint: ' + str(points.positions[4]) + ']\033[0m')
            print('\033[91m=== wrist_3_joint: ' + str(points.positions[5]) + ']\033[0m')

            #points.time_from_start = rospy.Duration.from_sec(1)
            points.time_from_start = rospy.Duration.from_sec(duration)
            position.points.append(points)
            self.send_trajectory_pub.publish(position)
            #state = sol
            #rospy.sleep(10)
            self.robot_pose_updated = False
            print('\033[93m[ Enviada Trayectoria ]\033[0m')
            print('\033[93m[' + str(ps.position.x) + ', ' + str(ps.position.y) + ', ' + str(ps.position.z) + ']\033[0m')

        except Exception:
            print('\033[91m[ Singularidad, valores:' + str(ps.position.x) + ', ' + str(ps.position.y) + ', ' + str(ps.position.z) + ']\033[0m')

    def get_transformed_position(self, palm_pos):
        
        desired_pos = geometry_msgs.msg.Pose().position
        pos_x = abs(self.leap_motion_reference_position.x - palm_pos.x)
        pos_y = abs(self.leap_motion_reference_position.y - palm_pos.y)
        pos_z = abs(self.leap_motion_reference_position.z - palm_pos.z)

        if palm_pos.x > self.leap_motion_reference_position.x:
            desired_pos.x = -pos_x
        if palm_pos.x < self.leap_motion_reference_position.x:
            desired_pos.x = pos_x
        if palm_pos.x == self.leap_motion_reference_position.x:
            desired_pos.x = 0

        if palm_pos.y > self.leap_motion_reference_position.y:
            desired_pos.y = pos_y
        if palm_pos.y < self.leap_motion_reference_position.y:
            desired_pos.y = -pos_y
        if palm_pos.y == self.leap_motion_reference_position.y:
            desired_pos.y = 0

        if palm_pos.z > self.leap_motion_reference_position.z:
            desired_pos.z = pos_z
        if palm_pos.z < self.leap_motion_reference_position.z:
            desired_pos.z = -pos_z
        if palm_pos.z == self.leap_motion_reference_position.z:
            desired_pos.z = 0

        return desired_pos

    def send_gripper_cmd(self, gripper_distance):
        gripper = JointTrajectory()
        gripper.header.stamp=rospy.Time.now()
        gripper.header.frame_id = "/campero_ur10_ee_link"    
        gripper.joint_names = ['campero_robotiq_85_left_knuckle_joint']
        
        points = JointTrajectoryPoint()
        points.positions = [gripper_distance]
        points.time_from_start = rospy.Duration.from_sec(0.4)
        gripper.points.append(points)
        #print gripper
        self.send_gripper_cmd_pub.publish(gripper)
        print('\033[93m[' + str(gripper_distance) + ']\033[0m')

    def set_robot_reference(self):
        ## Puede que se anyada las orientaciones... primero posiciones
        self.robot_reference.position.x = self.current_robot_pose.position.x
        self.robot_reference.position.y = self.current_robot_pose.position.y
        self.robot_reference.position.z = self.current_robot_pose.position.z
        self.robot_reference.orientation.x = self.current_robot_pose.orientation.x
        self.robot_reference.orientation.y = self.current_robot_pose.orientation.y
        self.robot_reference.orientation.z = self.current_robot_pose.orientation.z
        self.robot_reference.orientation.w = self.current_robot_pose.orientation.w


    def update_current_pose(self, pose):
        #print("========== update current_robot_pose ============")
        #print("= Pose que viene del tf:")
        #print(pose)
        #self.current_robot_pose.position.x = (-1 * pose.position.x)
        #self.current_robot_pose.position.y = (-1 * pose.position.y)

        self.current_robot_pose.position.x = pose.position.x
        self.current_robot_pose.position.y = pose.position.y
        self.current_robot_pose.position.z = pose.position.z
        self.current_robot_pose.orientation.x = pose.orientation.x
        self.current_robot_pose.orientation.y = pose.orientation.y
        self.current_robot_pose.orientation.z = pose.orientation.z
        self.current_robot_pose.orientation.w = pose.orientation.w
        self.robot_pose_updated = True
        #print("= current_robot_pose:")
        #print(self.current_robot_pose)
        #print("========== fin de update current_robot_pose ============")


    def update_current_joint_states(self, joint_state_msg):
        #print("========== update current_joint_states ============")
        #print("= Mensaje publicado:")
        #print(joint_state_msg)
        
        # Para el campero
        shoulder_pan_joint = joint_state_msg.position[2]
        shoulder_lift_joint  = joint_state_msg.position[1]
        elbow_joint  = joint_state_msg.position[0]
        wrist_1_joint  = joint_state_msg.position[3]
        wrist_2_joint  = joint_state_msg.position[4]
        wrist_3_joint  = joint_state_msg.position[5]

        # Para Gazebo
        #[0, 1, 2, 3, 4, 5, 6, campero_ur10_elbow_joint, campero_ur10_shoulder_lift_joint,
        #campero_ur10_shoulder_pan_joint, campero_ur10_wrist_1_joint, campero_ur10_wrist_2_joint,
        #campero_ur10_wrist_3_joint]
        #shoulder_pan_joint = joint_state_msg.position[9]
        #shoulder_lift_joint  = joint_state_msg.position[8]
        #elbow_joint  = joint_state_msg.position[7]
        #wrist_1_joint  = joint_state_msg.position[10]
        #wrist_2_joint  = joint_state_msg.position[11]
        #wrist_3_joint  = joint_state_msg.position[12]


        self.current_joint_states = [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint]
        #print("== Current joint states to compare to ==")
        #print(self.current_joint_states)
        rospy.sleep(0.01)
        #print(self.current_joint_states)
        #print("========== fin de update current_robot_pose ============")

    # Set init pose with articular values
    def set_init_pose(self, pos_x, pos_y, pos_z, rot_x, rot_y, rot_z):
        position = JointTrajectory()
        position.header.stamp=rospy.Time.now()
        position.header.frame_id = "/campero_ur10_base_link"    
        position.joint_names = ['campero_ur10_shoulder_pan_joint','campero_ur10_shoulder_lift_joint','campero_ur10_elbow_joint',
                          'campero_ur10_wrist_1_joint','campero_ur10_wrist_2_joint','campero_ur10_wrist_3_joint']
        
        rcs = [pos_x, pos_y, pos_z, rot_x, rot_y, rot_z]
        
        points = JointTrajectoryPoint()
        points.positions = rcs
        points.time_from_start = rospy.Duration.from_sec(5)
        position.points.append(points)
        self.send_trajectory_pub.publish(position)


        
        
if __name__ == '__main__':
    rospy.init_node('campero_robot_real_manipulator', anonymous=True)
    cmd = CmdTrajectory()
    #rpy = tf.quaternion_from_euler(-3.12, 0.0, 1.62)
    #print rpy
    #[-0.68945825 -0.72424496  0.00781949  0.00744391]
    #cmd.send_trajectory(-0.6, -0.16, 0.62, rpy[0], rpy[1], rpy[2], rpy[3])
    
    # Posicion inicial del brazo
    #cmd.set_init_pose( -0.59597620794, 0.0535301135833, 0.753952353141, 3.140859, 0.000194, 0.000816)
    #cmd.set_init_pose(2.176, -1.518, -1.671, -1.511, 1.589, -1.014)
    cmd.send_gripper_cmd(0.0)
    print(cmd.robot_reference)
    rospy.sleep(5)
    #cmd.send_trajectory(-0.24, 0.632, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
    #rospy.sleep(4)
    #cmd.send_trajectory(0.24, 0.632, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
    #rospy.sleep(4)
    #cmd.send_trajectory(0.24, 0.8, 0.15, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
    #rospy.sleep(4)

    cmd.set_robot_reference()
    print(cmd.robot_reference)

    #cmd.pick_place()

    # Ejemplo de movmiento no deseado
    #cmd.send_trajectory(-0.30, 0.300, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
    #rospy.sleep(4)
    #cmd.send_trajectory(-0.40, 0.0, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
    #rospy.sleep(4)
    #cmd.send_trajectory(-0.80, -0.3, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
    #rospy.sleep(4)
    
    #cmd.pick_place()
    while not rospy.is_shutdown():
        rospy.sleep(0.1)
