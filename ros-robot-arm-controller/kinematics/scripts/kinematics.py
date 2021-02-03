#!/usr/bin/env python
# license removed for brevity
import rospy
import sys
import numpy as np
from math import atan2, degrees, radians, cos, sin, acos, atan, pi, sqrt
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

import tf_conversions
import tf2_ros
import geometry_msgs.msg


## Constant values
# Distance from base to base joint
B_0 = 3.97
# Distance from base to first joint
B = 9.66 
# Length of the first strut
L1 = 13
# Length of second strut
L2 = 14.7
# End effector length
E = 6.71

# Variables to hold angles
angles = [0, 0, 0, 0]

# Debug
debug_vars = [ 0 for i in range(7) ]

# RVIZ
markers = MarkerArray()

# Subscriber: receives end effector position and publishes rviz markers
def callback_point(data):
    ## Helper parameters
    # Corrected points
    a = sqrt(data.x**2 + data.y**2) - E
    b = data.z - B
    # C -> distance between first joint and end effector joint
    C = sqrt(a**2 + b**2)
    # theta -> angle that C makes with y axis
    theta = atan2(b,a)

    # Alpha equation
    try:
        alpha = theta + acos((L1**2 + C**2 - L2**2)/(2*L1*C))
    except:
        rospy.loginfo("Alpha error")
        return

    # Beta equation
    try:
        beta = pi/2 - acos((L1**2 + L2**2 - C**2)/(2*L1*L2))
    except:
        rospy.loginfo("Beta error")
        return
    
    gamma = atan2(data.x, data.y)


    angles[0] = 130 - degrees(alpha) # L1
    angles[1] = 102 - degrees(beta + pi/2 - alpha) # L2
    angles[2] = 90 + degrees(gamma) # base

    debug_vars[0] = a
    debug_vars[1] = b
    debug_vars[2] = C
    debug_vars[3] = theta
    debug_vars[4] = alpha
    debug_vars[5] = beta
    debug_vars[6] = gamma

    # rospy.loginfo("\n(%s, %s, %s)-----------------\n C: %s\n Theta: %s\n Alpha: %s\n Beta: %s\n Gamma: %s", data.x, data.y, data.z, C, degrees(theta), degrees(alpha), degrees(beta), degrees(gamma))


    # RVIZ
    markers.markers = []

    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = "robot_base"
    t.transform.translation.x = 0
    t.transform.translation.y = 0
    t.transform.translation.z = 0.0
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)

    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "robot_base"
    t.child_frame_id = "robot_rotating_joint_base"
    t.transform.translation.x = 0
    t.transform.translation.y = 0
    t.transform.translation.z = B_0
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)


    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "robot_rotating_joint_base"
    t.child_frame_id = "robot_rotating_joint"
    t.transform.translation.x = 0
    t.transform.translation.y = 0
    t.transform.translation.z = 0
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, - gamma)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)

    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "robot_rotating_joint"
    t.child_frame_id = "robot_alpha_base"
    t.transform.translation.x = 0
    t.transform.translation.y = 0
    t.transform.translation.z = B - B_0
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)

    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "robot_alpha_base"
    t.child_frame_id = "robot_alpha"
    t.transform.translation.x = 0
    t.transform.translation.y = 0
    t.transform.translation.z = 0
    q = tf_conversions.transformations.quaternion_from_euler(pi/2 - alpha, 0, 0)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)

    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "robot_alpha"
    t.child_frame_id = "robot_beta_base"
    t.transform.translation.x = 0
    t.transform.translation.y = 0
    t.transform.translation.z = L1
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)

    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "robot_beta_base"
    t.child_frame_id = "robot_beta"
    t.transform.translation.x = 0
    t.transform.translation.y = 0
    t.transform.translation.z = 0
    q = tf_conversions.transformations.quaternion_from_euler(pi/2 + beta, 0, 0)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)

    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "robot_beta"
    t.child_frame_id = "robot_effector_holder"
    t.transform.translation.x = 0
    t.transform.translation.y = 0
    t.transform.translation.z = L2
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)

    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "robot_effector_holder"
    t.child_frame_id = "robot_effector_base"
    t.transform.translation.x = 0
    t.transform.translation.y = 0
    t.transform.translation.z = 0
    q = tf_conversions.transformations.quaternion_from_euler(pi + (pi/2 - beta + alpha), 0, 0)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)

    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "robot_effector_base"
    t.child_frame_id = "robot_effector"
    t.transform.translation.x = 0
    t.transform.translation.y = 0
    t.transform.translation.z = E
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)



    marker = Marker()
    marker.header.frame_id = "/robot_base"
    marker.type = marker.CYLINDER
    marker.action = marker.ADD
    marker.scale.x = 10
    marker.scale.y = 10
    marker.scale.z = B_0
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = 0
    marker.pose.position.y = 0 
    marker.pose.position.z = marker.scale.z / 2
    markers.markers.append(marker)

    marker = Marker()
    marker.header.frame_id = "/robot_rotating_joint"
    marker.type = marker.CYLINDER
    marker.action = marker.ADD
    marker.scale.x = 7
    marker.scale.y = 7
    marker.scale.z = B - B_0
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = 0
    marker.pose.position.y = 0 
    marker.pose.position.z = marker.scale.z / 2
    markers.markers.append(marker)

    marker = Marker()
    marker.header.frame_id = "/robot_alpha"
    marker.type = marker.CYLINDER
    marker.action = marker.ADD
    marker.scale.x = 2.3
    marker.scale.y = 2.3
    marker.scale.z = L1
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = 0
    marker.pose.position.y = 0 
    marker.pose.position.z = marker.scale.z / 2
    markers.markers.append(marker)

    marker = Marker()
    marker.header.frame_id = "/robot_beta"
    marker.type = marker.CYLINDER
    marker.action = marker.ADD
    marker.scale.x = 1.8
    marker.scale.y = 1.8
    marker.scale.z = L2
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = 0
    marker.pose.position.y = 0 
    marker.pose.position.z = marker.scale.z / 2
    markers.markers.append(marker)

    marker = Marker()
    marker.header.frame_id = "/robot_effector_base"
    marker.type = marker.CYLINDER
    marker.action = marker.ADD
    marker.scale.x = 1
    marker.scale.y = 1
    marker.scale.z = E
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = 0
    marker.pose.position.y = 0 
    marker.pose.position.z = marker.scale.z / 2
    markers.markers.append(marker)

    # Renumber the marker IDs
    id = 0
    for m in markers.markers:
        m.id = id
        id += 1

    # rospy.loginfo("\tAngles: \t[ %s, %s, %s, %s ]\n\n\t\t\t\tPoint: \t\t[ %s, %s, %s ]\n\n\t\t\t\tPoint_B: \t[ %s, %s, %s ]\n\n\n\n", angles[0], angles[1], angles[2], angles[3], Px, Py, Pz, data.x, data.y, data.z)


def callback_rotation(data):
    angles[3] = data.data
    rospy.loginfo("Angle: ", angles[3])



# Publisher: sends angles to robot
def inverse_kinematics():
    # Node initializations
    rospy.init_node('inverse_kinematics', anonymous=True)

    # Subscribers
    rospy.Subscriber('effector_position', Point,  callback_point)
    # rospy.Subscriber('effector_rotation', Float32, callback_rotation)

    # Publishers
    s1 = rospy.Publisher('s1', Float32, queue_size=10)
    s2 = rospy.Publisher('s2', Float32, queue_size=10)
    s3 = rospy.Publisher('s3', Float32, queue_size=10)
    s4 = rospy.Publisher('s4', Float32, queue_size=10)


    m = rospy.Publisher('markers', MarkerArray, queue_size=10)

    debug = [rospy.Publisher('debug_' + var, Float32, queue_size=10) for var in ['a', 'b', 'C', 'theta', 'alpha', 'beta', 'gamma']]

    rate = rospy.Rate(5) # 1hz
    while not rospy.is_shutdown():

        # Debugging
        # rospy.loginfo(angles[0])
        # rospy.loginfo(angles[1])
        # rospy.loginfo(angles[2])
        # rospy.loginfo(angles[3])

        # # Publish angles to the robot
        try:
            s1.publish(angles[0])
            s2.publish(angles[1])
            s3.publish(angles[2])
            # s4.publish(angles[3])
            m.publish(markers)

            for i in range(len(debug)):
                debug[i].publish(debug_vars[i])
        except:
            rospy.logerr('Error publishing')
            pass

        # Delay for publishing rate
        rate.sleep()


# Main Function

if __name__ == '__main__':
    # Initialize node: inverse_kinematics
    try:
        inverse_kinematics()
    except rospy.ROSInterruptException:
        pass
