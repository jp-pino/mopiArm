#!/usr/bin/env python
# license removed for brevity
import rospy
import sys
import tf
import numpy as np
from math import atan2, degrees, cos, sin, acos, atan, pi
from std_msgs.msg import UInt16
from geometry_msgs.msg import Point, PoseStamped

# Constant values
L2 = 14.0
L3 = 15.2
D1 = 10.5

A4 = 2.257
D5 = 3.111



# Variables to hold angles
angles = [0, 0, 0, 0]
alpha = 0
beta = 0



# TRANSFORMATION MATRICES
# X_BC = 10.4
# Y_BC = 10.75
X_BC = 15
Y_BC = 15
Z_BC = 70
T_BC = [[1, 0, 0, X_BC], [0, cos(pi), -sin(pi), Y_BC], [0, sin(pi), cos(pi), Z_BC], [0, 0, 0, 1]]


# Subscriber: receives end effector position and rotation angle

def callback_point(data):
    # Transform point to base reference frame
    point = np.matmul(T_BC, [[data.pose.position.x * 100], [data.pose.position.y * 100], [data.pose.position.z * 100], [1]])

    # Calculate theta1
    theta1 = atan2(point[1][0], point[0][0])
    angles[0] = degrees(theta1)

    # Calculate Px, Py, Pz
    Pz = point[2][0] + D5 - D1
    Px = point[0][0] - A4 * cos(theta1)
    Py = point[1][0] - A4 * sin(theta1)

    # Caltulate theta2
    cos_alpha = (Px**2 + Py**2 + Pz**2 + L2**2 - L3**2)/(2 * L2 * ((Px**2 + Py**2 + Pz**2)**0.5))
    if (cos_alpha >= -1 and cos_alpha <= 1):
        alpha = degrees(acos(cos_alpha))
        gamma = degrees(atan(Pz / ((Px**2 + Py**2)**0.5)))
        angles[1] = alpha + gamma
    else:
        rospy.logerr("Alpha Error: %s", cos_alpha)


    # Calculate theta3
    cos_beta = (L2**2 + L3**2 - Px**2 - Py**2 - Pz**2 )/(2 * L2 * L3)
    if (cos_beta >= -1 and cos_beta <= 1):
        beta = degrees(acos(cos_beta))
        angles[2] = -(beta - 180) - angles[1]
    else:
        rospy.logerr("Beta Error: %s", cos_beta)


    # Calculate end effector rotation
    a = tf.transformations.euler_from_quaternion([data.pose.orientation.w, data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z])
    angles[3] = abs(degrees(a[0]))

    rospy.loginfo("\tAngles: \t[ %s, %s, %s, %s ]\n\n\t\t\t\tPoint: \t\t[ %s, %s, %s ]\n\n\t\t\t\tPoint_B: \t[ %s, %s, %s ]\n\n\t\t\t\tEuler: \t\t[ %s, %s, %s ]\n\n\n\n\n", angles[0], angles[1], angles[2], angles[3], Px, Py, Pz, point[0][0], point[1][0], point[2][0], degrees(a[0]), degrees(a[1]), degrees(a[2]))



# Publisher: sends angles to robot

def inverse_kinematics():
    # Node initialization
    rospy.init_node('inverse_kinematics', anonymous=True)

    # Subscribers
    rospy.Subscriber('/visp_auto_tracker/object_position', PoseStamped, callback_point)

    # Publishers
    s1 = rospy.Publisher('s1', UInt16, queue_size=10)
    s2 = rospy.Publisher('s2', UInt16, queue_size=10)
    s3 = rospy.Publisher('s3', UInt16, queue_size=10)
    s4 = rospy.Publisher('s4', UInt16, queue_size=10)


    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():

        # Debugging
        # rospy.loginfo(angles[0])
        # rospy.loginfo(angles[1])
        # rospy.loginfo(angles[2])
        # rospy.loginfo(angles[3])

        # Publish angles to the robot
        try:
            x = angles[0]
            x = -0.000000046857728 * x**4 + 0.000023052914391 * x**3 - 0.003933117349334 * x**2 + 1.087457595203660 * x + 5.338880798732130
            s1.publish(x)
            s2.publish(0.836968 * angles[1] + 26.118)
            s3.publish(1.043811 * angles[2] + 25.720)
            s4.publish(angles[3])
        except:
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
