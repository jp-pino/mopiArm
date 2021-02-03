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


# Publisher: sends angles to robot
def inverse_kinematics():
    # Node initializations
    rospy.init_node('sequence', anonymous=True)

    # Publishers
    p = rospy.Publisher('effector_position', Point, queue_size=10)


    x = -10
    y = 15
    z = 10
    dir = True

    rate = rospy.Rate(20) # 1hz
    while not rospy.is_shutdown():
        # # Publish angles to the robot
        try:
            if (dir):
                if (x < 10):
                    x += .1 
                else:
                    dir = False
            else:
                if (x > -10):
                    x -= .1 
                else:
                    dir = True

            p.publish(Point(x, y, z))

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
