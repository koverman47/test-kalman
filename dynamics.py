#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32
from time import time

def dynamics():
    rospy.init_node('dynamics_node')
    pub = rospy.Publisher('dynamics', Float32, queue_size=1)

    position = 0.
    velocity = Float32()
    
    t1 = time()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        t2 = time()
        r = np.random.normal(0.5, 0.1)

        dt = t2 - t1
        t1 = t2
        velocity.data = r

        pub.publish(velocity)
        position += dt * r

        rospy.loginfo("Position : %f" % position)
        rospy.loginfo("Velocity : %f" % velocity.data)

        rate.sleep()





if __name__ == "__main__":
    dynamics()
