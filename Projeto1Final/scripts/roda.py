#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from geometry_msgs.msg import Twist, Vector3
from math import pi

v = 0.15  # Velocidade linear (m/s)
w = 0.1  # Velocidade angular (rad/s)

t = float(pi/0.2) +0.075

if __name__ == "__main__":
    rospy.init_node("roda_exemplo")
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=3)

    try:
        while not rospy.is_shutdown():

            vel = Twist(Vector3(0,0,0), Vector3(0,0,w))
            pub.publish(vel)
            rospy.sleep(t)
            vel = Twist(Vector3(v,0,0), Vector3(0,0,0))
            pub.publish(vel)
            rospy.sleep(7.0)

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")