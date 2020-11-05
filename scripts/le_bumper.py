#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import UInt8

bumper = 0
passa = True

def scaneou(dado):
	global bumper
	bumper = dado.data
	print(dado.data)


if __name__=="__main__":

	rospy.init_node("le_scan")

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
	recebe_scan = rospy.Subscriber("/bumper", UInt8, scaneou)

	while not rospy.is_shutdown():
		if bumper == 1:
			v= -0.10
			w = -0.15
		elif bumper == 2:
			v= -0.10
			w = 0.15
		elif bumper == 3:
			v= 0.10
			w = -0.15
		elif bumper == 4:
			v= 0.10
			w = 0.15
		if passa:
			passa = False
			v=0
			w=0
		else:
			v = 0.17
			w = 0
		velocidade = Twist(Vector3(v, 0, 0), Vector3(0, 0,w))
		velocidade_saida.publish(velocidade)
rospy.sleep(0.5)
