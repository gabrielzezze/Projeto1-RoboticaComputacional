#!/usr/bin/env python
# -- coding:utf-8 --

# Imports #
import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import visao_module
import sources
import cormodule
from sensor_msgs.msg import LaserScan
from std_msgs.msg import UInt8

# Declaration of initial global variables #
status = None
bridge = CvBridge()
cv_image = None
media = []
centro = []
atraso = 1.5E9
resultados = []
area = 0.0
check_delay = False 
position = []
contador = 0
tracking_position = []
passa = True
media_cor = []
bumper = 0
bumped = False
frente = -1
distancias = []
v_scan = 0
w_scan = 0
scaned = False

# Function gets called every frame, checks for bumper activation
def scan_bumper(dado):
	global bumper
	bumper = dado.data

# Function gets called every frame, checks for laser signs
def scan_laser(dado):
    global frente
    global v_scan
    global w_scan
    global scaned
    leituras = np.array(dado.ranges).round(decimals=2)
    frente = leituras[0]
    v_scan, w_scan, scaned = sources.angulo_to_time(leituras)


# Function gets called every frame #
def roda_todo_frame(imagem):
    global media_cor
    global status
    global cv_image
    global media
    global centro
    global resultados
    global circles
    global contador
    global position
    global tracking_position
    global passa

    # Calculate Delay #
    now = rospy.get_rostime()
    imgtime = imagem.header.stamp
    lag = now-imgtime 
    delay = lag.nsecs
    if delay > atraso and check_delay==True:
		return 
    try:
        antes = time.clock()
        # Image Processing #
        cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")

        # Frame MetaData #
        centro, media, resultados, position =  visao_module.processa(cv_image)

        # Color Indentifier #
        media_cor = sources.Segue_cor(cv_image)

        # Check if Mobilnet results exists #
        if len(resultados) > 0:
            contador +=1
        elif contador <= 5: 
            contador = 0
            passa = True

        # Check if object is in frame for five frames #
        if contador > 5:
            # Init Tracking #
            img_tracking, status, tracking_position = sources.Track(cv_image,position,passa)
            passa = False

            # Check if tracking is sucessfull #
            if status == False:
                contador = 0
            
            # Delay calculation #
            depois = time.clock()

            # Display Final Frame #
            cv2.imshow("Camera", img_tracking)
            
        else:
            # Delay Calculation #
            depois = time.clock()
            cv2.imshow("Camera", cv_image)        
    except CvBridgeError as e:
        print('ex', e)



if __name__=="__main__":

    # Node Init and nescessary subscribers
    rospy.init_node("cor")
    topico_imagem = "/kamera"
    recebe_scan = rospy.Subscriber("/bumper", UInt8, scan_bumper)
    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    recebe_scan = rospy.Subscriber("/scan", LaserScan, scan_laser)

    try:
        while not rospy.is_shutdown():
            print(scaned)
            if scaned:
                vel = Twist(Vector3(v_scan,0,0), Vector3(0,0,w_scan))
                velocidade_saida.publish(vel)
                rospy.sleep(1)

            # Check if bumper is activated and react 
            if bumper == 1:
                v = -0.12
                w = -0.15
                bumped = True
            elif bumper == 2:
                v = -0.12
                w = 0.15
                bumped = True
            elif bumper == 3:
                v = 0.12
                w = -0.15
                bumped = True
            elif bumper == 4:
                v = 0.12
                w = 0.15
                bumped = True
            # If bumped go backwards and turn
            if bumped:
                vel = Twist(Vector3(v,0,0), Vector3(0,0,w))
                velocidade_saida.publish(vel)
                rospy.sleep(2)                

            bumper = 0
            bumped = False
            # If tracking is active, try to align
            if status:
                if tracking_position[0] < centro[0] + 20 and tracking_position[0] > centro[0] - 20:
                    w = 0
                elif tracking_position[0] > (centro[0] + 20):
                    w = -0.12
                else:
                    w = 0.12
                vel = Twist(Vector3(sources.controle_proporcional(frente),0,0), Vector3(0,0,w))
            # If designed color is found, escape from it
            elif sum(media_cor) != 0:
                if media_cor[0] < centro[0] + 20 and media_cor[0] > centro[0] - 20:
					w  = 0
                if media_cor[0] > centro[0]:
					w = 0.2
                else:
					w = -0.2
                vel = Twist(Vector3(sources.controle_proporcional_re(frente),0,0), Vector3(0,0,w))
            else:
                v=0
                w=0
                vel = Twist(Vector3(v,0,0), Vector3(0,0,w))
            velocidade_saida.publish(vel)
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
	    print("Ocorreu uma exceção com o rospy")