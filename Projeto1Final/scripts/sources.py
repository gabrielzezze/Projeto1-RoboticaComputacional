# import the necessary packages
from imutils.video import VideoStream
from imutils.video import FPS
import argparse
import imutils
import time
import cv2
from math import *

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", type=str,
	help="path to input video file")
ap.add_argument("-t", "--tracker", type=str, default="kcf",
	help="OpenCV object tracker type")
args = vars(ap.parse_args())

(major, minor) = cv2.__version__.split(".")[:2]
if int(major) == 3 and int(minor) < 3:
	tracker = cv2.Tracker_create(args["tracker"].upper())

else:
	OPENCV_OBJECT_TRACKERS = {
		"csrt": cv2.TrackerCSRT_create,
		"kcf": cv2.TrackerKCF_create,
		"boosting": cv2.TrackerBoosting_create,
		"mil": cv2.TrackerMIL_create,
		"tld": cv2.TrackerTLD_create,
		"medianflow": cv2.TrackerMedianFlow_create,
		"mosse": cv2.TrackerMOSSE_create
	}

	tracker = OPENCV_OBJECT_TRACKERS["kcf"]()

initBB = None
def Track(frame,positions,passa):
    if passa and len(positions) > 0:
        initBB = (positions[0],positions[1],positions[2],positions[3])
        tracker.init(frame, initBB)
    # frame = imutils.resize(frame,width=400)
    (H, W) = frame.shape[:2]
    try:
        (succes,box) = tracker.update(frame)
        if succes:
            (x, y, w, h) = [int(v) for v in box]
            cv2.rectangle(frame,(x,y),(x+w, y+h),(255,0,0),2)
            return frame, True, [x+w/2,y+h/2]
        else: 
            return frame, False, []
    except:
        return frame, False,[]

#-----------------------------------------------------------------------------------------------------------#
# Color Follow #
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cormodule
import rospy
import numpy as np
import tf
import math
import time
# Seguidor de Cor
bridge = CvBridge()
cv_image = None
media_cor = []
centro = []
atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos
area = 0.0 # Variavel com a area do maior contorno
#-----------------------------------------------------------------------------------------#
# Indentifica areas onde possuem mais indicios da cor escolhida (Azul)
def Segue_cor(imagem):
	# now = rospy.get_rostime()
	# imgtime = imagem.header.stamp
	# lag = now-imgtime # calcula o lag
	# delay = lag.nsecs
	# print("delay ", "{:.3f}".format(delay/1.0E9))
	# if delay > atraso and check_delay==True:
	# 	print("Descartando por causa do delay do frame:", delay)
	# 	return 
	antes = time.clock()
	# cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
	media_cor, centro, area =  cormodule.identifica_cor(imagem)
	depois = time.clock()
	return media_cor

#--------------------------------------------------------------------------------------------#
# Funcao que retorna velocidade em funcao da distancia que o objeto se apresenta
def controle_proporcional(dist):
	if dist < 0.1:
		return 0
	else:
		return 0.22/1.4*(dist-0.1)

def controle_proporcional_re(dist):
	if dist > 2:
		return 0
	else:
		return 0.219/2*(dist-2)

#-----------------------------------------------------------------------------------------------#
def angulo_to_time(lista):
	lista[lista == 0] = 99
	print(min(lista))
	if min(lista) > 0.23:
		return 0, 0, False
	else:
		for i in range(len(lista)):
			if lista[i] == min(lista):
				if i > 20 and i < 340:
					if i < 90:
						return -0.19, 0.19, True
					elif i < 180 and i > 90:
						return 0.19, -0.19, True
					elif i < 270 and i > 180:
						return 0.19, 0.19, True
					else:
						return -0.19, -0.19, True
				else:
					return 0, 0, False