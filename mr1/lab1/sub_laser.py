#!/usr/bin/env python

# run: python subscriber_scan.py 5  (where 5 is robot number)
import sys
import rospy
import time
import signal

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import tf

import matplotlib.pyplot as plt
import numpy as np

scan = []

twist = 0

def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x, y)

def set_vel(v, w):
    global twist
    global pub
    twist = Twist()
    twist.linear.x = v
    twist.angular.z = w
    print(twist)
    pub.publish(twist)


def start_vel_publisher():
    global pub
    pub = rospy.Publisher("/PIONIER" + nr + "/RosAria/cmd_vel", Twist, queue_size=3)
    rospy.init_node('vel_controller', anonymous=True)


def callback_scan(msg):
    global scan
    # extract ranges from message
    scan = list(msg.ranges)
    for i in range(-1,len(scan),64):
    	print("Angle: %f" % (180/len(scan)*i - 90))
    	print("Scan x: %f Scan y: %f" % (pol2cart(scan[i],180/len(scan)*i - 90)))


def listener():
    # Subscribe topics and bind with callback functions
    rospy.Subscriber("/PIONIER" + nr + "/scan", LaserScan, callback_scan)


def exit_int():
    sys.exit(0)



signal.signal(signal.SIGINT, exit_int)


nr = sys.argv[1]

start_vel_publisher()
listener()

# time.sleep(2)
# set_vel(0.1,0)

while True:
	time.sleep(1)
