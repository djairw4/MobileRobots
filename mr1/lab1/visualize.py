import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import rospy
import sys
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import LaserScan
import tf
from time import sleep

nr = "1"

loop_count=25  # drawing loop iterations
beam_half_angle=7.5 # haf of sonar angular beam width
laser_half_angle=3.5

# 
# A function to calculate Cartesian coordinates to polar
#  result: a tuple (rho,phi)
#  rho - radius, phi - angle in degrees
def cart2pol(x, y):
    rho = np.sqrt(x**2 + y**2)
    phi = math.degrees(np.arctan2(y, x))+90
    return(rho, phi)

# A function to transform polar  coordinates to Cartesian
# input angle in degrees
# returns a tuple (x,y)
def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x, y)

def callback_scan_sonar(msg):
    global sonar_scan
    ## extract ranges from message
    sonar_scan=list(msg.points)

def callback_scan_laser(msg):
    global laser_scan
    # extract ranges from message
    laser_scan = list(msg.ranges)
    #for i in range(-1,len(laser_scan),64):
   # 	print("Angle: %f" % (180/len(laser_scan)*i - 90))
   # 	print("Scan x: %f Scan y: %f" % (pol2cart(laser_scan[i],180/len(laser_scan)*i - 90)))



# plotting sonars
def plotsonars(ax,sonarreads):
    pol=[cart2pol(x.x,x.y) for x in sonarreads ]
    for item in pol:
        print (item[0],item[1])
        wedge = mpatches.Wedge([0,0], item[0], item[1]-beam_half_angle, item[1]+beam_half_angle, alpha=0.4, ec="black",fc="CornflowerBlue")
        ax.add_patch(wedge)

# plotting laser
def plotlasers(ax,laserreads):
    item = [0,0]
    for i in range(-1,len(laserreads),32):
    	item[0] = laser_scan[i]
    	item[1] = 180/len(laser_scan)*i
    	print(item[0],item[1])
    	wedge = mpatches.Wedge([0,0], item[0], item[1]-laser_half_angle, item[1]+laser_half_angle, alpha=0.4, ec="black",fc="Red")
    	ax.add_patch(wedge)

def plotarrows(ax,arrlist):
    y=[[0,0]+x for x in arrlist ]
    soa =np.array(y) 
    X,Y,U,V = zip(*soa)
    ax.quiver(X,Y,U,V,angles='xy',scale_units='xy',scale=1)

global sonar_scan
global laser_scan
rospy.init_node('listener', anonymous=True)
rospy.Subscriber("/PIONIER"+nr+"/RosAria/sonar", PointCloud, callback_scan_sonar)
rospy.Subscriber("/PIONIER" + nr + "/scan", LaserScan, callback_scan_laser)
sleep(0.5)
plt.figure()
ax = plt.gca()
ax.set_aspect('equal')
ax.set_xlim([-6,6])
ax.set_ylim([-6,6])
plt.ion()
plt.show()

for i in range(loop_count):
    ax.cla()
    plotsonars(ax,sonar_scan[0:7])
    plotlasers(ax,laser_scan) 
    ax.set_xlim([-6,6])
    ax.set_ylim([-6,6])
    plt.draw()
    plt.pause(0.0001)
    sleep(0.2)
