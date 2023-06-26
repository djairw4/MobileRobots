#!/usr/bin/env python

# run: python subscriber_scan.py 5  (where 5 is robot number)
import sys
import rospy
from sensor_msgs.msg import PointCloud
import tf

def callback_scan(msg):
    ## extract ranges from message
    scan=list(msg.points)
    for i in range(8):
	    print (" Sensor: %f Scan x: %f  y: %f" % (i, scan[i].x,scan[i].y))


    
def listener():
    rospy.init_node('listener', anonymous=True)

    # Subscribe topics and bind with callback functions
    rospy.Subscriber("/PIONIER"+nr+"/RosAria/sonar", PointCloud, callback_scan)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        sys.stderr.write('Usage: sys.argv[0] \n')
        sys.exit(1)

    nr=sys.argv[1]
    listener()  
