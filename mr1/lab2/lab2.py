import csv
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import math


class data:
    def __init__(self):
        self.time = []
        self.posL = []
        self.posR = []
        self.velL = []
        self.velR = []


class pose:
    def __init__(self):
        self.x = []
        self.y = []
        self.z = []
        self.w = []


gear_ratio = 38.3
wheel_d = 195.3
axle_length = 327
encoder_max = 65536
ticks_per_rev = 500

def get_csv(filename):
    new_data = data()
    with open(filename) as csvfile:
        reader = csv.DictReader(csvfile, delimiter=';', quotechar='|')
        for row in reader:
            temp = (row['#time'].split(':')[-1], row['posL'], row['posR'], row['velL'], row ['velR'])
            new_data.time.append(float(temp[0]))
            new_data.posL.append(float(temp[1]))
            new_data.posR.append(float(temp[2]))
            new_data.velL.append(float(temp[3]))
            new_data.velR.append(float(temp[4]))
        return new_data


def pol2cart(dist, angle):
    x = dist * np.cos(angle)
    y = dist * np.sin(angle)
    return(x, y)


def calc_dist_enc(data):
    pos_now = 0
    pos_prev = 0
    dist_L = 0
    dist_R = 0
    for i in range(len(data.time)):
        pos_now_L = data.posL[i]
        if (i >= 1):
            pos_prev_L = data.posL[i-1]
        else:
            pos_prev_L = pos_now_L
        delta_encoder_L = (pos_now_L - pos_prev_L + encoder_max/2) % encoder_max - encoder_max/2
        # (4 x ticks per rev x gear-ratio) / (wheel_diameter x Π)
        magic = (4 * ticks_per_rev * gear_ratio) / (np.pi * wheel_d)
        travel_L = (delta_encoder_L) / magic
        dist_L += travel_L

        pos_now_R = data.posR[i]
        if (i >= 1):
            pos_prev_R = data.posR[i-1]
        else:
            pos_prev_R = pos_now_R
        delta_encoder_R = (pos_now_R - pos_prev_R + encoder_max/2) % encoder_max - encoder_max/2
        # (4 x ticks per rev x gear-ratio) / (wheel_diameter x Π)
        travel_R = (delta_encoder_R) / magic
        dist_R += travel_R
    return (dist_L, dist_R)


def calc_dist_vel(data):
    time_now = 0
    time_prev = 0
    dist_L = 0
    dist_R = 0
    for i in range(len(data.time)):
        time_now = data.time[i]
        dist_L = dist_L + (data.velL[i] * (time_now - time_prev))
        dist_R = dist_R + (data.velR[i] * (time_now - time_prev))
        time_prev = time_now
    return (dist_L, dist_R)


def calc_pos_vel(data):
    time_now = 0
    time_prev = 0
    pos_x=0
    pos_y=0
    pos_theta=0
    dist_L = 0
    dist_R = 0
    for i in range(len(data.time)):
        dist_R, dist_L = [0,0]
        time_now = data.time[i]
        dist_L = (data.velL[i] * (time_now - time_prev))
        dist_R = (data.velR[i] * (time_now - time_prev))
        # print(time_now,' ', time_prev, ' ', time_now-time_prev)
        delta_pos_theta = np.arcsin((dist_R-dist_L)/axle_length)
        pos_theta += delta_pos_theta
        delta_pos_x, delta_pos_y = pol2cart((dist_L + dist_R) / 2, pos_theta)
        pos_x += delta_pos_x
        pos_y += delta_pos_y
        time_prev = time_now
    return (pos_x, pos_y, ((pos_theta*180/np.pi)+180)%360 -180)


def calc_pos_enc(data):
    pos_x=0
    pos_y=0
    pos_theta=0
    dist_L = 0
    dist_R = 0
    for i in range(len(data.time)):
        pos_now_L = data.posL[i]
        if (i >= 1):
            pos_prev_L = data.posL[i-1]
        else:
            pos_prev_L = pos_now_L
        delta_encoder_L = (pos_now_L - pos_prev_L + encoder_max/2) % encoder_max - encoder_max/2
        # (4 x ticks per rev x gear-ratio) / (wheel_diameter x Π)
        magic = (4 * ticks_per_rev * gear_ratio) / (np.pi * wheel_d)
        dist_L = (delta_encoder_L) / magic

        pos_now_R = data.posR[i]
        if (i >= 1):
            pos_prev_R = data.posR[i-1]
        else:
            pos_prev_R = pos_now_R
        delta_encoder_R = (pos_now_R - pos_prev_R + encoder_max/2) % encoder_max - encoder_max/2
        # (4 x ticks per rev x gear-ratio) / (wheel_diameter x Π)
        dist_R = (delta_encoder_R) / magic
        delta_pos_theta = np.arcsin((dist_R-dist_L)/axle_length)
        pos_theta += delta_pos_theta
        delta_pos_x, delta_pos_y = pol2cart((dist_L + dist_R) / 2, pos_theta)
        pos_x += delta_pos_x
        pos_y += delta_pos_y
    return (pos_x, pos_y, ((pos_theta*180/np.pi)+180)%360 -180)


def listener_wheels():
    rospy.init_node('encoder_', anonymous=True)
    # Subscribe topics and bind with callback functions
    rospy.Subscriber("/PIONIER6/RosAria/wheels", JointState, callback_wheels)


def listener_pose():
    rospy.init_node('pose_', anonymous=True)
    # Subscribe topics and bind with callback functions
    rospy.Subscriber("/PIONIER6/RosAria/pose", Odometry, callback_pose)


def callback_wheels(msg):
    global wheels_data
    global time_begin_wheels
    if not time_begin_wheels:
        time_begin_wheels = rospy.Time.now().to_sec()
    time = rospy.Time.now().to_sec() - time_begin_wheels
    pos=list(msg.position)
    vel=list(msg.velocity)
    wheels_data.time.append(time)
    wheels_data.posL.append(pos[0])
    wheels_data.posR.append(pos[1])
    wheels_data.velL.append(vel[0])
    wheels_data.velR.append(vel[1])
    return wheels_data


def callback_pose(msg):
    global pose_data
    pose_data.x.append(msg.pose.pose.position.x)
    pose_data.y.append(msg.pose.pose.position.y)
    pose_data.z.append(msg.pose.pose.orientation.z)
    pose_data.w.append(msg.pose.pose.orientation.w)
    return pose_data


def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x*180/np.pi, pitch_y*180/np.pi, yaw_z*180/np.pi 


if __name__=="__main__":
    wheels_data = data()
    pose_data = pose()
    time_begin_wheels=0
    time_begin_pose=0
    result = get_csv('square_left.csv')
    print("Forward Results")
    print("vel - Distance L wheel: %s, Distance R wheel: %s" %  calc_dist_vel(result))
    print("enc - Distance L wheel: %s, Distance R wheel: %s" %  calc_dist_enc(result))
    print("vel - Position X: %.1f mm, Position Y: %.1f mm, Theta: %.1f°" % calc_pos_vel(result))
    print("enc - Position X: %.1f mm, Position Y: %.1f mm, Theta: %.1f°" % calc_pos_enc(result))

    # listener_wheels()
    # rospy.spin()

    # print("\nForward Results - Rosbag")
    # print("vel - Distance L wheel: %s, Distance R wheel: %s" %  calc_dist_vel(wheels_data))
    # print("enc - Distance L wheel: %s, Distance R wheel: %s" %  calc_dist_enc(wheels_data))
    # print("vel - Position X: %.1f mm, Position Y: %.1f mm, Theta: %.0f°" % calc_pos_vel(wheels_data))
    # print("enc - Position X: %.1f mm, Position Y: %.1f mm, Theta: %.0f°" % calc_pos_enc(wheels_data))
    
    listener_pose()
    rospy.spin()
    x = pose_data.x[-1] - pose_data.x[0]
    y = pose_data.y[-1] - pose_data.y[0]
    z = pose_data.z[-1] - pose_data.z[0]
    w = pose_data.w[-1] - pose_data.w[0]
    theta = euler_from_quaternion(0,0,z,w)[2]
    print("pose - Position X: %s mm, Position Y: %s mm, Theta: %s°" % (x*1000,y*1000,theta))

