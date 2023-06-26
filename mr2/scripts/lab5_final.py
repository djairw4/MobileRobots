import matplotlib.pyplot as plt
import numpy as np
import rospy
import math
import numpy.ma as ma
from threading import Thread
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from scipy.ndimage import binary_dilation, generate_binary_structure
import time
from geometry_msgs.msg import Twist

twist=0
roomSize = 20
fieldSize = 1
mapScale = 1/fieldSize
mapSize = math.ceil(roomSize/fieldSize)
p_occ=0.95
p_free=0.3
sensorDisp = 0.18
new_scan = False

def set_vel(v,w):
   global twist
   twist = Twist()
   twist.linear.x = v
   twist.angular.z =w
   pub.publish(twist)

def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x, y)

def poseinfo(msg):
    global pose
    rx= msg.pose.pose.position.x
    ry= msg.pose.pose.position.y
    quaternion = (
    msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.y,
    msg.pose.pose.orientation.z,
    msg.pose.pose.orientation.w)
    rth=euler_from_quaternion (quaternion)[2]
    pose = [rx,ry,rth]

def callback_scan(msg):
    global scan
    global new_scan
    new_scan = True
    scan=list(msg.ranges)

def numpy_to_occupancy_grid(arr):
    msg = OccupancyGrid()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'map'
    msg.info.width = arr.shape[1]
    msg.info.height = arr.shape[0]
    msg.info.resolution = fieldSize
    data = np.array(arr.flatten() * 100, dtype=np.int8)
    msg.data = list(data)
    return msg

def scale(point):
    try:
        x = round(point[0]/fieldSize)
        y = round(point[1]/fieldSize)
    except:
        print(point)
        x=0
        y=0
    return (x,y)

def addDisplacement(pose, point):
    x = pose[0]+point[0]
    y = pose[1]+point[1]
    return (x,y)

def Bresenham(x1, y1, x2, y2):
    x = round(x1) 
    y = round(y1)
    if x1 < x2:
        xi = 1
        dx = x2 - x1
    else:
        xi = -1
        dx = x1 - x2
    if y1 < y2:
        yi = 1
        dy = y2 - y1
    else:
        yi = -1
        dy = y1 - y2
    gridMap[x][y] += np.log(p_occ/(1-p_occ))
    if (dx > dy):
        ai = (dy - dx) * 2
        bi = dy * 2
        d = bi - dx
        while (x != x2):
            if d >= 0:
                x += xi
                y += yi
                d += ai
            else:
                d += bi
                x += xi
            gridMap[x][y] += np.log(p_free/(1-p_free))
    else:
        ai = ( dx - dy ) * 2
        bi = dx * 2
        d = bi - dy
        while y != y2:
            if d >= 0:
                x += xi
                y += yi
                d += ai
            else:
                d += bi
                y += yi
            gridMap[x][y] += np.log(p_free/(1-p_free))

def neighbors(pos,all_cells):
    nghbrs = [(c[0],c[1]) for c in all_cells if (c[0]==pos[0] and (c[1]==pos[1]+1 or c[1]==pos[1]-1)) or (c[1]==pos[1] and (c[0]==pos[0]+1 or c[0]==pos[0]-1))]
    return nghbrs
            
def Wavefront_dilation(map, goal_pos, robot_pos):
    map[map>=0] = 1
    map[map<0] = 0
    kernel=generate_binary_structure(2, 2)
    map = binary_dilation(map, structure = kernel,iterations=2).astype(map.dtype)
    map[map>0] = map.shape[0]
    map[map==0] = -1
    map[goal_pos[0]][goal_pos[1]] = 0
    map_no_waves = map.copy()
    i=0
    while map[robot_pos[0]][robot_pos[1]]==-1 and i<map.shape[0]:
        front=np.transpose((map==i).nonzero())
        i+=1
        for f in front:
            if f[0]>0:
                if map[f[0]-1][f[1]]==-1:
                    map[f[0]-1][f[1]] = i
            if f[0]< map.shape[0]-1:
                if map[f[0]+1][f[1]] == -1:
                    map[f[0]+1][f[1]] = i
            if f[1]>0:
                if map[f[0]][f[1]-1] == -1:
                    map[f[0]][f[1]-1] = i
            if f[1]< map.shape[1]-1:
                if map[f[0]][f[1]+1] == -1:
                    map[f[0]][f[1]+1] = i
    path = []
    i = map[robot_pos[0]][robot_pos[1]]
    cur_pos = robot_pos
    path.append(robot_pos)
    while i>0:
        cells = np.transpose((map==i-1).nonzero())
        cells = neighbors(cur_pos,cells)
        if cells:
            cur_pos=cells[0]
        else:
            break
        path.append(cur_pos)
        i-=1
    cmap = plt.cm.Blues
    norm = plt.Normalize(map_no_waves.min(), map_no_waves.max())
    rgba = cmap(norm(map_no_waves))
    # norm = plt.Normalize(map.min(), map.max())
    # rgba = cmap(norm(map))
    for p in path:
        rgba[p[0], p[1]] = 0.1, 1, 0.1, 0.5
    rgba[robot_pos[0], robot_pos[1]] = 1, 0, 0,1
    rgba[goal_pos[0], goal_pos[1]] = 0, 1, 0,1
    return rgba

def Wavefront(map, goal_pos, robot_pos):
    map[map>=0] = map.shape[0]
    map[map<0] = -1
    map[goal_pos[0]][goal_pos[1]] = 0
    map_no_waves = map.copy()
    i=0
    while map[robot_pos[0]][robot_pos[1]]==-1 and i<map.shape[0]:
        front=np.transpose((map==i).nonzero())
        i+=1
        for f in front:
            if f[0]>0:
                if map[f[0]-1][f[1]]==-1:
                    map[f[0]-1][f[1]] = i
            if f[0]< map.shape[0]-1:
                if map[f[0]+1][f[1]] == -1:
                    map[f[0]+1][f[1]] = i
            if f[1]>0:
                if map[f[0]][f[1]-1] == -1:
                    map[f[0]][f[1]-1] = i
            if f[1]< map.shape[1]-1:
                if map[f[0]][f[1]+1] == -1:
                    map[f[0]][f[1]+1] = i
    path = []
    i = map[robot_pos[0]][robot_pos[1]]
    cur_pos = robot_pos
    path.append(robot_pos)
    while i>0:
        cells = np.transpose((map==i-1).nonzero())
        cells = neighbors(cur_pos,cells)
        if cells:
            cur_pos=cells[0]
        else:
            break
        path.append(cur_pos)
        i-=1
    cmap = plt.cm.Blues
    norm = plt.Normalize(map_no_waves.min(), map_no_waves.max())
    rgba = cmap(norm(map_no_waves))
    # norm = plt.Normalize(map.min(), map.max())
    # rgba = cmap(norm(map))
    for p in path:
        rgba[p[0], p[1]] = 0.1, 1, 0.1, 0.5
    rgba[goal_pos[0], goal_pos[1]] = 0, 1, 0,1
    rgba[robot_pos[0], robot_pos[1]] = 1, 0, 0,1
    return rgba

def point2point_dist(P1,P2):
    return np.sqrt((P1[0]-P2[0])**2+(P1[1]-P2[1])**2)

# if the goal is not available, the function sets a temporary goal
def Wavefront2(map, goal_pos, robot_pos):
    global goal_dist
    map[map>=0] = 2*map.shape[0]
    map[map<0] = -1
    map[goal_pos[0]][goal_pos[1]] = 0
    map_no_waves = map.copy()
    i=0
    while map[robot_pos[0]][robot_pos[1]]==-1 and i<2*map.shape[0]:
        front=np.transpose((map==i).nonzero())
        i+=1
        for f in front:
            if f[0]>0:
                if map[f[0]-1][f[1]]==-1:
                    map[f[0]-1][f[1]] = i
            if f[0]< map.shape[0]-1:
                if map[f[0]+1][f[1]] == -1:
                    map[f[0]+1][f[1]] = i
            if f[1]>0:
                if map[f[0]][f[1]-1] == -1:
                    map[f[0]][f[1]-1] = i
            if f[1]< map.shape[1]-1:
                if map[f[0]][f[1]+1] == -1:
                    map[f[0]][f[1]+1] = i
    path = []
    i = map[robot_pos[0]][robot_pos[1]]
    cur_pos = robot_pos
    tmp_goal = goal_pos
    path.append(robot_pos)
    while i < 0:
        mask = map!=-1
        masked_dist = ma.masked_array(goal_dist, mask)
        masked_min= ma.where(masked_dist == masked_dist.min())
        tmp_goal = (masked_min[0][0],masked_min[1][0])
        map[tmp_goal[0]][tmp_goal[1]] = 0
        i=0
        while map[robot_pos[0]][robot_pos[1]]==-1 and i<2*map.shape[0]:
            front=np.transpose((map==i).nonzero())
            i+=1
            for f in front:
                if f[0]>0:
                    if map[f[0]-1][f[1]]==-1:
                        map[f[0]-1][f[1]] = i
                if f[0]< map.shape[0]-1:
                    if map[f[0]+1][f[1]] == -1:
                        map[f[0]+1][f[1]] = i
                if f[1]>0:
                    if map[f[0]][f[1]-1] == -1:
                        map[f[0]][f[1]-1] = i
                if f[1]< map.shape[1]-1:
                    if map[f[0]][f[1]+1] == -1:
                        map[f[0]][f[1]+1] = i
        i = map[robot_pos[0]][robot_pos[1]]
    while i>0:
        cells = np.transpose((map==i-1).nonzero())
        cells = neighbors(cur_pos,cells)
        if cells:
            cur_pos=cells[0]
        else:
            break
        path.append(cur_pos)
        i-=1
    cmap = plt.cm.Blues
    norm = plt.Normalize(map_no_waves.min(), map_no_waves.max())
    rgba = cmap(norm(map_no_waves))
    for p in path:
        rgba[p[0], p[1]] = 0.1, 1, 0.1, 1
    rgba[tmp_goal[0], tmp_goal[1]] = 1, 1, 0,1
    rgba[goal_pos[0], goal_pos[1]] = 1, 1, 0,1
    rgba[robot_pos[0], robot_pos[1]] = 1, 0, 0,1
    return rgba, path

def follow_path2(path):
    if len(path) < 2:
        return
    destination = path[1]
    nextAngle = np.arctan2(destination[1] - path[0][1], destination[0] - path[0][0]) #[-pi, pi]
    robotAngle = pose[2]
    vel = 0.2
    lin_vel = 0.1
    diff = nextAngle - robotAngle
    sign = np.sign(diff)
    vel = sign*vel
    diff=abs(diff)
    while diff > 0.05:
        set_vel(0,vel)
        diff = abs(nextAngle - pose[2])
    set_vel(0,0)
    time.sleep(2)
    if (abs(nextAngle)>np.pi/4 and abs(nextAngle)<np.pi*0.75):
        ind = 1
    else:
        ind = 0
    destination = destination[ind]
    diff = destination - (pose[ind]+10)
    # diff = abs(destination - (pose[ind]*2+20))
    diff=abs(diff)
    while diff > 0.05:
        set_vel(lin_vel, 0)
        diff = abs(destination - (pose[ind]+10))
        print(pose[ind]+10, destination)
        # diff = abs(destination - (pose[ind]*2+20))
        # print(pose[ind]*2+20, destination)
    set_vel(0,0)
    time.sleep(2)

def mapping():
    global goal
    global new_scan
    global scan
    global theta
    global goal_dist
    global path
    while not rospy.is_shutdown():
        if goal != tuple(rospy.get_param('/myparam/goal')): 
            goal = tuple(rospy.get_param('/myparam/goal'))
            goal_dist = np.array([[ abs(i-goal[0])+abs(j-goal[1]) for j in range(goal_dist.shape[1])]
                        for i in range(goal_dist.shape[0])])
        if new_scan:
            for i in range(len(scan)):
                if np.isnan(scan[i]) or np.isinf(scan[i]) or np.isinf(pose[0]) or np.isinf(pose[1]) or np.isinf(pose[2]) or np.isnan(pose[0]) or np.isnan(pose[1]) or np.isnan(pose[2]):
                    continue
                obst = pol2cart(scan[i]+sensorDisp, pose[2]+theta[i])
                obst = addDisplacement(pose,obst)
                obst = scale(obst)
                robot = scale(pose)
                Bresenham(obst[0]+mapSize//2,obst[1]+mapSize//2,robot[0]+mapSize//2,robot[1]+mapSize//2)
            # probMap = 1-(1/(1+np.exp(gridMap)))     
            if fieldSize >= 0.5:
                rgba,path = Wavefront2(gridMap.copy(), goal, (robot[0]+mapSize//2,robot[1]+mapSize//2))
            else:
                rgba = Wavefront_dilation(gridMap.copy(), goal, (robot[0]+mapSize//2,robot[1]+mapSize//2)) 
            plt.imshow(rgba, interpolation="nearest")
            plt.pause(0.01)
            new_scan = False

if __name__ == "__main__":
    n =726
    # n =512
    x = np.arange(0,n)
    theta = (np.pi/n )*(x-n/2)  
    gridMap = np.zeros((mapSize,mapSize))
    pose = [0,0,0]
    scan = list()
    path = list()
    rospy.init_node('controller', anonymous=True) #make node 
    rospy.Subscriber('/PIONIER4/RosAria/pose',Odometry,poseinfo)
    rospy.Subscriber('/PIONIER4/scan', LaserScan,callback_scan)
    pub = rospy.Publisher('/PIONIER4/RosAria/cmd_vel', Twist, queue_size=10)
    # map_publisher=rospy.Publisher('/my/gridmap', OccupancyGrid, queue_size=10)
    goal = scale((5,10))
    rospy.set_param('/myparam/goal', list(goal))
    goal_dist = gridMap.copy()
    goal_dist = np.array([[ abs(i-goal[0])+abs(j-goal[1]) for j in range(goal_dist.shape[1])]
                            for i in range(goal_dist.shape[0])])
    Thread(target=mapping).start()
    while not rospy.is_shutdown():
        follow_path2(path)