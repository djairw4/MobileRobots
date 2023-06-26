import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import roslib; 
import rospy
import json
import math

from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from scipy.ndimage import binary_dilation, generate_binary_structure

roomSize = 20
fieldSize = 0.5
mapScale = 1/fieldSize
mapSize = math.ceil(roomSize/fieldSize)
p_occ=0.95
p_free=0.3
sensorDisp = 0.18
new_scan = False

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
    print("x=%f y=%f th=%f" %(rx,ry,rth))
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


if __name__ == "__main__":
    x = np.arange(0,512)
    theta = (np.pi/512 )*(x-256)  
    gridMap = np.zeros((mapSize,mapSize))
    pose = [0,0,0]
    scan = list()
    probMap = gridMap

    ##reading data from the file
    # json_data = open('mr2/lab1/map_boxes_0.json')
    # json_data = open('mr2/lab1/map_boxes_1.json')
    # json_data = open('mr2/lab1/map_round.json')
    json_data = open('mr2/lab1/map_big.json')
    data = json.load(json_data)
    for j in range(len(data)):
        scan=data[j]["scan"]
        pose=data[j]["pose"]
        for i in range(len(scan)):
            if np.isnan(scan[i]) or np.isinf(scan[i]):
                continue
            obst = pol2cart(scan[i]+sensorDisp, pose[2]/180*np.pi+theta[i])
            obst = addDisplacement(pose,obst)
            obst = scale(obst)
            robot = scale(pose)
            Bresenham(obst[0]+mapSize//2,obst[1]+mapSize//2,robot[0]+mapSize//2,robot[1]+mapSize//2)
    probMap = 1-(1/(1+np.exp(gridMap)))
    if fieldSize >= 0.5:
        rgba = Wavefront(gridMap, scale((6,16)), scale((10,10)))
    else:
        rgba = Wavefront_dilation(gridMap, scale((6,16)), scale((10,10))) 
    plt.imshow(rgba, interpolation="nearest")
    # plt.imshow(probMap, interpolation="nearest",cmap='Blues')
    # plt.colorbar()
    plt.show()


    # plt.imshow(probMap, interpolation="nearest",cmap='Blues')
    # rospy.init_node('scan_and_pose_sub', anonymous=True) #make node 
    # rospy.Subscriber('/PIONIER6/RosAria/pose',Odometry,poseinfo)
    # rospy.Subscriber('/PIONIER6/scan', LaserScan,callback_scan)
    # map_publisher=rospy.Publisher('/my/gridmap', OccupancyGrid, queue_size=10)
    # while not rospy.is_shutdown():
    #     if new_scan:
    #         for i in range(len(scan)):
    #             if np.isnan(scan[i]) or np.isinf(scan[i]) or np.isinf(pose[0]) or np.isinf(pose[1]) or np.isinf(pose[2]) or np.isnan(pose[0]) or np.isnan(pose[1]) or np.isnan(pose[2]):
    #                 continue
    #             obst = pol2cart(scan[i]+sensorDisp, pose[2]+theta[i])
    #             obst = addDisplacement(pose,obst)
    #             obst = scale(obst)
    #             robot = scale(pose)
    #             Bresenham(obst[0]+mapSize//2,obst[1]+mapSize//2,robot[0]+mapSize//2,robot[1]+mapSize//2)
    #         probMap = 1-(1/(1+np.exp(gridMap)))     
    #         if fieldSize >= 0.5:
    #             rgba = Wavefront(gridMap, scale((6,16)), scale((10,10)))
    #         else:
    #             rgba = Wavefront_dilation(gridMap, scale((6,16)), scale((10,10))) 
    #         map_publisher.publish(numpy_to_occupancy_grid(probMap))
    #         # plt.imshow(probMap, interpolation="nearest",cmap='Blues')
    #         plt.imshow(gridMap, interpolation="nearest",cmap='Blues')
    #         plt.pause(0.01)
    #         new_scan = False

        