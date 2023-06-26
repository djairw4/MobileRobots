import json
import math
import numpy as np
import pylab as pl
from matplotlib import collections  as mc

def get_robot_pos_orien(line1, line2, corner):
    x = point2line_dist(line1[0],line1[1],(0,0))
    y = point2line_dist(line2[0],line2[1],(0,0))
    a, b = np.polyfit([line2[0][0], line2[1][0]], [line2[0][1], line2[1][1]], 1)
    phi = calculateAngle(-a,0)
    return 1.15-x, 1.15-y, phi*180/math.pi

def calculateAngle(A1, A2):
    cos_phi = (A1*A2+1)/(math.sqrt(A1**2+1)*math.sqrt(A2**2+1))
    phi=math.acos(cos_phi)
    return phi

def findCorner(line1, line2, d):
    if point2point_dist(line1[0],line2[0])<d or point2point_dist(line1[0],line2[1])<d \
        or point2point_dist(line1[1],line2[0])<d or point2point_dist(line1[1],line2[1])<d:
        a, b = np.polyfit([line1[0][0], line1[1][0]], [line1[0][1], line1[1][1]], 1)
        A1=-a
        C1=-b
        a, b = np.polyfit([line2[0][0], line2[1][0]], [line2[0][1], line2[1][1]], 1)
        A2=-a
        C2=-b
        cos_phi = (A1*A2+1)/(math.sqrt(A1**2+1)*math.sqrt(A2**2+1))
        phi=math.acos(cos_phi)
        print(phi)
        if phi < math.pi*5/9 and phi > math.pi*4/9:
            x=(C2-C1)/(A1-A2)
            y=(A1*(-C2)-A2*(-C1))/(A1-A2)
            return True, (x,y)
    return False, (0,0)

def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x, y)

def del_inf(points):
    return [p for p in points if (p[0]<1.5 and p[1]<1.5 and p[0]>-1.5 and p[1]>-1.5)]
    # return [p for p in points if (p[0]<3 and p[1]<3 and p[0]>-3 and p[1]>-3)]

def point2line_dist(P1,P2,P0):
    return abs((P2[0]-P1[0])*(P1[1]-P0[1])-(P1[0]-P0[0])*(P2[1]-P1[1]))\
        /np.sqrt((P2[0]-P1[0])**2+(P2[1]-P1[1])**2)

def point2line_dist2(P, A, B, C):
    return abs(A*P[0]+B*P[1]+C)/np.sqrt((A**2)+(B**2))

def point2point_dist(P1,P2):
    return np.sqrt((P1[0]-P2[0])**2+(P1[1]-P2[1])**2)

def check_line(a, b, x, y,d):
    for i in range(len(x)):
        if point2line_dist2((x[i],y[i]),-a,1,-b)>d:
            return False
    return True

def incremental_method4(points,d,d1):
    begin = points[0]
    end = points[4]
    x = np.array([begin[0],points[1][0],points[2][0],points[3][0],end[0]])
    y = np.array([begin[1],points[1][1],points[2][1],points[3][1],end[1]])
    a, b = np.polyfit(x, y, 1)
    k=5
    while not check_line(a,b,x,y,d):
        end = points[k]
        x=x[1:]
        y=y[1:]
        np.append(x, end[0])
        np.append(y, end[1])
        a, b = np.polyfit(x, y, 1)
        k = k+1
    lines = []
    while k < len(points):
        point = points[k]
        if point2line_dist2(point,-a,1,-b)<d and point2point_dist(end,point)<d1:
            x= np.append(x,point[0])
            y= np.append(y,point[1])
            a, b = np.polyfit(x, y, 1)
            end = point
        else:
            if(len(x)>10):
                lines.append([begin,end])
            if k<len(points)-5:
                begin = point
                tmp = points[k:k+4]
                x = np.array([p[0] for p in tmp])
                y = np.array([p[1] for p in tmp])
                end = points[k+4]
                k=k+4
                a, b = np.polyfit(x, y, 1)
                while (not check_line(a,b,x,y,d)) and k<len(points):
                    end = points[k]
                    x=x[1:]
                    y=y[1:]
                    np.append(x, end[0])
                    np.append(y, end[1])
                    a, b = np.polyfit(x, y, 1)
                    k = k+1
            else:
                begin= point
                x = np.array([begin[0]])
                y = np.array([begin[1]])
                a=0
                b=1000
        k=k+1
    if len(x)>10:
        lines.append([begin,end])
    return lines

###############################################################

if __name__=="__main__":
    x = np.arange(0,512)
    theta = (np.pi/512 )*(x-256)  

    ##reading data from the file
    json_data = open('lab3/line_localization_1.json')
    data = json.load(json_data)

    skan=data[4]["scan"] 

    merged_list = [(skan[i], theta[i]) for i in range(0, len(theta))]
    cart=[pol2cart(float(p[0]),p[1]) for p in merged_list]
    cart = del_inf(cart)
    lines=incremental_method4(cart,0.02,0.05)
    c = np.array([(1, 0, 0, 1), (0, 1, 0, 1), (0, 0, 1, 1)])
    lc = mc.LineCollection(lines, colors=c, linewidths=2)
    fig, ax = pl.subplots()
    pl.scatter([x[0] for x in cart],[x[1] for x in cart],s=2, c='black')
    ax.add_collection(lc)
    for i in range(len(lines)):
        for j in range(i,len(lines)):
            if i != j:
                detected, point = findCorner(lines[i], lines[j],0.10)
                if detected:
                    ax.scatter(point[0],point[1],c='r', zorder=100)
                    print(get_robot_pos_orien(lines[i], lines[j], point))

    pl.xlim([-1, 1.5])
    pl.ylim([-1, 1.5])
    pl.show()

