import sys
import math
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

def distance_2p(p1,p2):
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def find_angle(p1, p2):
    ang = math.atan2(p2[1]-p1[1], p2[0]-p1[0])
    if np.isnan(ang):
        if p2[1] > p1[1] : return math.pi/2
        else: return 3*math.pi/2
    if ang*180/math.pi < 0:
        return 2*math.pi + ang

    return ang
    


def high_resolution_path(path, k):
    # k multipy num of points by
    x = np.array(path.iloc[:, 0])
    n = np.arange(x.shape[0]) 
    y = np.array(path.iloc[:, 1])

    x_spline = interp1d(n, x,kind='cubic')

    n_ = np.linspace(n.min(), n.max(), k)
    y_spline = interp1d(n, y,kind='cubic')

    x_ = x_spline(n_)
    y_ = y_spline(n_)
    hpath = pd.DataFrame([x_.T, y_.T])
    hpath = hpath.T
    hpath.to_csv("hpath.csv", index=False, header=False)

    return hpath


class Path:
    """
    this clas is the path class.
    read the path coordinates from CSV file.
    the coordinates are in global system.

    params:
    xypath - 2D array of path coordiantes in global system [m]

    funcs:
    transG2P - transformation between global to path
    """

    def __init__(self,xypathfile="path.csv"):
        self.xypath = pd.read_csv(xypathfile)
        self.hpath = high_resolution_path(self.xypath, 500)
    
    def showcurve(self, high=1):
        """
        this show curve. high = 1 for high resolution, 0 for default path
        """
        if high == 1:
            plt.plot(self.hpath.iloc[:, 0], self.hpath.iloc[:, 1])
            plt.show()
        else:
            plt.plot(self.xypath.iloc[:, 0], self.xypath.iloc[:, 1])
            plt.show()

    # finds the point in the global coordinate system that correspond to (s_f, 0)
    def find_point_with_distance(self, s_f):
        epsi = 0.1
        s = 0 # distance
        i = 0
        while i < (len(self.hpath) - 1):
            s += distance_2p(self.hpath.iloc[i], self.hpath.iloc[i+1])
            if s + epsi > s_f:
                theta = find_angle(self.hpath.iloc[i], self.hpath.iloc[i+1])
                return [self.hpath.iloc[i+1][0], self.hpath.iloc[i+1][1], theta]
            i += 1
        sys.exit("point not found1")

    def find_distance_of_point_on_curve(self, cx, cy):
        epsi = 0.1
        s = 0 # distance
        i = 0
        
        while i < (len(self.hpath) - 1):
            s += distance_2p(self.hpath.iloc[i], self.hpath.iloc[i+1])
            d2p = distance_2p([cx, cy], [self.hpath.iloc[i][0], self.hpath.iloc[i][1]])
            if d2p < epsi:
                return s
            i += 1
        # sys.exit("point not found2")
        return np.nan

    def trans_P2G(self, s, t):
        cx, cy, theta = self.find_point_with_distance(s)
        if cx.isnull():
            return np.nan 
        return [cx + t*np.sin(theta), cy - t*np.cos(theta)]

    def trans_G2P(self, x_g, y_g):
        i = 0
        epsi = 0.1
        while i < (len(self.hpath) - 1):
            v1 = [x_g, y_g]-self.hpath.iloc[i]
            v2 = self.hpath.iloc[i+1]-self.hpath.iloc[i]
            dot_product = np.dot(v1, v2)/np.linalg.norm(v1,2)/np.linalg.norm(v2,2)
            if dot_product < epsi:
                cx, cy = self.hpath.iloc[i]
                t = abs(distance_2p(self.hpath.iloc[i],[x_g, y_g]))
                s = self.find_distance_of_point_on_curve(cx, cy)
                return [s, t]
            i += 1
        sys.exit("point not found3")

    def trans_E2P(self, x_e, y_e, psi):
        epsi = 0.5
        epsi2 = 0.05 #[deg]

        slope = math.tan((psi+epsi2+90)/180*math.pi)
        
        for i in range(len(self.hpath)):
            
            if abs(slope) < 15:
                d = slope*(x_e - self.hpath.iloc[i][0]) - (y_e - self.hpath.iloc[i][1])
            else:
                d = x_e - self.hpath.iloc[i][0]
            if abs(d) < epsi:
                return [self.hpath.iloc[i][0], self.hpath.iloc[i][1]]
            
        return [np.nan, np.nan]
        # sys.exit("point not found5")


    def find_ref_point(self, x0, y0, l_d, psi):
        epsi = 5
        for i in range(len(self.hpath)):
            d = abs(l_d**2 -distance_2p([x0, y0], [self.hpath.iloc[i][0], self.hpath.iloc[i][1]])**2)
            angle = find_angle([x0, y0], [self.hpath.iloc[i][0], self.hpath.iloc[i][1]])
            if abs(d) < epsi:
                print((angle)*180/math.pi, (psi)*180/math.pi)
                if (abs((angle - psi)*180/math.pi)%180 < 45):
                    return [self.hpath.iloc[i][0], self.hpath.iloc[i][1]]
        print('here')
        return [np.nan, np.nan]
        # sys.exit("point not found5")






        


