import sys
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from rachelspurepursuit import Vehicle
from rachelspurepursuit import Path

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

def plotting(dataSet, t, path):
    numDataPoints = len(t)
    def animate_func(num):
        # ax.clear()  # Clears the figure to update the line, point,   
                    # title, and axes
        # Updating Trajectory Line (num+1 due to Python indexing)
        ax.plot(dataSet[0][:num+1], dataSet[1][:num+1], c='blue')
        # Updating Point Location 
        ax.scatter(dataSet[0][num], dataSet[1][num], c='blue', marker='o')
        # Adding Constant Origin
        ax.plot(dataSet[0][0], dataSet[1][0],     
                c='black', marker='o')
        # Adding Ref Point
        ax.plot(dataSet[2][num], dataSet[3][num],     
                c='red', marker='o')
        if num == 0:
            ax.plot(path.hpath[0], path.hpath[1])

        # Setting Axes Limits
        ax.set_xlim([0, 80])
        ax.set_ylim([0, 50])

        # Adding Figure Labels
        ax.set_title('Trajectory \nTime = ' + str(np.round(t[num],decimals=2)) + ' sec')
        ax.set_xlabel('x')
        ax.set_ylabel('y')

    fig = plt.figure(1)
    ax = plt.axes()
    line_ani = animation.FuncAnimation(fig, animate_func, interval=100,   
                                   frames = numDataPoints, repeat = False)
    plt.show()

max_time = 10 #[sec]
step_size = 0.5 #[sec]
look_ahead_distance =  10 #[m] should be depend on speed
servo={"time_delay": 200, "max_command": 45, "rate_limiter": 20}
delta = 0
## Ford Fusion
weight = 1860 #[kg]
L = 2.486 #[m]

def simulation(x0, y0, psi, V, L, delta, look_ahead_distance):
    u = V * math.cos(psi)
    v = V * math.sin(psi)
    vehicle = Vehicle([x0, y0], [u, v], 0, psi, weight, servo, delta, L, look_ahead_distance)
    path = Path()
    ## run sim
    t=0
    epsi = 0.1
    time_history = [[],[],[],[],[]]
    x_ref = 0 
    y_ref = 0
    while (t < max_time) | (distance_2p(path.hpath.iloc[-1], [x_ref, y_ref]) < epsi):
        ## calc the starting point
        cx, cy = path.trans_E2P(x0, y0, psi)
        s_start = path.find_distance_of_point_on_curve(cx, cy)
        ## calc the ref point
        x_ref, y_ref = path.find_ref_point(x0, y0, look_ahead_distance, psi)
        ## calc the radius
        alpha = find_angle([x0, y0], [x_ref, y_ref])
        if (abs((math.sin(alpha))) < epsi):
            delta = 0
            vehicle.angular_velocity = 0
        else:
            R = look_ahead_distance/(2*math.sin(alpha))
            delta = np.arctan(vehicle.L/R)
        
        temp = vehicle.move(step_size, delta)
        time_history[0].extend(temp[0])
        time_history[1].extend(temp[1])
        time_history[2].extend(np.ones(len(temp[1]))*x_ref)
        time_history[3].extend(np.ones(len(temp[1]))*y_ref)
        time_history[4].extend(np.ones(len(temp[1]))*vehicle.angular_velocity)
        x0 = vehicle.position[0]
        y0 = vehicle.position[1]
        psi = vehicle.psi
        t += step_size

    # Plotting the Animation
    t = np.linspace(0, max_time, len(time_history[0]))
    plotting(time_history, t, path)

if __name__ == "__main__":
    x0 = float(sys.argv[2])
    y0 = float(sys.argv[4])
    psi = float(sys.argv[6])
    V = float(sys.argv[8])

    simulation(x0, y0, psi/180*math.pi, V, L, delta, look_ahead_distance)


