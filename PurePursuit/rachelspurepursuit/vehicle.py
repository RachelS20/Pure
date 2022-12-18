import math
import numpy as np
class Vehicle:
    """
    this class is the vehicle class.
    
    params:
    position - x, y coordiantes of vehicle in global system [m]
    velocity - u, v velocities components in x, y respectively [m/s]
    angular velocity - omega [rad/sec]
    psi - heading angle[rad]
    weight - weight of the vehicle [kg]
    L - 2.486 [m]
    look ahead distance - 5[m]

    funcs:
    transG2E - transformation between global to path
    """

    def __init__(self, position = [0, 0], velocity=[0, 0], angular_velocity=0, psi=0, weight= 1860, servo={"time_delay": 200, "max_command": 45, "rate_limiter": 20}, delta= 0, L= 2.486, look_ahead_distance= 5):
        self.position = position
        self.velocity = velocity
        self.angular_velocity = angular_velocity
        self.psi = psi
        self.weight = weight
        self.servo = servo
        self.L = L 
        self.look_ahead_distance = look_ahead_distance
        self.delta = delta
        
        # time delay in [msec], max_command in [deg], rate_limiter in [deg/sec] 

    def transE2G(self, x_e, y_e, psi):
        """
        transformation between ego to global 
        """
        x_g = x_e*math.cos(psi) - y_e*math.sin(psi) + self.position[0]
        y_g = x_e*math.sin(psi) + y_e*math.cos(psi) + self.position[1]

        return [x_g, y_g]
    
    def move(self,time_step, delta):
        """
        time step
        """
        time_history = [[self.position[0]],[self.position[1]]]
        #calc delta
        print("delta", delta)
        if abs(delta) > abs(self.servo["max_command"]/180*math.pi):
            delta = self.servo["max_command"]*np.sign(delta)/180*math.pi
        if (abs(delta)/math.pi*180 < 1):
            V = np.linalg.norm(self.velocity)
            self.angular_velocity = 0
        else:
            R = self.L/math.tan(delta)
            V = np.linalg.norm(self.velocity)
            self.angular_velocity = V/R
            print("R", R)
        
        
        time_step2 = time_step/5
        time_to_command = abs((delta - self.delta)/self.servo["rate_limiter"]) + self.servo["time_delay"]/1000
        t = 0
        while t < time_step:
            if t > self.servo["time_delay"]/1000:
                if t < time_to_command:
                    self.delta = self.delta + time_step2*self.servo["rate_limiter"]*math.pi/180*np.sign(delta - self.delta)
                else:
                    self.delta = delta
            
            self.psi = self.psi + self.angular_velocity*time_step2
            
            self.velocity[0] = V * math.cos(self.psi)
            self.velocity[1] = V * math.sin(self.psi)
            self.position[0] = self.position[0] + time_step2*self.velocity[0]
            self.position[1] = self.position[1] + time_step2*self.velocity[1]
            time_history[0].append(self.position[0])
            time_history[1].append(self.position[1])
            t += time_step2
        return time_history

    
    