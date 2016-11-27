from math import *
import random


# helper function to map all angles onto [-pi, pi]
def angle_trunc(a):
    while a < 0.0:
        a += pi * 2
    return ((a + pi) % (pi * 2)) - pi

class robot:

    def __init__(self, x = 0.0, y = 0.0, heading = 0.0, turning = 2*pi/10, distance = 1.0):
        """This function is called when you create a new robot. It sets some of 
        the attributes of the robot, either to their default values or to the values
        specified when it is created."""
        self.x = x
        self.y = y
        self.heading = heading
        self.turning = turning # only applies to target robots who constantly move in a circle
        self.distance = distance # only applies to target bot, who always moves at same speed.
        self.turning_noise    = 0.0
        self.distance_noise    = 0.0
        self.measurement_noise = 0.0


    def set_noise(self, new_t_noise, new_d_noise, new_m_noise):
        """This lets us change the noise parameters, which can be very
        helpful when using particle filters."""
        self.turning_noise    = float(new_t_noise)
        self.distance_noise    = float(new_d_noise)
        self.measurement_noise = float(new_m_noise)


    def move(self, turning, distance, tolerance = 0.001, max_turning_angle = pi):
        """This function turns the robot and then moves it forward."""
        # apply noise, this doesn't change anything if turning_noise
        # and distance_noise are zero.
        turning = random.gauss(turning, self.turning_noise)
        distance = random.gauss(distance, self.distance_noise)

        # truncate to fit physical limitations
        turning = max(-max_turning_angle, turning)
        turning = min( max_turning_angle, turning)
        distance = max(0.0, distance)

        # Execute motion
        self.heading += turning
        self.heading = angle_trunc(self.heading)
        self.x += distance * cos(self.heading)
        self.y += distance * sin(self.heading)

    def move_in_circle(self):
        """This function is used to advance the runaway target bot."""
        self.move(self.turning, self.distance)

    def sense(self):
        """This function represents the robot sensing its location. When
        measurements are noisy, this will return a value that is close to, 
        but not necessarily equal to, the robot's (x, y) position."""
        return (random.gauss(self.x, self.measurement_noise),
                random.gauss(self.y, self.measurement_noise))

    def __repr__(self):
        """This allows us to print a robot's position"""
        return '[%.5f, %.5f]'  % (self.x, self.y)
    
    #setting particles for particle filter 
    def set_new(self, world_size):
        self.x = random.random() * world_size
        self.y = random.random() * world_size
        self.heading = random.random() * 2.0 * pi
        self.turning_noise    = 0.0
        self.distance_noise = 0.0
        self.measurement_noise   = 0.0
    
    #measuring distance from the object to landmark
    def sense_landmark(self, landmarks):
        Z = []
        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            Z.append(dist)
        return Z
    
        
    def Gaussian(self, mu, sigma, x):
        
        # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
        return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))
    
    
    def measurement_prob(self, measurement, landmarks):
        
        # calculates how likely a measurement should be
        
        prob = 1.0;
        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            prob *= self.Gaussian(dist, self.measurement_noise, measurement[i])
        return prob
    
    def move_particle(self, turning, distance):
        if distance < 0:
            raise ValueError, 'Robot cant move backwards'         
        
        # turn, and add randomness to the turning command
        heading= self.heading + float(turning) + random.gauss(0.0, self.turning_noise)
        heading = angle_trunc(heading)
        # move, and add randomness to the motion command
        dist = float(distance) + random.gauss(0.0, self.distance_noise)
        x = self.x + (cos(heading) * dist)
        y = self.y + (sin(heading) * dist)

        # set particle
        res = robot()
        res.set(x, y, heading)
        res.set_noise(self.distance_noise, self.turning_noise, self.measurement_noise)
        return res


    
    def set(self, new_x, new_y, new_orientation):
        self.x = float(new_x)
        self.y = float(new_y)
        self.heading = float(new_orientation)
        
