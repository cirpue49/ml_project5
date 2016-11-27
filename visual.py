from robot import *
from math import *
from matrix import *
import random




def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def demo_grading(hunter_bot, target_bot, next_move_fcn, n_particles,n_landmarks, OTHER = None):
    """Returns True if your next_move_fcn successfully guides the hunter_bot
    to the target_bot. This function is here to help you understand how we 
    will grade your submission."""
    max_distance = 0.98 * target_bot.distance # 0.98 is an example. It will change.
    separation_tolerance = 0.02 * target_bot.distance # hunter must be within 0.02 step size to catch target
    caught = False
    ctr = 0
    #For Visualization
    import turtle
    window = turtle.Screen()
    window.bgcolor('white')
    chaser_robot = turtle.Turtle()
    chaser_robot.shape('arrow')
    chaser_robot.color('blue')
    chaser_robot.resizemode('user')
    chaser_robot.shapesize(0.3, 0.3, 0.3)
    broken_robot = turtle.Turtle()
    broken_robot.shape('turtle')
    broken_robot.color('green')
    broken_robot.resizemode('user')
    broken_robot.shapesize(0.3, 0.3, 0.3)
    size_multiplier = 15.0 #change size of animation
    chaser_robot.hideturtle()
    chaser_robot.penup()
    chaser_robot.goto(hunter_bot.x*size_multiplier, hunter_bot.y*size_multiplier-100)
    chaser_robot.showturtle()
    broken_robot.hideturtle()
    broken_robot.penup()
    broken_robot.goto(target_bot.x*size_multiplier, target_bot.y*size_multiplier-100)
    broken_robot.showturtle()
    measuredbroken_robot = turtle.Turtle()
    measuredbroken_robot.shape('circle')
    measuredbroken_robot.color('red')
    measuredbroken_robot.penup()
    measuredbroken_robot.resizemode('user')
    measuredbroken_robot.shapesize(0.1, 0.1, 0.1)
    broken_robot.pendown()
    chaser_robot.pendown()
    #End of Visualization
    # We will use your next_move_fcn until we catch the target or time expires.
    while not caught and ctr < 1000:
        # Check to see if the hunter has caught the target.
        hunter_position = (hunter_bot.x, hunter_bot.y)
        target_position = (target_bot.x, target_bot.y)
        separation = distance_between(hunter_position, target_position)
        if separation < separation_tolerance:
            print "You got it right! It took you ", ctr, " steps to catch the target."
            caught = True

        # The target broadcasts its noisy measurement
        target_measurement = target_bot.sense()

        try:
            next_move_fcn(hunter_position, hunter_bot.heading, target_measurement, max_distance, OTHER, n_particles, n_landmarks)
        except:
            print 'Error happened when calling next_move_fcn.'

        # This is where YOUR function will be called.
        turning, distance, OTHER = next_move_fcn(hunter_position, hunter_bot.heading, target_measurement, max_distance, OTHER, n_particles, n_landmarks)

        # Don't try to move faster than allowed!
        if distance > max_distance:
            distance = max_distance

        # We move the hunter according to your instructions
        hunter_bot.move(turning, distance)

        # The target continues its (nearly) circular motion.
        target_bot.move_in_circle()
        #Visualize it
        measuredbroken_robot.setheading(target_bot.heading*180/pi)
        measuredbroken_robot.goto(target_measurement[0]*size_multiplier, target_measurement[1]*size_multiplier-100)
        measuredbroken_robot.stamp()
        broken_robot.setheading(target_bot.heading*180/pi)
        broken_robot.goto(target_bot.x*size_multiplier, target_bot.y*size_multiplier-100)
        chaser_robot.setheading(hunter_bot.heading*180/pi)
        chaser_robot.goto(hunter_bot.x*size_multiplier, hunter_bot.y*size_multiplier-100)

        
        #End of visualization
        ctr += 1            
        if ctr >= 1000:
            print "It took too many steps to catch the target."
    return caught

def angle_trunc(a):
    """This maps all angles to a domain of [-pi, pi]"""

    while a < 0.0:
        a += pi * 2
    return ((a + pi) % (pi * 2)) - pi

def get_heading(hunter_position, target_position):
    """Returns the angle, in radians, between the target and hunter positions"""

    hunter_x, hunter_y = hunter_position
    target_x, target_y = target_position
    heading = atan2(target_y - hunter_y, target_x - hunter_x)
    heading = angle_trunc(heading)
    return heading
    
def estimate_next_pos(n_particles,n_landmarks, measurement, est_other = None):
    """Particle filter would estimate the next location"""

    N = n_particles

    #First time, this function is called
    #the robot is directly moving to present location of the target, not estimating next location for the target
    if not est_other:
        est_other = measurement
        xy_estimate = measurement

    #Second time, this fundction is called
    elif len(est_other) ==2:
        distance = distance_between(measurement, est_other)
        dx = measurement[0]-est_other[0]
        dy = measurement[1]-est_other[1]
        heading = atan(dy/dx)

        world_size = distance*10
        landmarks = [[random.random() * world_size,  random.random() * world_size] for i in range(n_landmarks)]
        
        p = []
        for i in range(N):
            r = robot()
            r.set_new(world_size)
            r.set_noise(0.05,0.05,distance/4.)
            p.append(r)
        est_other = []
        turning = 0
        est_other.append(measurement)
        est_other.append([distance])
        est_other.append(heading)
        est_other.append(turning)
        est_other.append(landmarks)
        est_other.append(world_size)
        est_other.append(p)
        xy_estimate = measurement


    #Except for first and seconf time
    elif len(est_other) ==7:
        est_other[1].append(distance_between(measurement, est_other[0]))
        distance = sum(est_other[1])/float(len(est_other[1]))
        dx = measurement[0] - est_other[0][0]
        dy = measurement[1] - est_other[0][1]
        heading = atan2(dy,dx)
        turning = angle_trunc(heading-est_other[2])
        est_other[0] = measurement
        est_other[2] = heading
        est_other[3] = turning     
        exp_bot =robot(measurement[0], measurement[1], heading, turning, distance)
        exp_bot.move_in_circle()
        Z = exp_bot.sense_landmark(est_other[4])

        #p2 is list for moved particles
        p2 = []
        for i in range(N):
            p2.append(est_other[6][i].move_particle(turning, distance))
        p = p2
        w = []
        for i in range(N):
            w.append(p[i].measurement_prob(Z, est_other[4]))

        #p3 is list for survived paricles
        p3 = []
        index = int(random.random() * N)
        beta = 0.0
        mw = max(w)
        for i in range(N):
            beta += random.random() * 2.0 * mw
            while beta > w[index]:
                beta -= w[index]
                index = (index + 1) % N
            p3.append(p[index])
        p = p3

        #Estimate a location by averaging survived partiles for x and y coordinate.
        x = 0
        y = 0
        for i in range(len(p)):
            x += p[i].x
            y += p[i].y
        average_x = x/len(p)
        average_y = y/len(p)
        xy_estimate = (average_x, average_y)
        est_other[6] = p

    return xy_estimate, est_other






def next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER, n_particles, n_landmarks):
    if not OTHER: # first time calling this function, set up my OTHER variables.
        measurements = [target_measurement]
        hunter_positions = [hunter_position]
        hunter_headings = [hunter_heading]
        OTHER = (measurements, hunter_positions, hunter_headings)
        xy_estimate, est_other = estimate_next_pos(n_particles,n_landmarks, target_measurement, est_other = None)

        #using for PDI control, P
        CTE = 0
        #using for PDI control, I
        int_CTE = 0
        count = 0
        OTHER = (measurements, hunter_positions, hunter_headings, [xy_estimate, est_other],[CTE], [int_CTE], [count]) 
    else: # not the first time, update my history

        """ reference to this: 
        OTHER = (measurements, hunter_positions, hunter_headings, [xy_estimate, est_other],[CTE], [int_CTE], [count])"""

        OTHER[0].append(target_measurement)
        OTHER[1].append(hunter_position)
        OTHER[2].append(hunter_heading)

        xy_estimate, est_other = estimate_next_pos(n_particles,n_landmarks, target_measurement, OTHER[3][1])
        OTHER[3][0] =xy_estimate
        OTHER[3][1] = est_other

    #count trials in OTHER[6][0]
    OTHER[6][0]= OTHER[6][0] + 1

    #when the agent is close enough to catch the target
    if distance_between(hunter_position, xy_estimate) < max_distance:
        heading_to_target = get_heading(hunter_position, xy_estimate)
        heading_difference = angle_trunc(heading_to_target-hunter_heading)
        turning =  heading_difference # turn towards the target
        distance = distance_between(hunter_position, xy_estimate)
    
    #start estimatig a next location of the target by using particle filter    
    elif OTHER[6][0] > 20:
        #using xy_estimate instead of target_measurement
        heading_to_target = get_heading(hunter_position, xy_estimate)
        heading_difference = angle_trunc(heading_to_target-hunter_heading)
        #PID control
        #twiddle [-0.3673, 0.10000000000000002, 0.03]
        #0.68, 34, 50
        # turning =  heading_difference + heading_difference * (-0.36)  + (heading_difference-OTHER[4][0])*0.1 + OTHER[5][0] * 0.03
        turning =  heading_difference + heading_difference * 0.2  + (heading_difference-OTHER[4][0])*0.1 + OTHER[5][0] * 0.03
        #0.8, 40, 50
        distance = distance_between(hunter_position, xy_estimate)

    #just chasing the target, during the first 20 trials 
    else: 
        heading_to_target = get_heading(hunter_position,target_measurement )
        heading_difference = angle_trunc(heading_to_target-hunter_heading)
        turning =  heading_difference 
        distance = max_distance
       
    OTHER[4][0] = heading_difference
    OTHER[5][0] += turning
    return turning, distance, OTHER

n = 50
count = 0
sum_steps = 0

#-------paramators for particle filter 
#number of particles
n_particles = 500
#nuber of landmarks 
n_landmarks = 8

target = robot(0.0, 10.0, 0.0, 2*pi / 30, 1.5)
measurement_noise = 0.05*target.distance # VERY NOISY!!
target.set_noise(0.0, 0.0, measurement_noise)

hunter = robot(-10.0, -10.0, 0.0)

# print demo_grading(hunter, target, naive_next_move)
print demo_grading(hunter, target, next_move, n_particles,n_landmarks, OTHER = None)


