"""
 === Introduction ===

   Part A:
        Create a SLAM implementation to process a series of landmark measurements and movement updates.

    Part B:
        The rover does unfortunately has a series of limitations:

        - Start position
          - The rover will land somewhere within range of at least 3 or more satellites for measurements.

        - Measurements
          - Measurements will come from satellites and test sites within range of the rover's antenna horizon.
            * The format is {'landmark id':{'distance':0.0, 'bearing':0.0, 'type':'satellite'}, ...}
          - Satellites and test sites will always return a measurement if in range.

        - Movements
          - Action: 'move 1.570963 1.0'
            * The rover will turn counterclockwise 90 degrees and then move 1.0
          - stochastic due to the icy surface.
          - if max distance or steering is exceeded, the rover will not move.

        - Samples
          - Provided as list of x and y coordinates, [[0., 0.], [1., -3.5], ...]
          - Action: 'sample'
            * The rover will attempt to take a sample at the current location.
          - A rover can only take a sample once per requested site.
          - The rover must be with 0.25 distance to successfully take a sample.
            * Hint: Be sure to account for floating point limitations
          - The is a 100ms penalty if the robot is requested to sample a site not on the list or if the site has
            previously been sampled.
          - Use sys.stdout = open('stdout.txt', 'w') to directly print data if necessary.         

        The rover will always execute a measurement first, followed by an action.

        The rover will have a time limit of 10 seconds to find and sample all required sites.
"""

import robot
from math import *
import math
import random
import matrix

PI = math.pi
NOISE_FLAG = True
NOISE_MOVE = 0.01


class SLAM:
    """Create a basic SLAM module.
    """
    def __init__(self):
        """Initialize SLAM components here.
        """
        self.steering = 0
        self.distance = 0
        self.orientation = 0
        self.x = 0
        self.y = 0
        self.dx = 0
        self.dy = 0
        self.data = []
        self.init_x = 0
        self.init_y = 0

    def process_measurements(self, measurements):
        """Process a new series of measurements.

        Args:
            measurements(dict): Collection of measurements
                in the format {'landmark id':{'distance':0.0, 'bearing':0.0, 'type':'satellite'}, ...}

        Returns:
            x, y: current belief in location
        """
        num_landmarks = len(measurements)
        newkey = 0
        for keys, values in measurements.items():
            measurements[str(newkey)] = measurements.pop(keys)
            newkey += 1
        Z = []
        for keys, values in measurements.items():
            distance = measurements[keys]["distance"]
            bearing = measurements[keys]["bearing"]
            dx = distance * cos(bearing + self.orientation)
            dy = distance * sin(bearing + self.orientation)
            Z.append([int(keys), dx, dy])
       
        
        data = []
        data.append([Z, [self.dx, self.dy]])
        result = online_slam(self.init_x, self.init_y, data, 1, num_landmarks)
        x = result.value[0][0]
        y = result.value[1][0]
        self.init_x = x
        self.init_y = y

        return x, y
    


    def process_movement(self, steering, distance,motion_noise = 0.01):
        """Process a new movement.

        Args:
            steering(float): amount to turn
            distance(float): distance to move
            motion_noise(float): movement noise

        Returns:
            x, y: current belief in location
        """
        self.steering = steering + random.uniform((-1) * motion_noise, motion_noise)
        self.distance = distance * random.uniform((1 - motion_noise), (1 + motion_noise))
        

        self.orientation = robot.truncate_angle(self.orientation + float(steering))
        self.dx = distance * math.cos(self.orientation)
        self.dy = distance * math.sin(self.orientation)
        self.x += self.dx
        self.y += self.dy


        x = self.x
        y = self.y


        return x, y

    def rand(self):
        return random.random() * 2.0 - 1.0

    def generate_measurements(self, noise=NOISE_FLAG, horizon_distance = 3.0):
        """Generate measurements of landmarks within range.

        Args:
            noise(bool): Move with noise if True.
                Default: NOISE_FLAG

        Returns:
            Measurements to landmarks in the format:
                {'landmark id':{'distance':0.0, 'heading':0.0}, ...}
        """
        measurements = dict()

        # process landmarks
        for location in self.landmarks:
            distance, bearing = self.robot.measure_distance_and_bearing_to((location['x'], location['y']), noise=noise)

            if distance <= horizon_distance:
                measurements[int(hashlib.md5(str(location) + HASH_SEED).hexdigest(), 16)] = {'distance': distance,
                                                                                             'bearing': bearing,
                                                                                             'type': 'beacon'}
           

        return measurements

    

class WayPointPlanner:
    """Create a planner to navigate the rover to reach all the intended way points from an unknown start position.
    """
    def __init__(self,  max_distance, max_steering):
        """Initialize your planner here.

        Args:
            max_distance(float): the max distance the robot can travel in a single move.
            max_steering(float): the max steering angle the robot can turn in a single move.
        """
        self.max_distance = max_distance
        self.max_steering = max_steering
        self.orientation = 0
        self.target = 0
        self.spiral_leg = 1
        self.count = 0
        self.samples = []
        self.found = 0   # found 1st sample
        self.samplex = 0
        self.sampley = 0
        self.nextsite = ()
        

    def next_move(self, sample_todo, measurements):
        """Next move based on the current set of measurements.

        Args:
            sample_todo(list): Set of locations remaining still needing a sample to be taken.
            measurements(dict): Collection of measurements from satellites and test sites in range.
                in the format {'landmark id':{'distance':0.0, 'bearing':0.0, 'type':'satellite'}, ...}

        Return:
            Next command to execute on the rover.
                allowed:
                    'move 1.570963 1.0' - turn left 90 degrees and move 1.0 distance
                    'take sample' - take sample (will succeed if within tolerance of intended sample site)
        """
        if len(self.samples) == 0:
            self.samples = sample_todo[:]         

        if self.found == 0:
            if len(sample_todo) == (len(self.samples) - 1):
                samplesite = set(self.samples) - set(sample_todo)
                samplesite = list(samplesite)
                self.samplex = samplesite[0][0]
                self.sampley = samplesite[0][1]
                self.found += 1
                

        sites = {}
        for keys, values in measurements.items():

            if measurements[keys]["type"] == "site":
                if len(sites) == 0:
                    sites[keys] = measurements[keys]
                else:
                    if measurements[keys]["distance"] < sites.values()[0]["distance"]:
                        sites = {}
                        sites[keys] = measurements[keys]
        
        if (self.target) == 0 and len(sites) > 0:
            keys = sites.keys()[0]
            self.target = keys
        
        if self.target > 0:
            distance = measurements[self.target]["distance"]
            bearing = measurements[self.target]["bearing"]
            if distance < 0.2:
                action = "sample"
                #print action
                self.target = 0
                if len(self.nextsite)>0:
                    self.samplex,self.sampley = self.nextsite
                self.nextsite = ()
                return action

            if self.check_distance_ok(distance) and self.check_steering_ok(bearing):
                action = 'move ' + str(bearing) + ' ' + str(distance)
                self.orientation = robot.truncate_angle(self.orientation + float(bearing))
                #print action
                return action
            else:
                if (bearing > self.max_steering):
                    action = 'move ' + str(self.max_steering) + ' ' + str(0.0)
                    #print action
                    self.orientation = robot.truncate_angle(self.orientation + float(self.max_steering))
                    return action
                if (bearing < -self.max_steering):
                    action = 'move ' + str(-self.max_steering) + ' ' + str(0.0)
                    self.orientation = robot.truncate_angle(self.orientation + float(-self.max_steering))
                    #print action
                    return action
                if (distance > self.max_distance):
                    action = 'move ' + str(bearing) + ' ' + str(self.max_distance)
                    self.orientation = robot.truncate_angle(self.orientation + float(bearing))
                    #print action
                    return action





        if self.found == 0:   # firstly do spiral exploration until find 1st site
            if self.count == self.spiral_leg:
                steering = self.max_steering
                self.count = 0
                self.spiral_leg += 1
            else:
                steering = 0

            action = 'move ' + str(steering) + ' ' + str(self.max_distance)
            self.orientation = robot.truncate_angle(self.orientation + float(steering))
            self.count += 1
        else:
            if len(self.nextsite) == 0:
                nextdistance = 10000
                for i in range(len(sample_todo)):
                    distance = robot.compute_distance((self.samplex, self.sampley), sample_todo[i])
                    if distance < nextdistance:
                        nextdistance = distance
                        self.nextsite = sample_todo[i]
                        nextbearing = robot.compute_bearing((self.samplex, self.sampley), self.nextsite)
            nextbearing = robot.compute_bearing((self.samplex, self.sampley), self.nextsite)
            nextdistance = robot.compute_distance((self.samplex, self.sampley), self.nextsite)
            nextsteering = nextbearing - self.orientation
            if self.check_distance_ok(nextdistance) and self.check_steering_ok(nextsteering):
                action = 'move ' + str(nextsteering) + ' ' + str(nextdistance)
                rover_slam = SLAM()
                rover_slam.orientation = self.orientation
                x, y = rover_slam.process_movement(nextsteering, nextdistance)
                self.samplex  += x
                self.sampley  += y
                self.orientation = robot.truncate_angle(self.orientation + float(nextsteering))
                return action
            else:
                if (nextsteering > self.max_steering):
                    action = 'move ' + str(self.max_steering) + ' ' + str(0.0)
                    #print action
                    self.orientation = robot.truncate_angle(self.orientation + float(self.max_steering))
                    return action
                if (nextsteering < -self.max_steering):
                    action = 'move ' + str(-self.max_steering) + ' ' + str(0.0)
                    #print action
                    self.orientation = robot.truncate_angle(self.orientation + float(-self.max_steering))
                    return action
                if (nextdistance > self.max_distance):
                    action = 'move ' + str(nextsteering) + ' ' + str(self.max_distance)
                    #print action
                    rover_slam = SLAM()
                    rover_slam.orientation = self.orientation
                    x, y = rover_slam.process_movement(nextsteering, self.max_distance)
                    self.samplex  += x
                    self.sampley  += y
                    #print x, y
                    #print self.samplex, self.sampley
                    self.orientation = robot.truncate_angle(self.orientation + float(nextsteering))
                    #print self.orientation
                    return action

                


        return action

    def check_distance_ok(self, distance):
        return 0.0 <= distance <= self.max_distance

    def check_steering_ok(self, steering):
        return -self.max_steering <= steering <= self.max_steering

def online_slam(x, y, data,  N, num_landmarks, motion_noise=1.0, measurement_noise = 1.0):
    # Set the dimension of the filter
    dim = 2 * (1 + num_landmarks) 

    # make the constraint information matrix and vector
    Omega = matrix.matrix()
    Omega.zero(dim, dim)
    Omega.value[0][0] = 1.0
    Omega.value[1][1] = 1.0

    Xi = matrix.matrix()
    Xi.zero(dim, 1)
    Xi.value[0][0] = x
    Xi.value[1][0] = y
    
    # process the data

    for k in range(len(data)):

    
        measurement = data[k][0]
        motion      = data[k][1]
    
        # integrate the measurements
        for i in range(len(measurement)):
    
            # m is the index of the landmark coordinate in the matrix/vector
            m = 2 * (1 + measurement[i][0])
    
            # update the information maxtrix/vector based on the measurement
            for b in range(2):
                Omega.value[b][b] +=  1.0 / measurement_noise
                Omega.value[m+b][m+b] +=  1.0 / measurement_noise
                Omega.value[b][m+b] += -1.0 / measurement_noise
                Omega.value[m+b][b] += -1.0 / measurement_noise
                Xi.value[b][0]      += -measurement[i][1+b] / measurement_noise
                Xi.value[m+b][0]      +=  measurement[i][1+b] / measurement_noise

        list = [0,1] + range(4, dim +2)
        Omega = Omega.expand(dim + 2, dim+2, list, list)
        Xi = Xi.expand(dim+2, 1, list, [0])

        # update the information maxtrix/vector based on the robot motion
        for b in range(4):
            Omega.value[b][b] +=  1.0 / motion_noise
        for b in range(2):
            Omega.value[b  ][b+2] += -1.0 / motion_noise
            Omega.value[b+2][b  ] += -1.0 / motion_noise
            Xi.value[b  ][0]        += -motion[b] / motion_noise
            Xi.value[b+2][0]        +=  motion[b] / motion_noise

        newlist = range(2, len(Omega.value))
        a = Omega.take([0,1], newlist)
        b = Omega.take([0,1])
        c = Xi.take([0,1],[0])
        Omega = Omega.take(newlist) - a.transpose() * b.inverse() * a
        Xi = Xi.take(newlist, [0]) - a.transpose() * b.inverse() * c

    # compute best estimate
    mu = Omega.inverse() * Xi

    # return the result
    return mu # make sure you return both of these matrices to be marked correct.

def make_landmarks(world_size, num):
    return [(rnd.uniform(-world_size, world_size), rnd.uniform(-world_size, world_size)) for _ in range(num)]


def add_measurement(measurement, landmark, distance, bearing):
    measurement[int(hashlib.md5(str(landmark) + 'hash seed').hexdigest(), 16)] = {
        'distance': distance,
        'bearing': bearing,
        'type': 'beacon'
    }


def make_data(time_steps=20, num_landmarks=5, world_size=100, measurement_range=50, max_distance=20,
              motion_noise=.01):
    world_size /= 2.  # the world spans from -world_size/2 to world_size/2

    while True:
        measure_motions = []

        # make robot and landmarks
        r = Robot(max_distance=max_distance)
        landmarks = make_landmarks(world_size, num_landmarks)
        seen = [False] * num_landmarks

        for k in range(time_steps):
            # sense
            measurement = {}
            for i, landmark in enumerate(landmarks):
                distance, bearing = r.measure_distance_and_bearing_to(landmark, noise=True)
                if distance < measurement_range:
                    add_measurement(measurement, landmark, distance, bearing)
                    seen[i] = True

            # move
            x = r.x
            y = r.y
            bearing = r.bearing
            while True:
                steering = rnd.uniform(-r.max_steering, r.max_steering)
                distance = rnd.uniform(0, max_distance)
                r.move(steering, distance)
                if -world_size <= r.x <= world_size and -world_size <= r.y <= world_size:
                    steering += rnd.uniform(-motion_noise, motion_noise)
                    distance *= rnd.uniform(1. - motion_noise, 1. + motion_noise)
                    measure_motions.append((measurement, (steering, distance)))
                    break
                # if we'd be leaving the robot world, pick instead a new direction
                r.x = x
                r.y = y
                r.bearing = bearing

        # we are done when all landmarks were observed; otherwise re-run
        if sum(seen) == num_landmarks:
            return measure_motions, r.x, r.y


def is_close(a, b, tol=2):
    return abs(a - b) < tol

