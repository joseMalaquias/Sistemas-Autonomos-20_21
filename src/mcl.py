#!/usr/bin/python

# Libraries
from nav_msgs.srv import GetMap
import rospy
import math
import random
import numpy as np
import tf.transformations
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from sys import argv
import std_msgs.msg
import threading
from multiprocessing import Queue, Process

from scipy import ndimage

import measurePrediction


# ROS messages

# Occupancy Grid Map message


MAP_TOPIC = "static_map"
ODOMETRY_TOPIC = "/odometry/filtered"
SCAN_TOPIC = "/scan"

# https://stackoverflow.com/questions/4877624/numpy-array-of-objects
# class particle():


class particle():
    """ Class responsible for handling the callbacks that come from the
    ROS Subscriptions. This class handles two callbacks, one related to
    the robot's Odometry (/odometry/filtered) and one related to the
    robot's Laser measurement of its surround (/scan)
    """

    def __init__(self):

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.noise_x = 0.0
        self.noise_y = 0.0
        self.noise_yaw = 0.0
        self.v = 0.0
        self.movementDirection = 1
        self.first_msg = True

        # ROS subscriptions
        rospy.Subscriber("/odometry/filtered",
                         Odometry, self.OdometryCallback, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.LaserCallback, queue_size=10)

    def OdometryCallback(self, msg):
        """
        Callback that handles the odometry values
        """

        # First messsage initializes values
        if self.first_msg is True:
            self.old_msg = np.array([])
            self.new_msg = np.array([])
            msgQuaternion = [0, 0, 0, 0]

            # http://wiki.ros.org/tf2/Tutorials/Quaternions#Relative_rotations
            msgQuaternion[0] = msg.pose.pose.orientation.x
            msgQuaternion[1] = msg.pose.pose.orientation.y
            msgQuaternion[2] = msg.pose.pose.orientation.z
            msgQuaternion[3] = msg.pose.pose.orientation.w

            # Stores the value of x, y, and quaternion values from the Husky
            # Odometry ROS topic
            self.new_msg = msg.pose.pose.position.x, \
                msg.pose.pose.position.y, msgQuaternion
            self.first_msg = False

        # Following messages do the calculations
        else:
            msgQuaternion, orientationQuaternion, yawQuaternion = [
                0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]

            # Stores old information and retrieves new one from the topic
            self.old_msg = self.new_msg
            msgQuaternion[0] = msg.pose.pose.orientation.x
            msgQuaternion[1] = msg.pose.pose.orientation.y
            msgQuaternion[2] = msg.pose.pose.orientation.z
            msgQuaternion[3] = msg.pose.pose.orientation.w
            self.new_msg = msg.pose.pose.position.x, \
                msg.pose.pose.position.y, msgQuaternion

            # This variable determines if the robot is moving forward or
            # backwards, since Teleop allows the robot to move backwards
            if msg.twist.twist.linear.x > 0:
                self.movementDirection = 1
            else:
                self.movementDirection = -1

            # Distance traveled from the last message until now
            self.delta_x = self.new_msg[0] - self.old_msg[0]
            self.delta_y = self.new_msg[1] - self.old_msg[1]
            self.delta_yaw = tf.transformations.quaternion_multiply(
                self.new_msg[2],
                tf.transformations.quaternion_inverse(self.old_msg[2]))

            # Adds the distance traveled and rotation from the messages to the
            # variables that store the total distance traveled and rotation
            # while the information isn't retrieved by the program
            orientationQuaternion = tf.transformations.quaternion_from_euler(
                0, 0, self.yaw)
            yawQuaternion = tf.transformations.quaternion_multiply(
                orientationQuaternion, self.delta_yaw)
            self.x += self.delta_x
            self.y += self.delta_y

            # Velocity is calculated in order to later update the particles
            # position
            self.v = math.sqrt(self.x**2 + self.y**2)
            _, _, self.yaw = tf.transformations.euler_from_quaternion(
                yawQuaternion)

    def LaserCallback(self, msg):
        """
        Callback that handles the Laser Scan values
        """

        self.angle_max = msg.angle_max
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment
        self.range_max = msg.range_max
        self.range_min = msg.range_min
        self.ranges = msg.ranges

    def restartMovement(self):
        """
        Function that restarts the values related to the movement information
        """

        self.delta_x = 0.0
        self.delta_y = 0.0
        self.delta_yaw = 0.0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v = 0.0


class particleFilter():

    def __init__(self, NUMBER_OF_PARTICLES, RATE):

        self.numParticles = NUMBER_OF_PARTICLES
        self.rate = RATE
        self.mutex = threading.Lock()

        self.w_avg = 0.0
        self.w_slow = 0.0
        self.w_fast = 0.0
        self.alpha_slow = 0.4
        self.alpha_fast = 0.7

        # https://stackoverflow.com/questions/4877624/numpy-array-of-objects
        # There is no advantage in running a numpy array of
        # objects over iterating all the objects
        # So, numpy is used directly for performance
        # X, Y, Theta and Weight
        # Particle's pose separated from particle's weight for
        # performance (in the multiprocessing)
        self.particlesPose = np.zeros((self.numParticles, 3))
        self.particlesInGrid = np.zeros((self.numParticles, 3))
        # In the beginning, all particles have the same weight
        self.particlesWeight = np.full(
            (self.numParticles), 1.0 / self.numParticles)

        # Sample of Laser Beams evaluated from the original 720 Laser Beams
        # from the Husky's Laser
        self.desired_LaserBeams = 16
        self.predictedRanges = np.zeros(
            shape=(self.numParticles, self.desired_LaserBeams))

        self.map = None
        self.obstacleDistanceMap = None
        self.mapInfo = None

        # Stores the Map information
        self.getMap()
        # Precompute data
        # Creates the map with the euclidean distance from a cell
        # to the closest obstacle
        self.getObstacleDistanceMap()

        # Initialize the particles
        self.initializeParticles()
        # Initialize the ROS subscriptions from the particles() class
        self.callbacks = particle()

        # Waits for a new message from each subscription to be received
        # This prevents the case where the program is started without
        # the proper programs that initialize the topics running
        rospy.wait_for_message(ODOMETRY_TOPIC, Odometry)
        rospy.wait_for_message(SCAN_TOPIC, LaserScan)

        # Publisher
        self.particlePublisher = rospy.Publisher(
            "/mclocalization/particleCloud", PoseArray, queue_size=10)

        # Rate
        rate = rospy.Rate(self.rate)  # Hz

        while not rospy.is_shutdown():
            # Retrieves the information from the subscriptions/callbacks and
            # restarts their values
            self.newMovement()

            # Samples the 720 laser Beams to 16
            self.laserSample()
            self.particleWeight = np.array([])

            # Only updates if there is any movement
            # This is due to the fact that the Odometry Topic always retrieves
            # minimal movement that is insignificant/negligible
            if(self.v >= 0.001 or abs(self.yaw) >= 0.001):
                # Updates all particles movement

                for i in range(self.numParticles):
                    self.movementPrediction(i)

                # Calculates the predicted ranges to each particle using
                # multiprocessing

                self.particlesInGrid[:, 0], self.particlesInGrid[:, 1] = \
                    self.convertToGridArray(
                    self.particlesPose[:, 0], self.particlesPose[:, 1])
                self.particlesInGrid[:, 2] = self.particlesPose[:, 2]

                self.predictedRanges = measurePrediction.measurePrediction(self.particlesInGrid, self.mapInfo.resolution, np.array(
                    (self.callbacks.range_min, self.callbacks.range_max)), self.anglesSampled, self.obstacleDistanceMap, self.numParticles)

                # Calculates the difference between the robot true ranges and
                # the particle predicted ranges

                for i in range(self.numParticles):
                    self.measurePredictionDeviation(self.particlesPose[i], i)

                # Calculates the weights
                self.weightCalculation()
		
                # Resamples the particles

                if self.neff < (2*self.numParticles)/3:
                    self.resampling()

                # Handles the case of particle Deprivation/Kidnapping problem

                self.particleDeprivation()

            # Publishes the particles

            self.publishParticles()

            print("Publishing...\n")

            rate.sleep()

    def newMovement(self):
        """
        Retrieves the information from the subscriptions/callbacks and
        restarts their values
        """
        # Mutex is used to prevent updates in callbacks while retrieving
        # its values
        self.mutex.acquire()
        self.ranges = self.callbacks.ranges
        self.v = self.callbacks.v
        self.yaw = self.callbacks.yaw
        self.movementDirection = self.callbacks.movementDirection
        self.callbacks.restartMovement()
        self.mutex.release()

    def laserSample(self):
        """
        Samples the original 720 Husky's Laser values to 16 total values
        """
        num_LaserBeams = len(self.ranges)
        anglesCovered = np.array([])
        self.anglesSampled = np.array([])
        jump = num_LaserBeams/self.desired_LaserBeams

        # Calculates the angles that will be evaluated
        for i in range(len(self.ranges)):
            angle = self.callbacks.angle_min + i*self.callbacks.angle_increment
            anglesCovered = np.append(anglesCovered, angle)

        self.anglesSampled = anglesCovered[::jump]
        self.rangeSampled = np.array([])

        # Retrieves the Ranges from the angles to be evaluated
        for i in range(0, num_LaserBeams, jump):
            value = self.ranges[-i - 1]

            self.rangeSampled = np.append(self.rangeSampled, value)

    def movementPrediction(self, i):
        """
        Updates particle position based on odometry information
        """

        x_mov = self.v*math.cos(self.particlesPose[i, 2])
        y_mov = self.v*math.sin(self.particlesPose[i, 2])

        # Some noise is added
        # Around 10% on the movement and 5% of the rotation
        noise_x = np.random.normal(
            loc=0, scale=abs(x_mov)*0.1)
        noise_y = np.random.normal(
            loc=0, scale=abs(y_mov)*0.1)
        if(abs(self.yaw)*0.05 != 0):
            noise_yaw = np.random.normal(
                loc=0, scale=abs(self.yaw)*0.05)
        else:
            noise_yaw = 0.00001

        self.particlesPose[i, 0] += self.movementDirection * \
            x_mov + noise_x
        self.particlesPose[i, 1] += self.movementDirection * \
            y_mov + noise_y
        self.particlesPose[i, 2] += self.yaw + noise_yaw

    def measurePredictionDeviation(self, particlePos, index):
        """
        Calculates the difference between the real laser ranges and
        the predicted particle ranges.
        Calculates the weight (not normalized) of each particle.
        """
        rangesDifference = np.array([])
        x, y = self.convertToGrid(particlePos[0], particlePos[1])

        # If particle is out of bounds, attribute a weight of zero
        if(self.map[y, x] == 0):
            self.particleWeight = np.append(
                self.particleWeight, np.exp(-400**2))
            return

        # For all angles, calculate the difference between the ranges
        for i in range(len(self.predictedRanges[index])):
            # in case of the laser value being infinite, do not take
            # this measure into account
            if self.rangeSampled[i] == float("inf"):
                continue
            rangesDifference = np.append(
                rangesDifference, (self.rangeSampled[i] -
                                   self.predictedRanges[index, i]))

        # calculate the norm of those ranges differences
        particleDifference = np.linalg.norm(rangesDifference)
        laserDeviation = 10

        # Calculate the weight (not normalized) of that particle
        weight = (1/(math.sqrt(2*math.pi)*laserDeviation)) * \
            math.exp(-(particleDifference**2)/(2*laserDeviation**2))

        self.particleWeight = np.append(self.particleWeight, weight)

    def weightCalculation(self):
        """
        Normalizes the weights
        """
        totalSum = np.sum(self.particleWeight)
        self.particlesWeight = np.divide(self.particleWeight, totalSum)
        weights_squared = np.sum(np.square(self.particlesWeight))
        self.neff = 1/(weights_squared)
        self.w_avg = np.sum(np.divide(self.particleWeight, self.numParticles))

    def resampling(self):
        """
        Stochastic universal sampling algorithm
        """
        new_particlesPose = np.zeros((self.numParticles, 3))
        new_particlesWeight = np.zeros(self.numParticles)
        r = random.uniform(0, 1.0/self.numParticles)
        c = self.particlesWeight[0]
        i = 0
        j = 0
        for m in range(self.numParticles):
            U = r + float(m)/self.numParticles

            while U > c:
                i = i + 1
                c = c + self.particlesWeight[i]

            new_particlesPose[j] = self.particlesPose[i]
            new_particlesWeight[j] = self.particlesWeight[i]
            j += 1
        self.particlesPose = new_particlesPose
        self.particlesWeight = new_particlesWeight

    def particleDeprivation(self):
        self.w_slow += self.alpha_slow * (self.w_avg - self.w_slow)
        self.w_fast += self.alpha_fast * (self.w_avg - self.w_fast)
        self.deprivation_prob = max(0, 1.0 - self.w_fast / self.w_slow)
        particles_to_replace = []
        if self.deprivation_prob > 0.20:
            self.deprivation_prob = 0.20
        for i in range(int(self.numParticles * self.deprivation_prob)):
            a = random.randint(0, self.numParticles-1)
            while a in particles_to_replace:
                a = random.randint(0, self.numParticles-1)
            particles_to_replace.append(a)
        self.initializeNewParticle(particles_to_replace)

    def getMap(self):
        """
        Gets all the necessary information about the map as well as
        the occupied and free positions on the map
        """

        # http://docs.ros.org/en/diamondback/api/map_server/html/consumer_8py_source.html

        print("Getting occupancy grid map from service...\n")
        rospy.wait_for_service(MAP_TOPIC)
        mapMsg = rospy.ServiceProxy(MAP_TOPIC, GetMap)().map
        print("Got occupancy grid map.\n")
        # https://www.pluralsight.com/guides/different-ways-create-numpy-arrays
        # -1: inexistent, not mapped 0: free, 1-100: chance of obstacle

        print("Creating map...\n")
        map = np.array(mapMsg.data).reshape(
            (mapMsg.info.height, mapMsg.info.width))
        # Used later on to convert between world coordinates and map coordinate
        self.mapInfo = mapMsg.info

        # 0: obstacle 1: free
        self.map = np.zeros(
            (self.mapInfo.height, self.mapInfo.width), dtype=np.bool)

        # https://stackoverflow.com/questions/19766757/replacing-numpy-elements-if-condition-is-met
        self.map[map == 0] = 1

        print("Map created.\n")

        return

    def getObstacleDistanceMap(self):

        print("Getting euclidean distance among positions and closest obstacles...\n")

        # https://docs.scipy.org/doc/scipy-0.14.0/reference/generated/scipy.ndimage.morphology.distance_transform_edt.html

        self.obstacleDistanceMap = ndimage.distance_transform_edt(self.map)

        print("Euclidean distance transform map created.\n")

        return

    def initializeNewParticle(self, particles_to_replace):
        """
        In case of the position estimation being way off track or
        existing a kidnapping situation
        """
        freeSpace = np.where(self.map == 1)
        for i in particles_to_replace:
            # https://thispointer.com/find-the-index-of-a-value-in-numpy-array/

            # https://numpy.org/doc/stable/reference/random/generated/numpy.random.randint.html
            position = np.random.randint(
                0, len(freeSpace[0]), size=1)
            self.particlesPose[i, 0] = freeSpace[1][position]
            self.particlesPose[i, 1] = freeSpace[0][position]

            # https://numpy.org/doc/stable/reference/random/generated/numpy.random.random_sample.html
            # Convert angle to a number between 0 and 2*Pi
            self.particlesPose[i, 2] = np.random.random_sample(
                1) * math.pi * 2.0

            roll, pitch, yaw = tf.transformations.euler_from_quaternion(
                (self.mapInfo.origin.orientation.x,
                 self.mapInfo.origin.orientation.y,
                 self.mapInfo.origin.orientation.z,
                 self.mapInfo.origin.orientation.w))
            # Theta = yaw
            cos = math.cos(yaw)
            sin = math.sin(yaw)

            # Store x values temporarily since it will be changed
            tempX = np.copy(self.particlesPose[i, 0])
            self.particlesPose[i, 0] = (cos*self.particlesPose[i, 0] -
                                        sin*self.particlesPose[i, 1]) * \
                float(self.mapInfo.resolution) + self.mapInfo.origin.position.x

            self.particlesPose[i, 1] = (sin*tempX + cos*self.particlesPose[i, 1]) * \
                float(self.mapInfo.resolution) + self.mapInfo.origin.position.y

            self.particlesPose[i, 2] += yaw

    def initializeParticles(self):
        """
        Initial inicialization of particles
        """

        print("Initializing the particles...\n")

        # https://thispointer.com/find-the-index-of-a-value-in-numpy-array/
        freeSpace = np.where(self.map == 1)

        # https://numpy.org/doc/stable/reference/random/generated/numpy.random.randint.html
        positions = np.random.randint(
            0, len(freeSpace[0]), size=self.numParticles)

        self.particlesPose[:, 0] = freeSpace[1][positions]
        self.particlesPose[:, 1] = freeSpace[0][positions]

        # https://numpy.org/doc/stable/reference/random/generated/numpy.random.random_sample.html
        # Convert angle to a number between 0 and 2*Pi
        self.particlesPose[:, 2] = np.random.random_sample(
            self.numParticles) * math.pi * 2.0

        # Convert map indices to real world coordinates
        self.convertToWorld()
        print("Particles Initialized.\n")

        return

    # https://answers.ros.org/question/10268/where-am-i-in-the-map/?fbclid=IwAR2IQXDmSXQZkBQK4wDJ-243bbHMw-noLNU18-E7FSxcqNfv3yxqESfhDZM
    # https://answers.ros.org/question/161576/solved-getting-map-co-ordinates/?fbclid=IwAR3dcXXAKbBcXLDnB2bRI8pQXnzA_eQKZa9e-ilHuhloPB11PBYSg6zcAu0
    # [x;y] = R * [px * resolution ; py * resolution] + [x0,y0]
    # R = [(cos theta, -sin theta), (sin theta, cos theta)]
    def convertToWorld(self):
        """
        Converts the coordinates of grid map to real world coordinates
        """

        roll, pitch, yaw = tf.transformations.euler_from_quaternion(
            (self.mapInfo.origin.orientation.x,
             self.mapInfo.origin.orientation.y,
             self.mapInfo.origin.orientation.z,
             self.mapInfo.origin.orientation.w))
        # Theta = yaw
        cos = math.cos(yaw)
        sin = math.sin(yaw)

        # Store x values temporarily since it will be changed
        tempX = np.copy(self.particlesPose[:, 0])
        self.particlesPose[:, 0] = (cos*self.particlesPose[:, 0] -
                                    sin*self.particlesPose[:, 1]) * \
            float(self.mapInfo.resolution) + self.mapInfo.origin.position.x

        self.particlesPose[:, 1] = (sin*tempX + cos*self.particlesPose[:, 1]) \
            * float(self.mapInfo.resolution) + self.mapInfo.origin.position.y

        self.particlesPose[:, 2] += yaw

        return

    def convertToGrid(self, x, y):
        """
        Converts from real world coordinates to Grid Map coordinates
        """
        x_grid = int((x - self.mapInfo.origin.position.x) /
                     self.mapInfo.resolution)
        y_grid = int((y - self.mapInfo.origin.position.y) /
                     self.mapInfo.resolution)
        return x_grid, y_grid

    def convertToGridArray(self, x, y):
        """
        Converts an array from real world coordinates to Grid Map coordinates
        """
        x_grid = (x - self.mapInfo.origin.position.x) / self.mapInfo.resolution
        y_grid = (y - self.mapInfo.origin.position.y) / self.mapInfo.resolution

        # x_grid.astype(int)
        # y_grid.astype(int)

        return x_grid, y_grid

    def publishParticles(self):
        """
        Function responsible for publishing the particles
        """

        # http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseArray.html
        # http://docs.ros.org/en/diamondback/api/geometry_msgs/html/msg/PoseArray.html
        # https://answers.ros.org/question/60209/what-is-the-proper-way-to-create-a-header-with-python/
        # https://www.programcreek.com/python/example/86351/std_msgs.msg.Header

        particleCloud = PoseArray()

        # Create PoseArray header
        particleCloud.header = std_msgs.msg.Header()
        particleCloud.header.stamp = rospy.Time.now()
        particleCloud.header.frame_id = "map"

        particlePoses = []
        for i in range(self.numParticles):
            pose = Pose()
            pose.position.x = self.particlesPose[i, 0]
            pose.position.y = self.particlesPose[i, 1]

            # https://answers.ros.org/question/69754/quaternion-transformations-in-python/
            quaternion = tf.transformations.quaternion_from_euler(
                0, 0, self.particlesPose[i, 2])

            pose.orientation.x = quaternion[0]
            pose.orientation.y = quaternion[1]
            pose.orientation.z = quaternion[2]
            pose.orientation.w = quaternion[3]

            particlePoses.append(pose)

        particleCloud.poses = particlePoses

        self.particlePublisher.publish(particleCloud)


if __name__ == "__main__":
    rospy.init_node("mclocalization")
    if(len(argv) == 1):
        numParticles = 100
        rate = 1
    elif(len(argv) == 2):
        numParticles = int(argv[1])
        rate = 1
    else:
        numParticles = int(argv[1])
        rate = float(argv[2])

    pf = particleFilter(numParticles, rate)
    rospy.spin()
