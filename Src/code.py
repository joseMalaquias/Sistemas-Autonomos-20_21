# Libraries
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


# ROS messages

# Occupancy Grid Map message
from nav_msgs.srv import GetMap


MAP_TOPIC = "static_map"
ODOMETRY_TOPIC = "/odometry/filtered"
SCAN_TOPIC = "/scan"

# https://stackoverflow.com/questions/4877624/numpy-array-of-objects
# class particle():


class particle():

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
        rospy.Subscriber("/odometry/filtered",
                         Odometry, self.OdometryCallback, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.LaserCallback, queue_size=10)

    def OdometryCallback(self, msg):
        if self.first_msg is True:
            self.old_msg = np.array([])
            self.new_msg = np.array([])
            msgQuaternion = [0, 0, 0, 0]

            # http://wiki.ros.org/tf2/Tutorials/Quaternions#Relative_rotations
            msgQuaternion[0] = msg.pose.pose.orientation.x
            msgQuaternion[1] = msg.pose.pose.orientation.y
            msgQuaternion[2] = msg.pose.pose.orientation.z
            # Negate for inverse
            msgQuaternion[3] = msg.pose.pose.orientation.w
            self.new_msg = msg.pose.pose.position.x, \
                msg.pose.pose.position.y, msgQuaternion
            self.first_msg = False
        else:
            msgQuaternion, orientationQuaternion, yawQuaternion = [
                0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]

            self.old_msg = self.new_msg
            msgQuaternion[0] = msg.pose.pose.orientation.x
            msgQuaternion[1] = msg.pose.pose.orientation.y
            msgQuaternion[2] = msg.pose.pose.orientation.z
            msgQuaternion[3] = msg.pose.pose.orientation.w
            self.new_msg = msg.pose.pose.position.x, \
                msg.pose.pose.position.y, msgQuaternion
            if msg.twist.twist.linear.x > 0:
                self.movementDirection = 1
            else:
                self.movementDirection = -1

            # 2% of traveled distance is added as noise
            self.delta_x = self.new_msg[0] - self.old_msg[0]
            self.delta_y = self.new_msg[1] - self.old_msg[1]

            # difference between old msg yaw and new msg yaw
            self.delta_yaw = tf.transformations.quaternion_multiply(
                self.new_msg[2],
                tf.transformations.quaternion_inverse(self.old_msg[2]))

            # yaw from calculated values

            # Distance traveled is calculated
            orientationQuaternion = tf.transformations.quaternion_from_euler(
                0, 0, self.yaw)
            yawQuaternion = tf.transformations.quaternion_multiply(
                orientationQuaternion, self.delta_yaw)
            self.x += self.delta_x
            self.y += self.delta_y
            self.v = math.sqrt(self.x**2 + self.y**2)
            _, _, self.yaw = tf.transformations.euler_from_quaternion(
                yawQuaternion)

    def LaserCallback(self, msg):
        self.angle_max = msg.angle_max
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment
        self.range_max = msg.range_max
        self.range_min = msg.range_min
        self.ranges = msg.ranges

    def restartMovement(self):
        self.delta_x = 0.0
        self.delta_y = 0.0
        self.delta_yaw = 0.0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v = 0.0


class particleFilter():

    def __init__(self, NUMBER_OF_PARTICLES):
        self.numParticles = NUMBER_OF_PARTICLES

        self.mutex = threading.Lock()

        # https://stackoverflow.com/questions/4877624/numpy-array-of-objects
        # There is no advantage in running a numpy array of
        # objects over iterating all the objects
        # So, numpy is used directly for performance
        # X, Y, Theta and Weight
        self.particles = np.zeros((self.numParticles, 4))
        # In the beginning, all particles have the same weight
        self.particles[:, 3] = 1.0 / self.numParticles

        self.map = None
        self.mapInfo = None

        self.getMap()
        self.initializeParticles()
        self.callbacks = particle()
        rospy.wait_for_message(ODOMETRY_TOPIC, Odometry)
        rospy.wait_for_message(SCAN_TOPIC, LaserScan)

        # Publisher
        # Publisher needs a queue_size, see if that matters
        self.particlePublisher = rospy.Publisher(
            "/MCLocalization/particleCloud", PoseArray, queue_size=1)

        # JUST FOR TEST FOR NOW
        # https://answers.ros.org/question/316829/how-to-publish-a-pose-in-quaternion/
        rate = rospy.Rate(2)  # Hz

        while not rospy.is_shutdown():
            # _, _, yaw = tf.transformations.euler_from_quaternion([
            #    0.0, 0.0, -0.000108081936731, 0.999999994159])
            self.newMovement()
            self.laserSample()
            self.particleWeight = np.array([])
            self.particleDifference = np.array([])
            # self.particles[0, 0] = 0.000605634648655
            # self.particles[0, 1] = 0.0
            # self.particles[0, 2] = yaw
            # self.particles

            if(self.v >= 0.001 or abs(self.yaw) >= 0.001):
                for i in range(self.numParticles):
                    self.movementPrediction(i)
                    self.measurePrediction(self.particles[i])
                    self.measurePredictionDeviation(self.particles[i])
                self.weightCalculation()
                self.resampling()
                self.particleDeprivation()
            self.publishParticles()
            print("Publishing...\n")
            rate.sleep()

    def particleDeprivation(self):
        for i in range(self.numParticles):
            if self.particleDifference[i] >= \
                    np.linalg.norm([7]*self.desired_LaserBeams):
                self.initializeNewParticle(i)

    def newMovement(self):
        self.mutex.acquire()
        self.ranges = self.callbacks.ranges
        self.v = self.callbacks.v
        self.yaw = self.callbacks.yaw
        self.movementDirection = self.callbacks.movementDirection
        self.callbacks.restartMovement()
        self.mutex.release()

    def resampling(self):
        new_particles = np.zeros((self.numParticles, 4))

        r = random.uniform(0, 1.0/self.numParticles)
        c = self.particles[0][3]
        i = 0
        j = 0
        for m in range(self.numParticles):
            U = r + float(m)/self.numParticles

            while U > c:
                i = i + 1
                c = c + self.particles[i][3]

            new_particles[j] = self.particles[i]
            j += 1
        self.particles = new_particles

    def weightCalculation(self):
        totalSum = sum(self.particleWeight)
        for i in range(self.numParticles):
            self.particles[i, 3] = self.particleWeight[i] / totalSum

    def measurePredictionDeviation(self, particlePos):
        rangesDifference = np.array([])
        x, y = self.convertToGrid(particlePos[0], particlePos[1])
        if(self.map[x, y] == 0):
            self.particleWeight = np.append(
                self.particleWeight, np.exp(-400**2))
            self.particleDifference = np.append(
                self.particleDifference, self.desired_LaserBeams * 1000)
            return
        for i in range(len(self.predictedRanges)):
            if self.rangeSampled[i] == float("inf"):
                continue
            rangesDifference = np.append(
                rangesDifference, (self.rangeSampled[i] -
                                   self.predictedRanges[i]))

        particleDifference_aux = np.linalg.norm(rangesDifference)
        self.particleDifference = np.append(
            self.particleDifference, particleDifference_aux)
        laserDeviation = 100

        weight = (1/(math.sqrt(2*math.pi)*laserDeviation)) * \
            math.exp(-(particleDifference_aux**2)/(2*laserDeviation**2))

        self.particleWeight = np.append(self.particleWeight, weight)

    def laserSample(self):

        num_LaserBeams = len(self.ranges)
        self.desired_LaserBeams = 16
        anglesCovered = np.array([])
        self.anglesSampled = np.array([])
        jump = num_LaserBeams/self.desired_LaserBeams
        for i in range(len(self.ranges)):
            angle = self.callbacks.angle_min + i*self.callbacks.angle_increment
            anglesCovered = np.append(anglesCovered, angle)

        self.anglesSampled = anglesCovered[::jump]
        self.rangeSampled = np.array([])
        for i in range(0, num_LaserBeams, jump):
            value = self.ranges[-i - 1]

            self.rangeSampled = np.append(self.rangeSampled, value)

    def movementPrediction(self, i):

        x_mov = self.v*math.cos(self.particles[i, 2])
        y_mov = self.v*math.sin(self.particles[i, 2])
        noise_x = np.random.normal(
            loc=0, scale=abs(x_mov)*0.1)
        noise_y = np.random.normal(
            loc=0, scale=abs(y_mov)*0.1)
        noise_yaw = np.random.normal(
            loc=0, scale=abs(self.yaw)*0.05)

        self.particles[i, 0] += self.movementDirection * \
            x_mov + noise_x
        self.particles[i, 1] += self.movementDirection * \
            y_mov + noise_y
        self.particles[i, 2] += self.yaw + noise_yaw

    def measurePrediction(self, particlePosition):

        self.predictedRanges = np.array([])
        currentAngle = self.callbacks.angle_min
        particleOrientation = tf.transformations.quaternion_from_euler(
            0, 0, particlePosition[2])
        for currentAngle in self.anglesSampled:

            laserOrientation = tf.transformations.quaternion_from_euler(
                0, 0, currentAngle)
            _, _, laserRotation = tf.transformations.euler_from_quaternion(
                tf.transformations.quaternion_multiply(particleOrientation,
                                                       laserOrientation))
            currentRange = self.callbacks.range_min
            while(currentRange <= self.callbacks.range_max):
                x_laser = currentRange * \
                    math.cos(laserRotation)
                y_laser = currentRange * \
                    math.sin(laserRotation)
                x_predicted = particlePosition[0] + x_laser
                y_predicted = particlePosition[1] + y_laser
                x_predicted_grid, y_predicted_grid = self.convertToGrid(
                    x_predicted, y_predicted)
                if(self.map[y_predicted_grid, x_predicted_grid] == 0):
                    break
                currentRange += self.mapInfo.resolution
            if(currentRange > self.callbacks.range_max):
                currentRange = self.callbacks.range_max
            elif (currentRange < self.callbacks.range_min):
                currentRange = self.callbacks.range_min

            self.predictedRanges = np.append(
                self.predictedRanges, currentRange)

    def getMap(self):

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
            (self.mapInfo.height, self.mapInfo.width), dtype=bool)

        # https://stackoverflow.com/questions/19766757/replacing-numpy-elements-if-condition-is-met
        self.map[map == 0] = 1

        print("Map created\n")

        return

    def initializeNewParticle(self, i):

        # https://thispointer.com/find-the-index-of-a-value-in-numpy-array/
        freeSpace = np.where(self.map == 1)

        # https://numpy.org/doc/stable/reference/random/generated/numpy.random.randint.html
        position = np.random.randint(
            0, len(freeSpace[0]), size=1)

        self.particles[i, 0] = freeSpace[1][position]
        self.particles[i, 1] = freeSpace[0][position]

        # https://numpy.org/doc/stable/reference/random/generated/numpy.random.random_sample.html
        # Convert angle to a number between 0 and 2*Pi
        self.particles[i, 2] = np.random.random_sample(1) * math.pi * 2.0

        roll, pitch, yaw = tf.transformations.euler_from_quaternion(
            (self.mapInfo.origin.orientation.x,
             self.mapInfo.origin.orientation.y,
             self.mapInfo.origin.orientation.z,
             self.mapInfo.origin.orientation.w))
        # Theta = yaw
        cos = math.cos(yaw)
        sin = math.sin(yaw)

        # Store x values temporarily since it will be changed
        tempX = np.copy(self.particles[i, 0])
        self.particles[i, 0] = (cos*self.particles[i, 0] -
                                sin*self.particles[i, 1]) * \
            float(self.mapInfo.resolution) + self.mapInfo.origin.position.x

        self.particles[i, 1] = (sin*tempX + cos*self.particles[i, 1]) * \
            float(self.mapInfo.resolution) + self.mapInfo.origin.position.y

        self.particles[i, 2] += yaw

    def initializeParticles(self):

        print("Initializing the particles...\n")

        # https://thispointer.com/find-the-index-of-a-value-in-numpy-array/
        freeSpace = np.where(self.map == 1)

        # https://numpy.org/doc/stable/reference/random/generated/numpy.random.randint.html
        positions = np.random.randint(
            0, len(freeSpace[0]), size=self.numParticles)

        self.particles[:, 0] = freeSpace[1][positions]
        self.particles[:, 1] = freeSpace[0][positions]
        # _, _, yaw = tf.transformations.euler_from_quaternion(
        #    [0.0, 0.0, 0.00003821815, 0.99999999927])

        # for i in range(self.numParticles):
        #    self.particles[i, 0] = np.random.normal(loc=0, scale=1)
        #    self.particles[i, 1] = np.random.normal(loc=0, scale=1)
        #    self.particles[:, 2] = yaw + np.random.normal(loc=0, scale=0.01)

        # https://numpy.org/doc/stable/reference/random/generated/numpy.random.random_sample.html
        # Convert angle to a number between 0 and 2*Pi
        self.particles[:, 2] = np.random.random_sample(
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

        roll, pitch, yaw = tf.transformations.euler_from_quaternion(
            (self.mapInfo.origin.orientation.x,
             self.mapInfo.origin.orientation.y,
             self.mapInfo.origin.orientation.z,
             self.mapInfo.origin.orientation.w))
        # Theta = yaw
        cos = math.cos(yaw)
        sin = math.sin(yaw)

        # Store x values temporarily since it will be changed
        tempX = np.copy(self.particles[:, 0])
        self.particles[:, 0] = (cos*self.particles[:, 0] -
                                sin*self.particles[:, 1]) * \
            float(self.mapInfo.resolution) + self.mapInfo.origin.position.x

        self.particles[:, 1] = (sin*tempX + cos*self.particles[:, 1]) * \
            float(self.mapInfo.resolution) + self.mapInfo.origin.position.y

        self.particles[:, 2] += yaw

        return

    def convertToGrid(self, x, y):
        x_grid = int((x - self.mapInfo.origin.position.x) /
                     self.mapInfo.resolution)
        y_grid = int((y - self.mapInfo.origin.position.y) /
                     self.mapInfo.resolution)
        return x_grid, y_grid

    def publishParticles(self):
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
            pose.position.x = self.particles[i, 0]
            pose.position.y = self.particles[i, 1]

            # https://answers.ros.org/question/69754/quaternion-transformations-in-python/
            quaternion = tf.transformations.quaternion_from_euler(
                0, 0, self.particles[i, 2])
            pose.orientation.x = quaternion[0]
            pose.orientation.y = quaternion[1]
            pose.orientation.z = quaternion[2]
            pose.orientation.w = quaternion[3]

            particlePoses.append(pose)

        particleCloud.poses = particlePoses

        self.particlePublisher.publish(particleCloud)


if __name__ == "__main__":
    rospy.init_node("MCLocalization")
    if(len(argv) >= 1):
        numParticles = int(argv[1])
    else:
        numParticles = 100

    pf = particleFilter(numParticles)
    rospy.spin()
