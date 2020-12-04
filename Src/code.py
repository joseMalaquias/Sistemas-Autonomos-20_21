# Libraries
import rospy
import copy
import math
import numpy as np
import tf.transformations
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from sys import argv
import std_msgs.msg

# ROS messages

# Occupancy Grid Map message
from nav_msgs.srv import GetMap


MAP_TOPIC = "static_map"
ODOMETRY_TOPIC = "/husky_velocity_controller/odom"
SCAN_TOPIC = "/scan"

# https://stackoverflow.com/questions/4877624/numpy-array-of-objects
# class particle():


class particle():

    def __init__(self):

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.first_msg = True
        rospy.Subscriber("/husky_velocity_controller/odom",
                         Odometry, self.OdometryCallback, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.LaserCallback, queue_size=10)

    def OdometryCallback(self, msg):
        if self.first_msg is True:
            self.old_msg = []
            self.new_msg = []
            self.new_msg = msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.w
            self.first_msg = False
        else:
            self.old_msg = copy.deepcopy(self.new_msg)
            self.new_msg = msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.w
            # 2% of traveled distance is added as noise
            self.real_delta_x = self.new_msg[0] - self.old_msg[0]
            self.real_delta_y = self.new_msg[1] - self.old_msg[1]
            self.real_delta_yaw = self.new_msg[2] - self.old_msg[2]
            self.noise_x = np.random.normal(
                loc=0, scale=abs(self.real_delta_x)*0.02)
            self.noise_y = np.random.normal(
                loc=0, scale=abs(self.real_delta_y)*0.02)
            self.noise_yaw = np.random.normal(
                loc=0, scale=abs(self.real_delta_yaw)*0.02)
            # Distance traveled is calculated
            self.delta_x = self.real_delta_x + self.noise_x
            self.delta_y = self.real_delta_y + self.noise_y
            self.delta_theta = self.real_delta_yaw + self.noise_yaw
            self.moveParticle(self.delta_x, self.delta_y, self.delta_theta)

    def LaserCallback(self, msg):
        self.angle_max = msg.angle_max
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment
        self.range_max = msg.range_max
        self.range_min = msg.range_min
        self.ranges = msg.ranges

    def moveParticle(self, x, y, theta):
        self.x = self.x + x
        self.y = self.y + y
        self.yaw = self.yaw + theta

    def restartMovement(self):
        self.real_delta_x = 0.0
        self.real_delta_y = 0.0
        self.real_delta_yaw = 0.0
        self.delta_x = 0.0
        self.delta_y = 0.0
        self.delta_yaw = 0.0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0


class particleFilter():

    def __init__(self, NUMBER_OF_PARTICLES):
        self.numParticles = NUMBER_OF_PARTICLES

        # https://stackoverflow.com/questions/4877624/numpy-array-of-objects
        # There is no advantage in running a numpy array of objects over iterating all the objects
        # So, numpy is used directly for performance
        # X, Y, Theta and Weight
        self.particles = np.zeros((self.numParticles, 4))
        # In the beginning, all particles have the same weight
        self.particles[:, 3] = 1.0 / self.numParticles

        self.map = None
        self.mapInfo = None

        # self.i = 0

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
        rate = rospy.Rate(1)  # Hz
        while not rospy.is_shutdown():
            self.laserSample()
            self.weights, self.err = [], []
            for i in range(self.numParticles):
                self.movementPrediction(i)
                self.measurePrediction(self.particles[i])
                self.measurePredictionDeviation()
            self.weighCalculation()
            self.resampling()
            self.publishParticles()
            # print(self.weights)
            # print(sum(self.weights))
            print("Publishing...\n")
            self.callbacks.restartMovement()
            rate.sleep()

    def resampling(self):
        raise "NotImplementedError"

    def weighCalculation(self):
        total = sum(self.err)
        for i in range(len(self.err)):
            self.weights.append(self.err[i] / total)

    def measurePredictionDeviation(self):
        deltas = []
        for i in range(len(self.predictedRanges)):
            deltas.append((self.rangeSampled[i] - self.predictedRanges[i])**2)
        self.err.append(np.linalg.norm(deltas)**2)

    def laserSample(self):
        anglesCovered = []
        num_LaserBeams = len(self.callbacks.ranges)
        desired_LaserBeams = 20
        jump = num_LaserBeams/desired_LaserBeams
        for i in range(len(self.callbacks.ranges)):
            angle = self.callbacks.angle_min + i*self.callbacks.angle_increment
            anglesCovered.append(angle)

        self.anglesSampled = anglesCovered[::jump]
        self.rangeSampled = []
        for i in range(0, num_LaserBeams, jump):
            if self.callbacks.ranges[i] == float("inf"):
                # print("entra")
                value = self.callbacks.range_max
            else:
                # print(type(self.callbacks.ranges[i]))
                value = self.callbacks.ranges[i]
            self.rangeSampled.append(value)
        self.aux = self.callbacks.ranges[::jump]
        # print("484", self.aux, "-------------", self.rangeSampled, "4949")

    def movementPrediction(self, i):
        self.particles[i, 0] += self.callbacks.x
        self.particles[i, 1] += self.callbacks.y
        self.particles[i, 2] += self.callbacks.yaw

    def measurePrediction(self, particlePosition):

        self.predictedRanges = []
        currentAngle = self.callbacks.angle_min
        for currentAngle in self.anglesSampled:

            currentRange = self.callbacks.range_min
            while(currentRange <= self.callbacks.range_max):
                x_laser = currentRange * \
                    math.cos(particlePosition[2] + currentAngle)
                y_laser = currentRange * \
                    math.sin(particlePosition[2] + currentAngle)
                # print(x_laser, y_laser)
                x_predicted = particlePosition[0] + x_laser
                y_predicted = particlePosition[1] + y_laser
                # print(x_predicted, y_predicted)
                x_predicted_grid, y_predicted_grid = self.convertToGrid(
                    x_predicted, y_predicted)
                # print(x_predicted_grid, y_predicted_grid)
                # print(self.map[x_predicted_grid, y_predicted_grid])
                if(self.map[x_predicted_grid, y_predicted_grid] == 0):
                    # print("Entra")
                    break
                currentRange += self.mapInfo.resolution
            if(currentRange > self.callbacks.range_max):
                currentRange = self.callbacks.range_max
            elif (currentRange < self.callbacks.range_min):
                currentRange = self.callbacks.range_min
            self.predictedRanges.append(currentRange)
        # print(self.predictedRanges)

    def getMap(self):

        # http://docs.ros.org/en/diamondback/api/map_server/html/consumer_8py_source.html

        print("Getting occupancy grid map from service...\n")
        rospy.wait_for_service(MAP_TOPIC)
        mapMsg = rospy.ServiceProxy(MAP_TOPIC, GetMap)().map
        print("Got occupancy grid map.\n")
        # print(map)
        # print(map.info)
        # https://www.pluralsight.com/guides/different-ways-create-numpy-arrays
        # -1: inexistent, not mapped 0: free, 1-100: chance of obstacle

        print("Creating map...\n")
        map = np.array(mapMsg.data).reshape(
            (mapMsg.info.height, mapMsg.info.width))
        # print(mapMsg.info)
        # print("\n")
        # Used later on to convert between world coordinates and map coordinates
        self.mapInfo = mapMsg.info

        # 0: obstacle 1: free
        self.map = np.zeros(
            (self.mapInfo.height, self.mapInfo.width), dtype=bool)
        # print(self.map)
        # print("\n")

        # https://stackoverflow.com/questions/19766757/replacing-numpy-elements-if-condition-is-met
        self.map[map == 0] = 1
        # print(self.map)

        print("Map created\n")

        return

    def initializeParticles(self):

        print("Initializing the particles...\n")

        # https://thispointer.com/find-the-index-of-a-value-in-numpy-array/
        freeSpace = np.where(self.map == 1)

        # https://numpy.org/doc/stable/reference/random/generated/numpy.random.randint.html
        positions = np.random.randint(
            0, len(freeSpace[0]), size=self.numParticles)

        self.particles[:, 0] = freeSpace[1][positions]
        self.particles[:, 1] = freeSpace[0][positions]

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
            (self.mapInfo.origin.orientation.x, self.mapInfo.origin.orientation.y, self.mapInfo.origin.orientation.z, self.mapInfo.origin.orientation.w))
        # print("este", self.mapInfo.origin.orientation.y, roll, pitch, yaw)
        # Theta = yaw
        cos = math.cos(yaw)
        sin = math.sin(yaw)

        # Store x values temporarily since it will be changed
        tempX = np.copy(self.particles[:, 0])
        self.particles[:, 0] = (cos*self.particles[:, 0] - sin*self.particles[:, 1]) * \
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
    print(numParticles)

    pf = particleFilter(numParticles)
    rospy.spin()
