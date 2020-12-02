#Libraries
import rospy
import math
import numpy as np
import tf.transformations
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import std_msgs.msg

#ROS messages

#Occupancy Grid Map message
from nav_msgs.srv import GetMap

#Constants

NUMBER_OF_PARTICLES = 100

MAP_TOPIC = "static_map"

#https://stackoverflow.com/questions/4877624/numpy-array-of-objects
#class particle():
class particle():

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
	self.first_msg = True
	rospy.Subscriber("/husky_velocity_controller/odom", Odometry, self.OdometryCallback, queue_size = 10)
	rospy.Subscriber("/scan", LaserScan, self.LaserCallback, queue_size = 10)
	

    def OdometryCallback(self, msg):
	if self.first_msg is True:
	    self.old_msg = []
	    self.new_msg = []
	    self.new_msg = msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.w
	    self.first_msg = False
	else:
	    self.old_msg = copy.deepcopy(self.new_msg)
	    self.new_msg = msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.w
	    # print(self.new_msg, self.old_msg)
	    self.delta_x = self.new_msg[0] - self.old_msg[0]
	    self.delta_y = self.new_msg[1] - self.old_msg[1]
	    self.delta_theta = self.new_msg[2] - self.old_msg[2]
	    self.moveParticle(self.delta_x, self.delta_y, self.delta_theta)
		
    def LaserCallback(self, msg):
	self.angle_max = msg.angle_max
        self.angle_min = msg.angle_min 
        self.angle_increment = msg.angle_increment
        self.range_max = msg.range_max
        self.ranges = msg.ranges
	    

    def moveParticle(self, x, y, theta):
	# self.x = msg.pose.pose.position.x
	# self.y = msg.pose.pose.position.y
	# self.yaw = msg.pose.pose.position.w
	self.x = self.x + x
	self.y = self.y + y
	self.yaw = self.yaw + theta
	# print(self.x, self.y, self.yaw)


    def restartMovement(self):
	# print("entra")
	self.delta_x = 0.0
	self.delta_y = 0.0
	self.delta_yaw = 0.0
	self.x = 0.0
	self.y = 0.0
	self.yaw = 0.0
		

class particleFilter():

    def __init__(self):
        
        self.numParticles = NUMBER_OF_PARTICLES

	#https://stackoverflow.com/questions/4877624/numpy-array-of-objects
	# There is no advantage in running a numpy array of objects over iterating all the objects
	# So, numpy is used directly for performance
	# X, Y, Theta and Weight
	self.particles = np.zeros((self.numParticles,4))
	# In the beginning, all particles have the same weight
	self.particles[:,3] = 1.0 / self.numParticles

	self.map = None
	self.mapInfo = None

	self.getMap()
	self.initializeParticles()
	self.particle = particle()

	# Publisher
	# Publisher needs a queue_size, see if that matters
	self.particlePublisher = rospy.Publisher("/MCLocalization/particleCloud", PoseArray, queue_size = 1)

	# JUST FOR TEST FOR NOW
	#https://answers.ros.org/question/316829/how-to-publish-a-pose-in-quaternion/
	rate = rospy.Rate(1) # Hz
        while not rospy.is_shutdown():
	    for i in range(self.numParticles):
		# print("aaa", self.particle.x, self.particle.y, self.particle.yaw)
	        self.particles[i,0] += self.particle.x
		self.particles[i,1] += self.particle.y
		self.particles[i,2] += self.particle.yaw
            self.publishParticles()
	    print("Publishing...\n")
            rate.sleep()

    def getMap(self):

        #http://docs.ros.org/en/diamondback/api/map_server/html/consumer_8py_source.html
        
	print("Getting occupancy grid map from service...\n")
        rospy.wait_for_service(MAP_TOPIC)
        mapMsg = rospy.ServiceProxy(MAP_TOPIC, GetMap)().map
        print("Got occupancy grid map.\n")
	#print(map)
	#print(map.info)
        #https://www.pluralsight.com/guides/different-ways-create-numpy-arrays
	# -1: inexistent, not mapped 0: free, 1-100: chance of obstacle

	print("Creating map...\n")
	map = np.array(mapMsg.data).reshape((mapMsg.info.height, mapMsg.info.width))
	#print(mapMsg.info)	
	#print("\n")
	# Used later on to convert between world coordinates and map coordinates
	self.mapInfo = mapMsg.info

	# 0: obstacle 1: free
	self.map = np.zeros((self.mapInfo.height, self.mapInfo.width), dtype = bool)
	#print(self.map)
	#print("\n")

	#https://stackoverflow.com/questions/19766757/replacing-numpy-elements-if-condition-is-met
        self.map[map == 0] = 1
	#print(self.map)
	
	print("Map created\n")

	return

    def initializeParticles(self):
        
        print("Initializing the particles...\n")

	#https://thispointer.com/find-the-index-of-a-value-in-numpy-array/
        freeSpace = np.where(self.map == 1)

	#https://numpy.org/doc/stable/reference/random/generated/numpy.random.randint.html
        positions = np.random.randint(0, len(freeSpace[0]), size = self.numParticles)

        self.particles[:,0] = freeSpace[1][positions]
        self.particles[:,1] = freeSpace[0][positions]

	#https://numpy.org/doc/stable/reference/random/generated/numpy.random.random_sample.html
	# Convert angle to a number between 0 and 2*Pi
        self.particles[:,2] = np.random.random_sample(self.numParticles) * math.pi * 2.0

	# Convert map indices to real world coordinates
	self.convertToWorld()
        
	print("Particles Initialized.\n")

	return
    
    #https://answers.ros.org/question/10268/where-am-i-in-the-map/?fbclid=IwAR2IQXDmSXQZkBQK4wDJ-243bbHMw-noLNU18-E7FSxcqNfv3yxqESfhDZM
    #https://answers.ros.org/question/161576/solved-getting-map-co-ordinates/?fbclid=IwAR3dcXXAKbBcXLDnB2bRI8pQXnzA_eQKZa9e-ilHuhloPB11PBYSg6zcAu0
    # [x;y] = R * [px * resolution ; py * resolution] + [x0,y0]
    # R = [(cos theta, -sin theta), (sin theta, cos theta)]
    def convertToWorld(self):

    	roll, pitch, yaw = tf.transformations.euler_from_quaternion((self.mapInfo.origin.orientation.x, self.mapInfo.origin.orientation.y, self.mapInfo.origin.orientation.z, self.mapInfo.origin.orientation.w)) 
	
	# Theta = yaw
    	cos = math.cos(yaw)
	sin = math.sin(yaw)	
	
	# Store x values temporarily since it will be changed
    	tempX = np.copy(self.particles[:,0])

    	self.particles[:,0] = (cos*self.particles[:,0] - sin*self.particles[:,1]) * float(self.mapInfo.resolution) + self.mapInfo.origin.position.x

    	self.particles[:,1] = (sin*tempX + cos*self.particles[:,1]) * float(self.mapInfo.resolution) + self.mapInfo.origin.position.y

    	self.particles[:,2] += yaw

	return

    def publishParticles(self):
	#http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseArray.html
	#http://docs.ros.org/en/diamondback/api/geometry_msgs/html/msg/PoseArray.html
	#https://answers.ros.org/question/60209/what-is-the-proper-way-to-create-a-header-with-python/
	#https://www.programcreek.com/python/example/86351/std_msgs.msg.Header

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

	    #https://answers.ros.org/question/69754/quaternion-transformations-in-python/ 
            quaternion = tf.transformations.quaternion_from_euler(0, 0, self.particles[i, 2])
	    pose.orientation.x = quaternion[0]
            pose.orientation.y = quaternion[1]
            pose.orientation.z = quaternion[2]
            pose.orientation.w = quaternion[3]
	    
            particlePoses.append(pose)

	particleCloud.poses = particlePoses

	self.particlePublisher.publish(particleCloud)


if __name__ == "__main__":
    rospy.init_node("MCLocalization")

    pf = particleFilter()
    rospy.spin()
