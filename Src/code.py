#Libraries
import rospy
import math
import numpy as np

#ROS messages

#Occupancy Grid Map message
from nav_msgs.srv import GetMap

#Constants

NUMBER_OF_PARTICLES = 100

MAP_TOPIC = "static_map"

#https://stackoverflow.com/questions/4877624/numpy-array-of-objects
#class particle():

#    def __init__(self):
#        self.x = 0.0
#        self.y = 0.0
#        self.yaw = 0.0
#
#    def moveParticle(self, x, y, theta):
#	self.x = self.x + x
#	self.y = self.y + y
#	self.yaw = self.yaw + theta

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

        self.particles[:,0] = freeSpace[0][positions]
        self.particles[:,1] = freeSpace[1][positions]

	#https://numpy.org/doc/stable/reference/random/generated/numpy.random.random_sample.html
	# Convert angle to a number between 0 and 2*Pi
        self.particles[:,2] = np.random.random_sample(self.numParticles) * math.pi * 2.0
	
        #TODO

	# Convert map indices to real world coordinates
	print("Dont forget to convert coordinates to real world\n")
        
	print("Particles Initialized.\n")

	return

if __name__ == "__main__":
    rospy.init_node("MCLocalization")

    pf = particleFilter()
    rospy.spin()
