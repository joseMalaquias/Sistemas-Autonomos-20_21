import numpy as np

# "cimport" is used to import special compile-time information
# about the numpy module (this is stored in a file numpy.pxd which is
# currently part of the Cython distribution).
cimport numpy as np

#from cython.parallel import prange
import tf  

cimport libc.math

# LASER BEAMS need to be modified and also the array predictedRanges to the same value of LASER BEAMS if modified in mcl.py
# https://stackoverflow.com/questions/8118982/rounding-numbers-to-a-specific-resolution
cdef double roundToMapResolution (double value, double resolution) nogil:
    # ATTENTION: Can have a small round error - test range compared with real range

    return libc.math.round(value / resolution) * resolution

# https://jakevdp.github.io/blog/2012/08/08/memoryview-benchmarks/
#cdef double* cTestRange(double minRange, double maxRange, double[:] anglesSampled, double[:, ::1] obstacleDistanceMap, double #resolution, double[:] particlePosition) nogil:
#
#    cdef int angle = 0, predictedYInt = 0, predictedXInt = 0, laserID = 0
#    cdef double currentRange = 0.0, laserRotation = 0.0, predictedX = 0.0, predictedY = 0.0, dist = 0.0, laserOrientation = 0.0, #coeff = 0.99
#    cdef int LASER_BEAMS = 16
#    cdef double predictedRanges[16]
#
#    for angle in range(LASER_BEAMS):
#        laserRotation = particlePosition[2] + anglesSampled[angle]
#        currentRange = 0.0
#        while(currentRange <= maxRange):
#            predictedX = particlePosition[0] + (currentRange * libc.math.cos(laserRotation)) 
#            predictedY = particlePosition[1] + (currentRange * libc.math.sin(laserRotation)) 
#
#            predictXInt = <int> predictedX            
#            predictYInt = <int> predictedY
#            dist = obstacleDistanceMap[predictedYInt][predictedXInt]
#            if dist == 0.0:
#                break
#            currentRange += max(dist*coeff, 1.0)
#
#        currentRange *= resolution
#        if(currentRange > maxRange):
#            currentRange = maxRange
#        elif (currentRange < minRange):
#            currentRange = minRange
#      
#        predictedRanges[laserID] = roundToMapResolution(currentRange, resolution)
#        laserID += 1
#    with gil:
#        print("No optimizado")
#        print(predictedRanges)
#    return predictedRanges   

cimport cython
@cython.boundscheck(False) # turn off bounds-checking for entire function
@cython.wraparound(False)  # turn off negative index wrapping for entire function
# LASER BEAMS need to be modified if modified in mcl.py
#https://cython.readthedocs.io/en/latest/src/tutorial/numpy.html
def measurePrediction(double[:, ::1] particlePosition, np.double_t resolution, np.ndarray[dtype = np.double_t, ndim = 1] laserDetails, double[:] anglesSampled, double[:, ::1] obstacleDistanceMap, np.int_t numParticles):

    cdef np.float64_t minRange = laserDetails[0] / resolution
    cdef np.float64_t maxRange = laserDetails[1] / resolution
    cdef np.float64_t currentAngle, currentRange, laserRotation, predictedX, predictedY, dist, coeff = 0.99
    cdef np.int_t i, laserID
    cdef int LASER_BEAMS = 16
    cdef double result[16]
    cdef np.ndarray[dtype = double, ndim = 2] predictedRanges = np.zeros((numParticles, LASER_BEAMS))
    #cdef np.ndarray[dtype = np.float64_t, ndim = 1] particleOrientation
    #cdef np.ndarray[dtype = np.float64_t, ndim = 1] laserOrientation
    
    #with nogil, parallel():
    for i in range(numParticles):
        #result = cTestRange(minRange, maxRange, anglesSampled, obstacleDistanceMap, resolution, particlePosition[i])
        #print(result)
        #for j in range(LASER_BEAMS):
        #    predictedRanges[i][j] = result[j]
        #particleOrientation = tf.transformations.quaternion_from_euler(0, 0, particlePosition[i][2])
	# Ranges will be calculated for each angle
        laserID = 0
        for currentAngle in anglesSampled:
            #laserOrientation = tf.transformations.quaternion_from_euler(0, 0, currentAngle)
            #_, _, laserRotation = tf.transformations.euler_from_quaternion(tf.transformations.quaternion_multiply(particleOrientation, laserOrientation))
            laserRotation = particlePosition[i][2] + currentAngle
            currentRange = 0.0
            while(currentRange <= maxRange):
                predictedX = particlePosition[i][0] + currentRange * libc.math.cos(laserRotation) 
                predictedY = particlePosition[i][1] + currentRange * libc.math.sin(laserRotation) 

                dist = obstacleDistanceMap[int(predictedY), int(predictedX)]
                if dist == 0.0:
                    break
                currentRange += max(dist*coeff, 1.0)

            currentRange *= resolution
            if(currentRange > maxRange):
                currentRange = maxRange
            elif (currentRange < minRange):
                currentRange = minRange
	        
            predictedRanges[i][laserID] = roundToMapResolution(currentRange, resolution)
            laserID += 1
    #with gil:
    return predictedRanges

