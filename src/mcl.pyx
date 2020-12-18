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

cdef double convertToGrid(double coordinate, double initialCoordinate, double resolution):
    """
    Converts an array from real world coordinates to Grid Map coordinates
    """

    return (coordinate - initialCoordinate) / resolution

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
def measurePrediction (double[:, ::1] particlePosition, np.double_t resolution, np.ndarray[dtype = np.double_t, ndim = 1] laserDetails, double[:] anglesSampled, double[:, ::1] obstacleDistanceMap, np.int_t numParticles):

    cdef np.float64_t minRange = laserDetails[0] / resolution
    cdef np.float64_t maxRange = laserDetails[1] / resolution
    cdef np.float64_t currentAngle, currentRange, laserRotation, predictedX, predictedY, dist, coeff = 0.99
    cdef np.int_t i, j, laserID
    cdef int LASER_BEAMS = 16
    #cdef double result[16]
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
        for j in range(LASER_BEAMS):
            #laserOrientation = tf.transformations.quaternion_from_euler(0, 0, currentAngle)
            #_, _, laserRotation = tf.transformations.euler_from_quaternion(tf.transformations.quaternion_multiply(particleOrientation, laserOrientation))
            currentAngle = anglesSampled[j]
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

def predictRangeAndCalculateDeviation(double[:, ::1] particlePosition, np.ndarray[dtype = np.double_t, ndim = 1] mapInfo, np.ndarray[dtype = np.double_t, ndim = 1] laserDetails, double[:] anglesSampled, double[:] rangesSampled, double[:, ::1] obstacleDistanceMap, np.int_t numParticles):

    cdef np.float64_t minRange = laserDetails[0] / mapInfo[2]
    cdef np.float64_t maxRange = laserDetails[1] / mapInfo[2]
    cdef np.float64_t currentAngle, currentRange, laserRotation, predictedX, predictedY, dist, gridX, gridY, weight, coeff = 0.99
    cdef np.int_t i, j, laserDeviation = 10
    cdef np.int_t LASER_BEAMS = 16
    #cdef double result[16]
    #cdef np.ndarray[dtype = double, ndim = 2] predictedRanges = np.zeros((numParticles, LASER_BEAMS))
    #cdef np.ndarray[dtype = np.float64_t, ndim = 1] particleOrientation
    #cdef np.ndarray[dtype = np.float64_t, ndim = 1] laserOrientation
    cdef np.ndarray[dtype = double, ndim = 1] particlesWeight = np.zeros(numParticles)
    cdef np.ndarray[dtype = double, ndim = 1] rangesDifference = np.array([])
    
    #with nogil, parallel():
    for i in range(numParticles):
        #result = cTestRange(minRange, maxRange, anglesSampled, obstacleDistanceMap, resolution, particlePosition[i])
        #print(result)
        #for j in range(LASER_BEAMS):
        #    predictedRanges[i][j] = result[j]
        #particleOrientation = tf.transformations.quaternion_from_euler(0, 0, particlePosition[i][2])
	# Ranges will be calculated for each angle
        gridX = convertToGrid(particlePosition[i][0], mapInfo[0], mapInfo[2])
        gridY = convertToGrid(particlePosition[i][1], mapInfo[1], mapInfo[2])
        if(obstacleDistanceMap[int(gridY), int(gridX)] == 0):
            particlesWeight[i] = libc.math.exp(-(400)*(400))
            continue

        rangesDifference = np.array([])
        for j in range(LASER_BEAMS):
            #laserOrientation = tf.transformations.quaternion_from_euler(0, 0, currentAngle)
            #_, _, laserRotation = tf.transformations.euler_from_quaternion(tf.transformations.quaternion_multiply(particleOrientation, laserOrientation))
            currentAngle = anglesSampled[j]
            # in case of the laser value being infinite, do not take
            # this measure into account
            if(rangesSampled[j] == float("inf")):
                continue
            laserRotation = particlePosition[i][2] + currentAngle
            currentRange = 0.0
            while(currentRange <= maxRange):
                predictedX = gridX + currentRange * libc.math.cos(laserRotation) 
                predictedY = gridY + currentRange * libc.math.sin(laserRotation) 

                dist = obstacleDistanceMap[int(predictedY), int(predictedX)]
                if dist == 0.0:
                    break
                currentRange += max(dist*coeff, 1.0)

            currentRange *= mapInfo[2]
            if(currentRange > maxRange):
                currentRange = maxRange
            elif (currentRange < minRange):
                currentRange = minRange
	        
            rangesDifference = np.append(rangesDifference, (rangesSampled[j] - roundToMapResolution(currentRange, mapInfo[2])))
        # calculate the norm of those ranges differences
        particleDifference = np.linalg.norm(rangesDifference)

        # Calculate the weight (not normalized) of that particle
        weight = (1/(libc.math.sqrt(2*libc.math.pi)*laserDeviation)) * \
            libc.math.exp(-(particleDifference**2)/(2*laserDeviation**2))

        particlesWeight[i] = weight

    #with gil:
    return particlesWeight

