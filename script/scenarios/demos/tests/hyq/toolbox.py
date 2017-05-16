import math

## MYEX (INHERITS EXCEPTION)
# Class derived from Exception class, used to implement the specific exceptions of the toolbox
class MyEx(Exception):
	# class (static) attribute
	myex_table = {
					0 : "A 3D vector (or point, ...) must contain exactly 3 elements",
					1 : "Bad AngleEnum parameter",
					2 : "A 2D vector (or point, ...) must contain exactly 2 elements",
					3 : "A plane must contain exactly 4 elements, [a, b, c, d] such as: ax + by + cz + d = 0",
					4 : "A 2D straight line must contain exactly 2 elements, [a, b] such as: y = ax + b",
					5 : "Impossible to compare two vectors with different sizes"
				 }

	def __init__(self, val):
		self.value = val
		if MyEx.myex_table.has_key(val):
			self.message = MyEx.myex_table[val]
		else:
			self.message = "Unknown exception"

	def __repr__(self):
		return "{} : {}".format(self.value, self.message)

	def __str__(self):
		return self.message

	def __call__(self):
		print "MyEx code " + str(self.value) + " : " + self.message

## ANGLEENUM
# Class used as an enumeration to describe angle units
class AngleEnum:
	# class (static) attribute
	(RADIANS, DEGREES) = range(2)

## BUILDQUATERNION
# Construct a quaternion from a rotation axis and a rotation angle around it
#
# @param [In] vector The rotation axis
# @param [In] angle The rotation angle
# @param [In] convention The angle unit (radians or degrees)
#
# @return The normalized quaternion in the following convention --> [w, x, y, z]
def buildQuaternion(vector, angle, convention):
	# check the vector size
	if len(vector) != 3:
		raise MyEx(0)

	# convert in radian if necessary (and check the convention parameter)
	if convention == AngleEnum.DEGREES:
		angle = angle * math.pi / 180
	elif convention != AngleEnum.RADIANS:
		raise MyEx(1)

	# get the norm of vector
	norm = 0.0
	for i in range(len(vector)):
		norm += math.pow(vector[i], 2)
	norm = math.sqrt(norm)

	# if vector not normalized, normalize it
	if norm != 1.0:
		for i in range(len(vector)):
			vector[i] /= norm

	# build the result, [w, x, y, z]
	res = [math.cos(angle/2)]
	for i in range(len(vector)):
		res.append(vector[i]*math.sin(angle/2))

	return res

## ANGLE2D
# Get the angle from 3 2D-points (not oriented)
#
# @param [In] center The center point of rotation
# @param [In] end1 The point which gives the first vector (end1 - center)
# @param [In] end2 The point which gives the second vector (end2 - center)
#
# @return The angle in radian between the two vectors
def angle2D(center, end1, end2):
	# parameters consistence checking
	if (len(center) != 2) or (len(end1) != 2) or (len(end2) != 2):
		raise MyEx(2)

	# get the two vectors
	v1 = []; v2 = []
	v1.append(end1[0] - center[0]); v1.append(end1[1] - center[1])
	v2.append(end2[0] - center[0]); v2.append(end2[1] - center[1])

	# get their norms
	n1 = math.sqrt(math.pow(v1[0], 2) + math.pow(v1[1], 2))
	n2 = math.sqrt(math.pow(v2[0], 2) + math.pow(v2[1], 2))

	# calculate the angle between the two vectors
	dotProduct = v1[0]*v2[0] + v1[1]*v2[1]
	return math.acos(dotProduct/(n1*n2))

## ORIENTEDANGLE2D
# Get the oriented angle from 3 2D-points
#
# @param [In] center The center point of rotation
# @param [In] base The point which gives the base vector (base - center)
# @param [In] goal The point which gives the goal vector (goal - center)
#
# @return The oriented angle in radian between the base vector and the goal vector
def orientedAngle2D(center, base, goal):
	# parameters consistence checking
	if (len(center) != 2) or (len(base) != 2) or (len(goal) != 2):
		raise MyEx(2)

	# get the two vectors
	vbase = [];	vgoal = []
	vbase.append(base[0] - center[0]); vbase.append(base[1] - center[1])
	vgoal.append(goal[0] - center[0]); vgoal.append(goal[1] - center[1])

	# normalize the vectors
	normBase = math.sqrt(math.pow(vbase[0], 2) + math.pow(vbase[1], 2))
	normGoal = math.sqrt(math.pow(vgoal[0], 2) + math.pow(vgoal[1], 2))
	for i in range(2):
		vbase[i] /= normBase
		vgoal[i] /= normGoal

	# update the new norms
	normBase = math.sqrt(math.pow(vbase[0], 2) + math.pow(vbase[1], 2))
	normGoal = math.sqrt(math.pow(vgoal[0], 2) + math.pow(vgoal[1], 2))

	# calculate the angle between the two vectors
	dotProduct = vbase[0]*vgoal[0] + vbase[1]*vgoal[1]
	angle = math.acos(dotProduct/(normBase*normGoal))

	# calculate the orientation of the angle
	if ((vbase[0] >= 0) and (vgoal[0] > 0)) or ((vbase[0] > 0) and (vgoal[0] >= 0)): # both vectors in the right side
		if vbase[1] > vgoal[1]:
			angle = -angle
	elif ((vbase[0] <= 0) and (vgoal[0] < 0)) or ((vbase[0] < 0) and (vgoal[0] <= 0)): # both vectors in the left side
		if vbase[1] < vgoal[1]:
			angle = -angle
	elif ((vbase[1] >= 0) and (vgoal[1] > 0)) or ((vbase[1] > 0) and (vgoal[1] >= 0)): # both vectors in the top side
		if vbase[0] < vgoal[0]:
			angle = -angle
	elif ((vbase[1] <= 0) and (vgoal[1] < 0)) or ((vbase[1] < 0) and (vgoal[1] <= 0)): # both vectors in the bottom side
		if vbase[0] > vgoal[0]:
			angle = -angle
	elif ((vbase[0] == 0) and (vgoal[0] == 0)) or ((vbase[1] == 0) and (vgoal[1] == 0)): # both vectors along the same axis
		pass # Unknown direction, we cannot determine whether the angle goes from base to goal by the first or the second side of the straight line
	else: # vectors in opposite quadrants
		if ((vbase[0] > 0) and (vbase[1] > 0)) or ((vgoal[0] < 0) and (vgoal[1] < 0)): # first and third quadrants (posX-posY, negX-negY)
			vbase[0] = abs(vbase[0]); vbase[1] = abs(vbase[1])
			vgoal[0] = abs(vgoal[0]); vgoal[1] = abs(vgoal[1])
			if vbase[1] < vgoal[1]:
				angle = -angle
		else: # second and fourth quadrants (negX-posY, posX-negY)
			vbase[0] = abs(vbase[0]); vbase[1] = abs(vbase[1])
			vgoal[0] = abs(vgoal[0]); vgoal[1] = abs(vgoal[1])
			if vbase[1] > vgoal[1]:
				angle = -angle
	# return the result
	return angle

## ISINSIDE
# Check if a point is inside a polygon (convex or not)
#
# @param [In] point The point we consider
# @param [In] polygon The concerned polygon
#
# @return True if the point is inside the polygon, False otherwise
def isInside(point, polygon): # We assume that the polygon is sorted (each next index lead to the real next point)
	# parameters consistence checking
	if len(point) != 2:
		raise MyEx(2)
	for p in polygon:
		if len(p) != 2:
			raise MyEx(2)

	# get the winding number (sumAngles == windingNumber*2*pi)
	sumAngles = 0.0 # sumAngles is the sum of all subtended angles by each polygon edge from the considered point
	for i in range(len(polygon)-1):
		base = polygon[i][:]
		goal = polygon[i+1][:]
		sumAngles += orientedAngle2D(point, base, goal)
	sumAngles += orientedAngle2D(point, polygon[-1][:], polygon[0][:])
	
	if abs(sumAngles) < 0.0001: # if sumAngles == 0 --> point is outside polygon
		return False
	elif abs(sumAngles - 2*math.pi) < 0.0001: # if sumAngles == 2*pi --> point is inside polygon
		return True
	else: # impossible case
		return "Impossible case"

## ORTHOGONALPROJECTION
# Compute the orthogonal projection of a 3D point on a given plane
#
# @param [In] point The considered 3D-point
# @param [In] plane The plane [a, b, c, d] on which we want to project the point
#
# @return The orthogonal projection of the point on the plane
def orthogonalProjection(point, plane):
	# parameters consistence checking
	if len(point) != 3:
		raise MyEx(0)
	if len(plane) != 4:
		raise MyEx(3)

	k = (plane[0]*point[0] + plane[1]*point[1] + plane[2]*point[2] + plane[3])/(math.pow(plane[0], 2) + math.pow(plane[1], 2) + math.pow(plane[2], 2))
	return [(point[0] - k*plane[0]), (point[1] - k*plane[1]), (point[2] - k*plane[2])]

## DISTANCETOSTRAIGHTLINE2D
# Compute the minimum distance between a 2D point and a given 2D straight line
#
# @param [In] point The considered 2D point
# @param [In] straightLine The considered straight line [a, b]
#
# @return The (minimal) distance between the specified point and straight line
def distanceToStraightLine2D(point, straightLine):
	# parameters consistence checking
	if len(point) != 2:
		raise MyEx(2)
	if len(straightLine) != 2:
		raise MyEx(4)

	# get two points on the straight line (pa and pb)
	pa = [0]; pb = [1]
	pa.append(straightLine[1]); pb.append(sum(straightLine))

	# get the angle theta between the two vectors formed by pa-point and pa-pb
	v1 = [point[0] - pa[0], point[1] - pa[1]]
	v2 = [pb[0] - pa[0], pb[1] - pa[1]]
	theta = angle2D(pa, point, pb)

	# get the distance using the sinus of theta and the distance between pa-point (the norm of v1)
	n1 = math.sqrt(math.pow(v1[0], 2) + math.pow(v1[1], 2))
	return (math.sin(theta) * n1)

## STRAIGHTLINEFROMPOINTS2D
# Compute the 2D straight line equation from two 2D points of the straight line
#
# @param [In] p1 A point belonging to the straight line sought
# @param [In] p2 Another point belonging to the same straight line
#
# @return The straight line equation [a, b]
def straightLineFromPoints2D(p1, p2):
	# parameters consistence checking
	if (len(p1) != 2) or (len(p2) != 2):
		raise MyEx(2)

	# get the slope
	a = (p2[1] - p1[1])*1.0/(p2[0] - p1[0])

	# get the intercept
	b = p1[1] - a*p1[0]

	return [a, b]

## EUCLIDEANDIST
# Compute the eucliean distance between two vectors
#
# @param [In] v1 The first vector
# @param [In] v2 The second vector
#
# @return The euclidean distance between v1 and v2
def euclideanDist(v1, v2):
	# parameters consistence checking
	if len(v1) != len(v2):
		raise MyEx(5)

	sumOfSquare = 0
	for i in range(len(v1)):
		sumOfSquare += math.pow(v1[i] - v2[i], 2)
	return math.sqrt(sumOfSquare)

## WEIGHTEDCENTROIDCONVEX2D
# Compute the weighted centroid of a convex polygon
#
# @param [In] convexPolygon
#
# @return The weighted centroid (real center approximation) of the polygon
def weightedCentroidConvex2D(convexPolygon):
	# parameters consistence checking
	for p in convexPolygon:
		if len(p) != 2:
			raise MyEx(2)

	res = []
	if len(convexPolygon) == 1:
		res = convexPolygon[0]
	elif len(convexPolygon) == 2:
		res.append((convexPolygon[0][0] + convexPolygon[1][0])/2)
		res.append((convexPolygon[0][1] + convexPolygon[1][1])/2)
	else:
		# get the longest edge and define the minimum admissible threshold for counting a vertex as a single point
		maxDist = euclideanDist(convexPolygon[-1], convexPolygon[0])
		for i in range(len(convexPolygon)-1):
			dist = euclideanDist(convexPolygon[i], convexPolygon[i+1])
			if dist > maxDist:
				maxDist = dist
		threshold = maxDist*1.0/10

		# shift the list until starting from a lonely (to the rear) point
		shifted = convexPolygon[:]
		while (euclideanDist(shifted[-1], shifted[0]) <= threshold):
			shifted.append(shifted.pop(0))

		# look over the shifted set
		finalSet = []
		localSubset = []
		subsetOngoing = False
		shifted.append(shifted[0])
		for i in range(len(shifted)-1):
			if euclideanDist(shifted[i], shifted[i+1]) > threshold:
				if not subsetOngoing:
					finalSet.append(shifted[i])
				else:
					localSubset.append(shifted[i])
					moyX = 0; moyY = 0
					for p in localSubset:
						moyX += p[0]
						moyY += p[1]
					moyX /= (len(localSubset)*1.0)
					moyY /= (len(localSubset)*1.0)
					finalSet.append([moyX, moyY])
					del localSubset[:]
					subsetOngoing = False
			else:
				localSubset.append(shifted[i])
				if not subsetOngoing:
					subsetOngoing = True
		resX = 0; resY = 0
		for p in finalSet:
			resX += p[0]
			resY += p[1]
		resX /= (len(finalSet)*1.0)
		resY /= (len(finalSet)*1.0)
		res.append(resX); res.append(resY)
	return res

## POINTCLOUDSMANAGER
# Class to operate on point clouds, also used as a namespace
class PointCloudsManager:

	## SCANNINGPROCESS2D (STATIC)
	# Method used in convex hull computation, not aimed to be used outside
	# The parameters are not described because this function is private (but Python does not know this concept), you do not have to use this function anyway
	#
	# @param [In] basePoint ...
	# @param [InOut] subset ... (will be copied, modified and returned, the reference will not be used)
	# @param [InOut] angle (will be copied, modified and returned)
	# @param [In] currentPoint ...
	# @param [In] higher It equals True if currentPoint is above basePoint, False otherwise
	# @param [In] direction If True, scan to the right. Otherwise, scan to the left
	#
	# @return subset, angle (after modifications)
	@staticmethod
	def scanningProcess2D(basePoint, subset, angle, currentPoint, higher, direction):
		sset = subset[:]
		higher_val = 1 if higher else -1
		direction_val = 1 if direction else -1
		if len(sset) == 1:
			angle = angle2D(basePoint, [basePoint[0] + direction_val, basePoint[1]], currentPoint)
			sset.append(currentPoint)
		elif (higher_val*currentPoint[1]) >= (higher_val*sset[-1][1]):
			opening = angle2D(sset[-1], [sset[-1][0] + direction_val, sset[-1][1]], currentPoint)
			if opening <= angle:
				sset.append(currentPoint)
				angle = opening
			else:
				del sset[-1]
				opening = angle2D(sset[-1], [sset[-1][0] + direction_val, sset[-1][1]], currentPoint)
				convex = False
				if len(sset) == 1:
					convex = True
				while not convex:
					base = sset[-2]
					angle = angle2D(base, [base[0] + direction_val, base[1]], sset[-1])
					if angle < opening:
						del sset[-1]
						opening = angle2D(sset[-1], [sset[-1][0] + direction_val, sset[-1][1]], currentPoint)
					else:
						convex = True
					if len(sset) == 1:
						convex = True
				sset.append(currentPoint)
				angle = opening
		return sset, angle

	## CONVEXHULL2D (STATIC)
	# Method to compute the 2D convex hull of a 2D point cloud
	#
	# @param [In] pointCloud2D The considered 2D point cloud
	#
	# @return The convex hull of the specified 2D point cloud
	@staticmethod
	def convexHull2D(pointCloud2D):
		res = []
		if len(pointCloud2D) != 0:
			pc = pointCloud2D[:]
			# sort the input set by x increasing
			sortedSet = []
			while len(pc) != 0:
				index = 0
				minx = pc[index][0]
				for i in range(len(pc)):
					if pc[i][0] < minx:
						minx = pc[i][0]
						index = i
				sortedSet.append(pc[index])
				del pc[index]

			# first scanning, to the right
			basePoint = sortedSet[0]
			tr_up_set = [basePoint]; tr_down_set = [basePoint]
			upAngle = 0.0; downAngle = 0.0

			for i in range(1, len(sortedSet)):
				if sortedSet[i][1] >= basePoint[1]: # if the point is higher than basePoint
					tr_up_set, upAngle = PointCloudsManager.scanningProcess2D(basePoint, tr_up_set, upAngle, sortedSet[i], True, True)
				else: # if the point is lower than basePoint
					tr_down_set, downAngle = PointCloudsManager.scanningProcess2D(basePoint, tr_down_set, downAngle, sortedSet[i], False, True)

			# second scanning, to the left
			basePoint = sortedSet[-1]
			tl_up_set = [basePoint]; tl_down_set = [basePoint]

			for i in range(len(sortedSet)-2, -1, -1):
				if sortedSet[i][1] >= basePoint[1]: # if the point is higher than basePoint
					tl_up_set, upAngle = PointCloudsManager.scanningProcess2D(basePoint, tl_up_set, upAngle, sortedSet[i], True, False)
				else: # if the point is lower than basePoint
					tl_down_set, downAngle = PointCloudsManager.scanningProcess2D(basePoint, tl_down_set, downAngle, sortedSet[i], False, False)

			# merge the four subset without keeping the duplicates (subsets boundaries, ...)
			res_tmp = []
			for p in tr_down_set:
				res_tmp.append(p)
			for i in range(len(tl_down_set)-2, -1, -1):
				res_tmp.append(tl_down_set[i])
			for i in range(1, len(tl_up_set)):
				res_tmp.append(tl_up_set[i])
			for i in range(len(tr_up_set)-2, 0, -1):
				res_tmp.append(tr_up_set[i])
			for p in res_tmp:
				if not p in res:
					res.append(p)
		return res

	## WEIGHTEDCENTROID2D
	# Compute the weighted centroid of a point cloud
	#
	# @param [In] pointCloud2D The considered 2D point cloud
	#
	# @return The weighted centroid (real center approximation) of the point cloud
	@staticmethod
	def weightedCentroid2D(pointCloud2D):
		cH = PointCloudsManager.convexHull2D(pointCloud2D)
		return weightedCentroidConvex2D(cH)

## ISVALIDZMP
# Compute the capture point criterion validation
# The goal is to extend the capture point criterion (using the Zero-Moment-Point) to a 3D case (non coplanar contacts)
#
# @param [In] convexHull The convex hull of the support polygon
# @param [In] comPos The position of the center of mass of the robot
# @param [In] comAccel The center of mass acceleration
# @param [In] g The gravity acceleration, default value: 9.80665 m.s-1
#
# @return True if the criterion is validated, False otherwise
def isValidZMP(convexHull, comPos, comAccel, g = 9.80665):
	# determine the ZMP position
	x_zmp = comPos[0] - (comPos[2]/g)*comAccel[0]
	y_zmp = comPos[1] - (comPos[2]/g)*comAccel[1]

	# return if the ZMP is inside the convex hull of the support polygon (equilibrium for planar contacts) or not (fall)
	return isInside([x_zmp, y_zmp], convexHull)

## EVALZMP
# Evaluate a contact configuration in accordance with the capture point criterion
#
# @param [In] convexHull The convex hull of the support polygon
# @param [In] comPos The position of the center of mass of the robot
# @param [In] comAccel The center of mass acceleration
# @param [In] g The gravity acceleration, default value: 9.80665 m.s-1
#
# @return The cost of the current contact configuration
def evalZMP(convexHull, comPos, comAccel, g = 9.80665):
	pass

# buildQuaternion example
v = [0, 0, 1] # Rotation around z-axiz
a = 90 # rotation of 90 degrees
quat = buildQuaternion(v, a, AngleEnum.DEGREES)
print
print "Quaternion : axis = " + str(v) + " , angle = " + str(a) + " --> " + str(quat)

# convexHull example
pointCloud = []
pointCloud.append([3, 4])
pointCloud.append([4, 5])
pointCloud.append([4, 3.5])
pointCloud.append([5, 5])
pointCloud.append([5, 4])
pointCloud.append([5, 2])
pointCloud.append([6, 5.5])
pointCloud.append([6, 3])
pointCloud.append([6.5, 2])
pointCloud.append([7, 5.5])
pointCloud.append([7, 4])
pointCloud.append([7, 1.5])
pointCloud.append([8, 5.5])
pointCloud.append([8, 3.5])
pointCloud.append([8, 1])
pointCloud.append([8, 0.5])
pointCloud.append([8.5, 2.5])
pointCloud.append([9, 4])
pointCloud.append([10, 3])
pointCloud.append([11, 3])

cH = PointCloudsManager.convexHull2D(pointCloud)
print ""
print "Point cloud : " + str(pointCloud)
print "Convex hull : " + str(cH)

# weightedCentroid example
poly = [[0, 0], [0.5, 0], [100, 0], [100, 100], [0, 100], [0, 0.5]]
realCenter = [50, 50]
medX = 0; medY = 0
for p in poly:
	medX += p[0]
	medY += p[1]
medX /= (len(poly)*1.0)
medY /= (len(poly)*1.0)
centroid = [medX, medY]
wcentroid = weightedCentroidConvex2D(poly)
print ""
print "Polygon : " + str(poly)
print "Centroid : " + str(centroid)
print "Weighted centroid : " + str(wcentroid)
print "Real center : " + str(realCenter)