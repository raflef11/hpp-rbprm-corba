import math

class MyEx(Exception):
	# class (static) attribute
	myex_table = {
					0 : "A 3D vector must contain exactly 3 elements",
					1 : "Bad AngleEnum parameter",
					2 : "A 2D vector must contain exactly 2 elements",
					3 : "A plane must contain exactly 4 elements, [a, b, c, d] such as: ax + by + cz + d = 0",
					4 : "The maximum allowed precision for a given number was exceeded"
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

class AngleEnum:
	# class (static) attribute
	(RADIANS, DEGREES) = range(2)

## BUILDQUATERNION
# Construct a quaternion from a rotation axis and a rotation angle around it
#
# @param vector The rotation axis
# @param angle The rotation angle
# @param convention The angle unit (radians or degrees)
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
# @param center The center point of rotation
# @param end1 The point which gives the first vector (end1 - center)
# @param end2 The point which gives the second vector (end2 - center)
#
# @return The angle between the two vectors
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
# @param center The center point of rotation
# @param base The point which gives the base vector (base - center)
# @param goal The point which gives the goal vector (goal - center)
#
# @return The oriented angle between the base vector and the goal vector
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
# @param point The point we consider
# @param polygon The concerned polygon
#
# @return True if the point is inside the polygon, False otherwise
def isInside(point, polygon): # We assume the the polygon is sorted (each next index lead to the real next point)
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
# Compute the orthogonal projection of a 3D-point on a given plane
#
# @param point The considered 3D-point
# @param plane The plane [a, b, c, d] on which we want to project the point
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

## POINTCLOUDSMANAGER
# Class to operate on point clouds, also used as a namespace
class PointCloudsManager:

	@staticmethod
	def scanningProcess2D(basePoint, subset, angle, sortedSet, i, higher, direction):
		pass

	@staticmethod
	def convexHull2D(pointCloud2D):
		pass

## ISVALIDZMP
# Compute the capture point criterion validation
# The goal is to extend the capture point criterion (using the Zero-Moment-Point) to a 3D case (non coplanar contacts)
#
# @param convexHull The convex hull of the support polygon
# @param comPos The position of the center of mass of the robot
# @param comAccel The center of mass acceleration
# @param g The gravity acceleration, default value: 9.80665 m.s-1
#
# @return True if the criterion is validated, False otherwise
def isValidZMP(convexHull, comPos, comAccel, g = 9.80665):
	pass

## EVALZMP
# Evaluate a contact configuration in accordance with the capture point criterion
#
# @param convexHull The convex hull of the support polygon
# @param comPos The position of the center of mass of the robot
# @param comAccel The center of mass acceleration
# @param g The gravity acceleration, default value: 9.80665 m.s-1
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