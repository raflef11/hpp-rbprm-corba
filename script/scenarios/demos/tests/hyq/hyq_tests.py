from hpp.gepetto import Viewer
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.affordance.affordance import AffordanceTool
from hyq_ref_pose import hyq_ref
import toolbox as tools
import hpp.corbaserver.rbprm.state_alg as state_alg

rootJointType = "freeflyer"
urdfSuffix = ""
srdfSuffix = ""

'''
# rbprmBuilder
packageName = "hpp-rbprm-corba"
meshPackageName = "hpp-rbprm-corba"
urdfName = "hyq_trunk_large"
urdfNameRom = ["hyq_lhleg_rom","hyq_lfleg_rom","hyq_rfleg_rom","hyq_rhleg_rom"]

rbprmBuilder = Builder()
rbprmBuilder.loadModel(urdfName, urdfNameRom, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
rbprmBuilder.setJointBounds ("base_joint_xyz", [-2, 5, -1, 1, 0.3, 4])
rbprmBuilder.setFilter(["hyq_rhleg_rom", "hyq_lfleg_rom", "hyq_rfleg_rom","hyq_lhleg_rom"])
rbprmBuilder.setAffordanceFilter("hyq_rhleg_rom", ["Support"])
rbprmBuilder.setAffordanceFilter("hyq_rfleg_rom", ["Support",])
rbprmBuilder.setAffordanceFilter("hyq_lhleg_rom", ["Support"])
rbprmBuilder.setAffordanceFilter("hyq_lfleg_rom", ["Support",])
rbprmBuilder.boundSO3([-0.4, 0.4, -3, 3, -3, 3])

pps = ProblemSolver(rbprmBuilder)
rr = Viewer(pps)

afftool = AffordanceTool()
afftool.loadObstacleModel(packageName, "darpa", "planning", rr)
'''

# fullBody
packageName = "hyq_description"
meshPackageName = "hyq_description"
urdfName = "hyq"

fullbody = FullBody()
fullbody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullbody.setJointBounds("base_joint_xyz", [-2, 5, -1, 1, 0.3, 4])

ps = ProblemSolver(fullbody)
#r = Viewer(ps, viewerClient=rr.client)
r = Viewer(ps)
afftool = AffordanceTool(); afftool.loadObstacleModel("hpp-rbprm-corba", "darpa", "planning", r)

cType = "_3_DOF"
offset = [0., -0.021, 0.]
normal = [0, 1, 0]
legx = 0.02; legy = 0.02
nbSamples = 20000

rLegId = "rfleg"
lLegId = "lfleg"
rArmId = "rhleg"
lArmId = "lhleg"

rLeg = "rf_haa_joint"
lLeg = "lf_haa_joint"
rArm = "rh_haa_joint"
lArm = "lh_haa_joint"

rFoot = "rf_foot_joint"
lFoot = "lf_foot_joint"
rHand = "rh_foot_joint"
lHand = "lh_foot_joint"

fullbody.addLimb(rLegId, rLeg, rFoot, offset, normal, legx, legy, nbSamples, "jointlimits", 0.05, cType)
fullbody.addLimb(lLegId, lLeg, lFoot, offset, normal, legx, legy, nbSamples, "jointlimits", 0.05, cType)
fullbody.addLimb(rArmId, rArm, rHand, offset, normal, legx, legy, nbSamples, "jointlimits", 0.05, cType)
fullbody.addLimb(lArmId, lArm, lHand, offset, normal, legx, legy, nbSamples, "jointlimits", 0.05, cType)

#fullbody.runLimbSampleAnalysis(rLegId, "jointLimitsDistance", True)
#fullbody.runLimbSampleAnalysis(lLegId, "jointLimitsDistance", True)
#fullbody.runLimbSampleAnalysis(rArmId, "jointLimitsDistance", True)
#fullbody.runLimbSampleAnalysis(lArmId, "jointLimitsDistance", True)

q_init = hyq_ref[:]
#fullbody.setStartState(q_init, [rLegId, lLegId, rArmId, lArmId])
r(q_init)

## HYQ
# Class to implement additional methods related to Hyq
class Hyq:

	## GETJOINTPOSITION
	# The fullbody.getJointPosition() is wrong, it inverses the front and the rear
	# This method corrects this error
	#
	# @param [In] prefix The prefix for the desired limb
	# @param [in] name The name of the desired joint
	#
	# @return The position of the joint of the specified limb
	@staticmethod
	def getJointPosition(prefix, name):
		prefixes = ["lh", "lf", "rh", "rf"]
		names = ["haa", "hfe", "kfe", "foot"]

		if prefix not in prefixes:
			return "Unknown prefix"
		if name not in names:
			return "Unknown joint name"
		# inverse front and rear in order to get the real position of the desired limb
		if prefix == prefixes[0]:
			prefix = prefixes[1]
		elif prefix == prefixes[1]:
			prefix = prefixes[0]
		elif prefix == prefixes[2]:
			prefix = prefixes[3]
		else:
			prefix = prefixes[2]
		return fullbody.getJointPosition(prefix + "_" + name + "_joint")

	# MGD
	# Compute the MGD of the Hyq
	#
	# @param [In] prefix The prefix of the considered joint in order to identify it
	# @param [In] q1 The haa configuration
	# @param [In] q2 The hfe configuration
	# @param [In] q3 The kfe configuration
	#
	# @return The position of the foot knwowing the configuration of its limb; The orientation of the foot; The transform matrix between world frame and the last joint (kfe)
	@staticmethod
	def MGD(prefix):
		prefixes = {"lh" : 7, "lf" : 10, "rh" : 13, "rf" : 16}
		if prefix not in prefixes.keys():
			return "Unknown prefix"

		k = prefixes[prefix]
		[q1, q2, q3] = fullbody.getCurrentConfig()[k:k+3]

		# get the transform between the world frame and the base coordinate system used for the MGD
		pos = Hyq.getJointPosition(prefix, "haa")[0:3]
		robotOrientQuat = fullbody.getCurrentConfig()[3:7]
		robotOrient = tools.quaternionToMatrix(robotOrientQuat, True)
		mgdBaseOrient = [[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
		rot = tools.multiplyMatrices(robotOrient, mgdBaseOrient)

		Tworld_0 = [
					[rot[0][0], rot[0][1], rot[0][2], pos[0]],
					[rot[1][0], rot[1][1], rot[1][2], pos[1]],
					[rot[2][0], rot[2][1], rot[2][2], pos[2]],
					[0.0,       0.0,             0.0,    1.0]
				]

		if (prefix == "lh") or (prefix == "lf"): # one of the limbs on the left
			q1 = -q1
		q1 -= tools.math.pi/2

		# compute the MGD
		c1 = tools.math.cos(q1); s1 = tools.math.sin(q1)
		c2 = tools.math.cos(q2); s2 = tools.math.sin(q2)
		c3 = tools.math.cos(q3); s3 = tools.math.sin(q3)
		#l1 = 0.082; l2 = 0.35; l3 = 0.35
		l1 = tools.euclideanDist(Hyq.getJointPosition(prefix, "haa")[0:3], Hyq.getJointPosition(prefix, "hfe")[0:3])
		l2 = tools.euclideanDist(Hyq.getJointPosition(prefix, "hfe")[0:3], Hyq.getJointPosition(prefix, "kfe")[0:3])
		l3 = tools.euclideanDist(Hyq.getJointPosition(prefix, "kfe")[0:3], Hyq.getJointPosition(prefix, "foot")[0:3])

		T01 = [[c1, -s1, 0, 0], [0, 0, -1, 0], [s1, c1, 0, 0], [0, 0, 0, 1]]
		T12 = [[c2, -s2, 0, l1], [0, 0, 1, 0], [-s2, -c2, 0, 0], [0, 0, 0, 1]]
		T23 = [[c3, -s3, 0, l2], [s3, c3, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]

		T03 = tools.multiplyMatrices(tools.multiplyMatrices(T01, T12), T23)

		# position of the OT
		X3 = [[l3], [0], [0], [1]]
		X0 = tools.multiplyMatrices(T03, X3) # X0 = T03 * X3

		# Xworldframe = Tworld_0 * X0
		# (Rworldframe == Rworld_3 * [[0, 0, -1], [-1, 0, 0], [0, 1, 0]], I don't know why, I thought it was just Rworld_3 (normally it is))
		# Tworld_3 = Tworld_0 * T03
		Xworldframe = tools.multiplyMatrices(Tworld_0, X0)
		Tworld_3 = tools.multiplyMatrices(Tworld_0, T03)
		Rworldframe = tools.multiplyMatrices([Tworld_3[0][0:3], Tworld_3[1][0:3], Tworld_3[2][0:3]], [[0, 0, -1], [-1, 0, 0], [0, 1, 0]])

		return Xworldframe, Rworldframe, Tworld_3

# ---------------------
# Work in progress zone
# ---------------------

'''
# MGI of Hyq
def HyqMGI(prefix):
	pass

def transformXworldToX0(prefix, Xworld):
	prefixes = ["lh", "lf", "rh", "rf"]
	if prefix not in prefixes:
		return "Not a valid prefix"
	if len(Xworld) != 7:
		return "Xworld must be a cartesian position and a [w,x,y,z] quaternion : vector of size 7"

	# get Tworld_0
	pos = Hyq.getJointPosition(prefix, "haa")[0:3]
	robotOrientQuat = fullbody.getCurrentConfig()[3:7]
	robotOrient = tools.quaternionToMatrix(robotOrientQuat)
	mgdBaseOrient = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
	rot = tools.multiplyMatrices(robotOrient, mgdBaseOrient)

	Tworld_0 = [[rot[0][0], rot[0][1], rot[0][2], pos[0]],
				[rot[1][0], rot[1][1], rot[1][2], pos[1]],
				[rot[2][0], rot[2][1], rot[2][2], pos[2]],
				[0.0,       0.0,             0.0,    1.0]]

	#get T0_world
	T0_world = tools.inverseHomogeneousMatrix(Tworld_0)

	# X0 = T0_world * Xworld
	# ---
	# P0 = T0_world * Pworld
	# R0 = R0_world * Rworld
	P0 = tools.multiplyMatrices(T0_world, [[Xworld[0]], [Xworld[1]], [Xworld[2]], [1]])
	Rworld = tools.quaternionToMatrix(Xworld[3:7])
	R0_world = tools.transposeMatrix(rot)
	R0 = tools.multiplyMatrices(R0_world, Rworld)
	#quat0 = tools.matrixToQuaternion(R0) # does not exist yet
	#return ([P0[0][0], P0[1][0], P0[2][0]] + quat0)
	return [P0[0][0], P0[1][0], P0[2][0]], R0 # P0, R0

# set an end-effector position
def setEndEffectorPosition(name, pos):
	pass

'''

# ----------
# Tests zone
# ----------

'''
# Test MGD Hyq
def testMGD(prefix):
	prefixes = ["lh", "lf", "rh", "rf"]
	if prefix not in prefixes:
		return "Not a valid prefix"
	Xreal = Hyq.getJointPosition(prefix, "foot")[0:3]
	print prefix + "_foot_joint position from model : " + str(Xreal)
	Xot, _, _ = Hyq.MGD(prefix)
	X = [Xot[0][0], Xot[1][0], Xot[2][0]]
	print prefix + "_foot_joint position from MGD :   " + str(X)
	print "errorX : " + str(abs(X[0] - Xreal[0]))
	print "errorY : " + str(abs(X[1] - Xreal[1]))
	print "errorZ : " + str(abs(X[2] - Xreal[2]))
	print "errorDist : " + str(tools.euclideanDist(X, Xreal))

# test each limb MGD
def eachLimbMGD():
	print "---"
	for p in ["lh", "lf", "rh", "rf"]:
		testMGD(p)
		print "---"

def mgiq1test(prefix, XworldDes, epsilon):
	prefixes = ["lh", "lf", "rh", "rf"]
	if prefix not in prefixes:
		return "Not a valid prefix"
	epsilon = 1.0 if epsilon >= 0 else -1.0

	l1 = tools.euclideanDist(Hyq.getJointPosition(prefix, "haa")[0:3], Hyq.getJointPosition(prefix, "hfe")[0:3])
	l2 = tools.euclideanDist(Hyq.getJointPosition(prefix, "hfe")[0:3], Hyq.getJointPosition(prefix, "kfe")[0:3])
	l3 = tools.euclideanDist(Hyq.getJointPosition(prefix, "kfe")[0:3], Hyq.getJointPosition(prefix, "foot")[0:3])

	P0T, R0T = transformXworldToX0(prefix, XworldDes) # R0T == R03
	P03 = tools.addMatrices([[P0T[0]], [P0T[1]], [P0T[2]]], tools.multiplyMatrices(R0T, [[l3], [0], [0]]), '-')
	T03 = [[R0T[0][0], R0T[0][1], R0T[0][2], P03[0][0]],
		   [R0T[1][0], R0T[1][1], R0T[1][2], P03[1][0]],
		   [R0T[2][0], R0T[2][1], R0T[2][2], P03[2][0]],
		   [0,         0,         0,         1        ]]

	# eq2
	q1_eq2 = tools.math.atan2(-T03[0][2], T03[2][2])

	# eq4
	q1_eq4 = tools.math.atan2(T03[2][0], T03[0][0])

	# eq5
	den = (tools.math.pow(T03[2][2], 2) + tools.math.pow(T03[0][2], 2))
	if den >= 1:
		s1_eq5 = (-T03[0][2] + epsilon*T03[2][2]*tools.math.sqrt(den - 1))/den
		c1_eq5 = (T03[2][2] + epsilon*T03[0][2]*tools.math.sqrt(den - 1))/den
		q1_eq5 = tools.math.atan2(s1_eq5, c1_eq5)
	else:
		print "den : " + str(den)
		q1_eq5 = -10

	# eq6
	q1_eq6 = tools.math.atan2(T03[2][3], T03[0][3])

	print "epsilon : " + str(epsilon)
	print str(q1_eq2) + " -- " + str(q1_eq4) + " -- " + str(q1_eq5) + " -- " + str(q1_eq6)

def mgiq2test(prefix, q1, XworldDes):
	prefixes = ["lh", "lf", "rh", "rf"]
	if prefix not in prefixes:
		return "Not a valid prefix"

	l1 = tools.euclideanDist(Hyq.getJointPosition(prefix, "haa")[0:3], Hyq.getJointPosition(prefix, "hfe")[0:3])
	l2 = tools.euclideanDist(Hyq.getJointPosition(prefix, "hfe")[0:3], Hyq.getJointPosition(prefix, "kfe")[0:3])
	l3 = tools.euclideanDist(Hyq.getJointPosition(prefix, "kfe")[0:3], Hyq.getJointPosition(prefix, "foot")[0:3])

	P0T, R0T = transformXworldToX0(prefix, XworldDes) # R0T == R03
	P03 = tools.addMatrices([[P0T[0]], [P0T[1]], [P0T[2]]], tools.multiplyMatrices(R0T, [[l3], [0], [0]]), '-')
	T03 = [[R0T[0][0], R0T[0][1], R0T[0][2], P03[0][0]],
		   [R0T[1][0], R0T[1][1], R0T[1][2], P03[1][0]],
		   [R0T[2][0], R0T[2][1], R0T[2][2], P03[2][0]],
		   [0,         0,         0,         1        ]]

	c1 = tools.math.cos(q1)
	s1 = tools.math.sin(q1)

	# eq3
	c2 = (c1*T03[0][3] + s1*T03[2][3] - l1)/l2
	print "c2 : " + str(c2)

	# eq9
	s2 = T03[1][3]/l2
	print "s2 : " + str(s2)

	# q2
	q2 = tools.math.atan2(s2, c2)

	print "atan2(s2, c2) : " + str(q2)
	print "acos(c2) : " + str(tools.math.acos(c2))
	print "asin(s2) : " + str(tools.math.asin(s2))

def mgiq3test(prefix, q1, q2, XworldDes, epsi1, epsi2):
	prefixes = ["lh", "lf", "rh", "rf"]
	if prefix not in prefixes:
		return "Not a valid prefix"
	epsi1 = 1.0 if epsi1 >= 0 else -1.0
	epsi2 = 1.0 if epsi2 >= 0 else -1.0

	l1 = tools.euclideanDist(Hyq.getJointPosition(prefix, "haa")[0:3], Hyq.getJointPosition(prefix, "hfe")[0:3])
	l2 = tools.euclideanDist(Hyq.getJointPosition(prefix, "hfe")[0:3], Hyq.getJointPosition(prefix, "kfe")[0:3])
	l3 = tools.euclideanDist(Hyq.getJointPosition(prefix, "kfe")[0:3], Hyq.getJointPosition(prefix, "foot")[0:3])

	P0T, R0T = transformXworldToX0(prefix, XworldDes) # R0T == R03
	P03 = tools.addMatrices([[P0T[0]], [P0T[1]], [P0T[2]]], tools.multiplyMatrices(R0T, [[l3], [0], [0]]), '-')
	T03 = [[R0T[0][0], R0T[0][1], R0T[0][2], P03[0][0]],
		   [R0T[1][0], R0T[1][1], R0T[1][2], P03[1][0]],
		   [R0T[2][0], R0T[2][1], R0T[2][2], P03[2][0]],
		   [0,         0,         0,         1        ]]

	c1 = tools.math.cos(q1)
	s1 = tools.math.sin(q1)
	c2 = tools.math.cos(q2)
	s2 = tools.math.sin(q2)

	# eq7
	den = (tools.math.pow(s2, 2) + tools.math.pow(c2, 2))
	s3_eq7 = (c2*T03[1][0] - epsi1*s2*tools.math.sqrt(den - tools.math.pow(T03[1][0], 2)))/den
	c3_eq7 = (s2*T03[1][0] + epsi1*c2*tools.math.sqrt(den - tools.math.pow(T03[1][0], 2)))/den
	q3_eq7 = tools.math.atan2(s3_eq7, c3_eq7)

	# eq1
	den = (tools.math.pow(c2, 2) + tools.math.pow(s2, 2))
	Z = c1*T03[0][0] + s1*T03[2][0]
	s3_eq1 = (-s2*Z + epsi2*c2*tools.math.sqrt(den - tools.math.pow(Z, 2)))/den
	c3_eq1 = (c2*Z + epsi2*s2*tools.math.sqrt(den - tools.math.pow(Z, 2)))/den
	q3_eq1 = tools.math.atan2(s3_eq1, c3_eq1)

	print "epsi1 : " + str(epsi1)
	print "epsi2 : " + str(epsi2)
	print str(q3_eq7) + " -- " + str(q3_eq1)

def testAllContacts(q = q_init[:]):
	q[3:7] = tools.buildQuaternion([0, 0, 1], 0, tools.AngleEnum.DEGREES)
	r(q)

	n = [0, 0, 1]

	# lh
	slh = state_alg.State(fullbody, -1, False, fullbody.getCurrentConfig())
	pos = fullbody.getJointPosition("lh_foot_joint")[0:3]
	slf, success = state_alg.addNewContact(slh, "lhleg", pos, n)
	if not success:
		print "failed to add lh contact"

	# lf
	pos = fullbody.getJointPosition("lf_foot_joint")[0:3]
	srf, success = state_alg.addNewContact(slf, "lfleg", pos, n)
	if not success:
		print "failed to add lf contact"

	# rf
	pos = fullbody.getJointPosition("rf_foot_joint")[0:3]
	srh, success = state_alg.addNewContact(srf, "rfleg", pos, n)
	if not success:
		print "failed to add rf contact"

	# rh
	pos = fullbody.getJointPosition("rh_foot_joint")[0:3]
	end_s, success = state_alg.addNewContact(srh, "rhleg", pos, n)
	if not success:
		print "failed to add rh contact"

	r(fullbody.getCurrentConfig())

	return end_s
'''

def testZMP(convexHSuppPolygon, centerOfMass, acceleration, display = False):
	stable = tools.isValidZMP(convexHSuppPolygon, centerOfMass, acceleration)
	cost = tools.evalZMP(convexHSuppPolygon, centerOfMass, acceleration)
	if display:
		print "Stable ? " + str(stable)
		print "Cost : " + str(cost)
	return stable, cost

print ""
print "---"
print ""

q = q_init[:]
q[7] += 0.3; q[10] += 0.3; q[13] -= 0.3; q[16] -= 0.3; r(q)

pos = []
pos.append(fullbody.getJointPosition("lh_foot_joint")[0:3])
pos.append(fullbody.getJointPosition("lf_foot_joint")[0:3])
pos.append(fullbody.getJointPosition("rf_foot_joint")[0:3])
pos.append(fullbody.getJointPosition("rh_foot_joint")[0:3])

suppoly = []
for p in pos:
	suppoly.append(tools.orthogonalProjection(p, [0, 0, 1, 0])[0:2])

chsp = tools.PointCloudsManager.convexHull2D(suppoly)

CoM = fullbody.client.basic.robot.getComPosition()

# ---

pas = -0.01

base_x_accel = 0.0
cost = testZMP(chsp, CoM, [base_x_accel, 0.0, 0.0])
optim_x_accel = base_x_accel + pas
next_cost = testZMP(chsp, CoM, [optim_x_accel, 0.0, 0.0])
while next_cost <= cost:
	cost = next_cost
	optim_x_accel += pas
	next_cost = testZMP(chsp, CoM, [optim_x_accel, 0.0, 0.0])

base_y_accel = 0.0
cost = testZMP(chsp, CoM, [0.0, base_y_accel, 0.0])
optim_y_accel = base_y_accel + pas
next_cost = testZMP(chsp, CoM, [0.0, optim_y_accel, 0.0])
while next_cost <= cost:
	cost = next_cost
	optim_y_accel += pas
	next_cost = testZMP(chsp, CoM, [0.0, optim_y_accel, 0.0])