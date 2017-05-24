from hpp.gepetto import Viewer
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.affordance.affordance import AffordanceTool
from hyq_ref_pose import hyq_ref
import toolbox as tools

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

	# HYQMGD (Currently not validated)
	# Compute the MGD of the Hyq
	#
	# @param [In] prefix The prefix of the considered joint in order to identify it
	# @param [In] q1 The haa configuration
	# @param [In] q2 The hfe configuration
	# @param [In] q3 The kfe configuration
	#
	# @return The position of the foot knwowing the configuration of its limb; The transform matrix between world frame and the last joint (kfe)
	@staticmethod
	def HyqMGD(prefix):
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

		if (prefix == prefixes["lh"]) or (prefix == prefixes["lf"]): # one of the limbs on the left
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
		return tools.multiplyMatrices(Tworld_0, X0), tools.multiplyMatrices(Tworld_0, T03)

# MGI of Hyq
def HyqMGI(prefix):
	pass

# set a end-effector position
def setEndEffectorPosition(name, pos):
	pass

# Test MGD Hyq
def testMGD(prefix):
	prefixes = ["lh", "lf", "rh", "rf"]
	if prefix not in prefixes:
		return "Not a valid prefix"
	Xreal = Hyq.getJointPosition(prefix, "foot")[0:3]
	print prefix + "_foot_joint position from model : " + str(Xreal)
	Xot, _ = Hyq.HyqMGD(prefix)
	X = [Xot[0][0], Xot[1][0], Xot[2][0]]
	print prefix + "_foot_joint position from MGD : " + str(X)
	print "errorX : " + str(abs(X[0] - Xreal[0]))
	print "errorY : " + str(abs(X[1] - Xreal[1]))
	print "errorZ : " + str(abs(X[2] - Xreal[2]))
	print "errorDist : " + str(tools.euclideanDist(X, Xreal))

#  test each limb MGD
def eachLimbMGD():
	print "---"
	for p in ["lh", "lf", "rh", "rf"]:
		testMGD(p)
		print "---"