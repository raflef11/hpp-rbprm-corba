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

	@staticmethod
	def getCoM():
		return fullbody.client.basic.robot.getCenterOfMass()[:]

	@staticmethod
	def getCHSP():
		pos = []
		pos.append(fullbody.getJointPosition("lh_foot_joint")[0:3])
		pos.append(fullbody.getJointPosition("lf_foot_joint")[0:3])
		pos.append(fullbody.getJointPosition("rf_foot_joint")[0:3])
		pos.append(fullbody.getJointPosition("rh_foot_joint")[0:3])

		suppoly = []
		for p in pos:
			suppoly.append(tools.orthogonalProjection(p, [0, 0, 1, 0])[0:2])

		return tools.PointCloudsManager.convexHull2D(suppoly)

# -----
# Tests
# -----

def testZMP(convexHSuppPolygon, centerOfMass, acceleration, display = False):
	stable = tools.isValidZMP(convexHSuppPolygon, centerOfMass, acceleration)
	cost = tools.evalZMP(convexHSuppPolygon, centerOfMass, acceleration)
	if display:
		print "Stable ? " + str(stable)
		print "Cost : " + str(cost)
	return stable, cost

def scenarioTestZMP(accel = [0.0, 0.0, 0.0]):
	import time

	print "Ready to begin..."
	raw_input()

	print "Constant acceleration considered for this scenario : " + str(accel) + "\n"
	time.sleep(1.0)

	q = q_init[:]
	q[0:3] = [-2.0, 0.0, 0.59]
	q[3:7] = tools.buildQuaternion([0.0, 0.0, 1.0], 0.0, tools.AngleEnum.DEGREES)
	q[7] = q[10] = q[13] = q[16] = 0.0
	q[8] = q[11] = q[14] = q[17] = 0.4
	q[9] = q[12] = q[15] = q[18] = 0.7

	print "--- Base configuration:"
	r(q)
	testZMP(Hyq.getCHSP(), Hyq.getCoM(), accel, True)

	print "Press --> Enter <-- to continue..."
	raw_input()

	q[14] = -1.0
	q[15] = 1.58

	print "--- Transitional configuration:"
	r(q)
	testZMP(Hyq.getCHSP(), Hyq.getCoM(), accel, True)

	print "Press --> Enter <-- to continue..."
	raw_input()

	q[8] = -1.0
	q[9] = 1.58

	print "--- Middle configuration:"
	r(q)
	testZMP(Hyq.getCHSP(), Hyq.getCoM(), accel, True)

	print "Press --> Enter <-- to continue..."
	raw_input()

	q[17] = -0.3
	q[18] = 1.44

	print "--- Transitional configuration:"
	r(q)
	testZMP(Hyq.getCHSP(), Hyq.getCoM(), accel, True)

	print "Press --> Enter <-- to continue..."
	raw_input()

	q[11] = -0.3
	q[12] = 1.44

	print "--- Final configuration:"
	r(q)
	testZMP(Hyq.getCHSP(), Hyq.getCoM(), accel, True)

	print "--- Finished ---"

print ""
print "---"
print ""
