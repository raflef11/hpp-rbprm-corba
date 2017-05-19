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

# MGD of Hyq
def HyqMGD(haaPos, q0, q1, q2): # Currently not validated
	base = []
	for v in haaPos:
		base.append([v])
	base.append([1])

	c0 = tools.math.cos(-q0); s0 = tools.math.sin(-q0) # -q0 because the model uses a q0 in the opposite direction
	c1 = tools.math.cos(q1); s1 = tools.math.sin(q1)
	c2 = tools.math.cos(q2); s2 = tools.math.sin(q2)
	l1 = 0.082; l2 = 0.35; l3 = 0.35
	
	'''
	T01 = [[1, 0,  0,   0],
		   [0, c0, -s0, 0],
		   [0, s0, c0, -l1],
		   [0, 0,  0,   1]]

	T12 = [[c1,  0, s1, l2],
		   [0,   1, 0,  0],
		   [-s1, 0, c1, 0],
		   [0,   0, 0,  1]]

	T23 = [[c2,  0, s2, 0],
		   [0,   1, 0,  0],
		   [-s2, 0, c2, l3],
		   [0,   0, 0,  1]]
	'''

	T01 = [[1, 0,  0,   0],
		   [0, c0, -s0, l1*s0],
		   [0, s0, c0,  -l1*c0],
		   [0, 0,  0,   1]]

	T12 = [[c1,  0, s1, -l2*s1],
		   [0,   1, 0,  0],
		   [-s1, 0, c1, -l2*c1],
		   [0,   0, 0,  1]]

	T23 = [[c2,  0, s2, l3*c2],
		   [0,   1, 0,  0],
		   [-s2, 0, c2, -l3*s2],
		   [0,   0, 0,  1]]

	T03 = tools.multiplyMatrices(tools.multiplyMatrices(T01, T12), T23)
	T30 = tools.inverseHomogeneousMatrix(T03)

	return tools.multiplyMatrices(T30, base) # the end-effector (foot) position

# MGI of Hyq
def HyqMGI(haaPos, footPos):
	pass

# set a end-effector position
def setEndEffectorPosition(name, pos):
	baseJoints = {7: "lh_haa_joint", 10: "rh_haa_joint", 13: "lf_haa_joint", 16: "rf_haa_joint"}
	endJoints = {"lh_foot_joint": 7, "rh_foot_joint": 10, "lf_foot_joint": 13, "rf_foot_joint": 16}
	if name in endJoints.keys():
		q = fullbody.getCurrentConfig()
		index = endJoints[name]
		footPos = pos
		haaPos = fullBody.getJointPosition(baseJoints[index])[0:3]
		qend = HyqMGI(haaPos, footPos)
		q[index:index+3] = qend[:]
		r(q)

# Test MGD Hyq
haaPos = fullbody.getJointPosition("lh_haa_joint")[0:3]
footPos = fullbody.getJointPosition("lh_foot_joint")[0:3]
qlh = fullbody.getCurrentConfig()[7:10]
print "MGD : " + str(HyqMGD(haaPos, qlh[0], qlh[1], qlh[2]))
print "foot : " + str(footPos)