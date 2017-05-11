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

# set fullbody configuration method from a base config and only one modification (tests)
def setQ(r, q, i, val) :
	q_tmp = q[:]
	i = i%len(q)
	q_tmp[i] = val
	r(q_tmp)
	return q_tmp