from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer
import omniORB.any
import test_hrp2_path as tp
import time

packageName = "hrp2_14_description"
meshPackageName = "hrp2_14_description"
rootJointType = "freeflyer"

urdfName = "hrp2_14"
urdfSuffix = "_reduced"
srdfSuffix = ""
pId = 0
fullBody = FullBody()

fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds("base_joint_xyz", [-2, 8, 0.0, 1.5, 0.25, 1.8])
fullBody.client.basic.robot.setDimensionExtraConfigSpace(tp.extraDof)

ps = tp.ProblemSolver(fullBody)
ps.client.problem.setParameter("aMax", omniORB.any.to_any(tp.aMax))
ps.client.problem.setParameter("vMax", omniORB.any.to_any(tp.vMax))
r = tp.Viewer(ps, viewerClient = tp.r.client)

rLegId = "hrp2_rleg_rom"
rLeg = "RLEG_JOINT0"
rLegOffset = [0, 0, -0.105]
rLegNormal = [0, 0, 1]
rLegx = 0.09
rLegy = 0.05
#fullBody.addLimb(rLegId, rLeg, "", rLegOffset, rLegNormal, rLegx, rLegy, 20000, "ZMP", 0.1)
fullBody.addLimb(rLegId, rLeg, "", rLegOffset, rLegNormal, rLegx, rLegy, 20000, "dynamic", 0.1)

lLegId = "hrp2_lleg_rom"
lLeg = "LLEG_JOINT0"
lLegOffset = [0, 0, -0.105]
lLegNormal = [0, 0, 1]
lLegx = 0.09
lLegy = 0.05
#fullBody.addLimb(lLegId, lLeg, "", lLegOffset, lLegNormal, lLegx, lLegy, 20000, "ZMP", 0.1)
fullBody.addLimb(lLegId, lLeg, "", lLegOffset, lLegNormal, lLegx, lLegy, 20000, "dynamic", 0.1)

q_0 = fullBody.getCurrentConfig()
q_init =[0.1, 0.0, 0.648702, 1.0, 0.0 , 0.0, 0.0,0.0, 0.0, 0.0, 0.0,0.261799388,  0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.17,0.261799388, -0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.17,0.0, 0.0, -0.453785606, 0.872664626, -0.41887902, 0.0,0.0, 0.0, -0.453785606, 0.872664626, -0.41887902, 0.0,0,0,0,0,0,0]
r (q_init)
fullBody.setCurrentConfig(q_init)

configSize = fullBody.getConfigSize() - fullBody.client.basic.robot.getDimensionExtraConfigSpace()

q_init = fullBody.getCurrentConfig()
q_init[0:7] = tp.ps.configAtParam(pId, 0.01)[0:7]
q_goal = fullBody.getCurrentConfig()
q_goal[0:7] = tp.ps.configAtParam(pId, tp.ps.pathLength(pId))[0:7]
dir_init = tp.ps.configAtParam(pId, 0.01)[tp.indexECS:tp.indexECS+3]
acc_init = tp.ps.configAtParam(pId, 0.01)[tp.indexECS+3:tp.indexECS+6]
dir_goal = tp.ps.configAtParam(pId, tp.ps.pathLength(pId)-0.01)[tp.indexECS:tp.indexECS+3]
acc_goal = [0, 0, 0]

robThreshold = 3

q_init[configSize:configSize+3] = dir_init[::]
q_init[configSize+3:configSize+6] = acc_init[::]
q_goal[configSize:configSize+3] = dir_goal[::]
q_goal[configSize+3:configSize+6] = acc_goal[::]

q_init[2] += 0.1; q_goal[2] += 0.1

fullBody.setStaticStability(False)

# Randomly generate a contact configuration at q_init
fullBody.setCurrentConfig(q_init)
r(q_init)
q_init = fullBody.generateContacts(q_init, dir_init, acc_init, robThreshold)
r(q_init)

# Randomly generate a contact configuration at q_goal
fullBody.setCurrentConfig(q_goal)
q_goal = fullBody.generateContacts(q_goal, dir_goal, acc_goal, robThreshold)
r(q_goal)

r(q_init)

fullBody.setStartState(q_init, [rLegId, lLegId])
fullBody.setEndState(q_goal, [rLegId, lLegId])

fullBody.runLimbSampleAnalysis(rLegId, "jointLimitsDistance", True)
fullBody.runLimbSampleAnalysis(lLegId, "jointLimitsDistance", True)

fullBody.setMaxContactBreaks(1)

from hpp.gepetto import PathPlayer
pp = PathPlayer(fullBody.client.basic, r)

import fullBodyPlayerHrp2

# But its "Threshold" and not "Treshold"
configs = fullBody.interpolate(0.05, pathId = pId, robustnessTreshold = 1, filterStates = True)
print "Number of configs : ", len(configs)
r(configs[-1])

player = fullBodyPlayerHrp2.Player(fullBody, pp, tp, configs, draw = False, use_window = 1, optim_effector = True, use_velocity = True, pathId = pId)

player.displayContactPlan(2.0)