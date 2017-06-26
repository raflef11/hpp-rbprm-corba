from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.gepetto import Viewer
from hpp.corbaserver import Client
from hpp.corbaserver.robot import Robot as Parent
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
import omniORB.any

class Robot (Parent):
	rootJointType = "freeflyer"
	packageName = "hpp-rbprm-corba"
	meshPackageName = "hpp-rbprm-corba"
	urdfName = "hrp2_trunk_flexible"
	urdfSuffix = ""
	srdfSuffix = ""
	def __init__(self, robotName, load = True):
		Parent.__init__(self, robotName, self.rootJointType, load)
		self.tf_root = "base_footprint"
		self.client.basic = Client()
		self.load = load

rootJointType = "freeflyer"
packageName = "hpp-rbprm-corba"
meshPackageName = "hpp-rbprm-corba"
urdfName = "hrp2_trunk_flexible"
urdfNameRom =  ["hrp2_larm_rom","hrp2_rarm_rom","hrp2_lleg_rom","hrp2_rleg_rom"]
urdfSuffix = ""
srdfSuffix = ""
vMax = 4.0
aMax = 6.0
extraDof = 6

rbprmBuilder = Builder()
rbprmBuilder.loadModel(urdfName, urdfNameRom, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
rbprmBuilder.setJointBounds("base_joint_xyz", [-2, 8, 0.0, 1.5, 0.25, 1.8])
rbprmBuilder.setJointBounds("CHEST_JOINT0", [0, 0])
rbprmBuilder.setJointBounds("CHEST_JOINT1", [-0.35, 0.1])
rbprmBuilder.setJointBounds("HEAD_JOINT0", [0, 0])
rbprmBuilder.setJointBounds("HEAD_JOINT1", [0, 0])

rbprmBuilder.setFilter(["hrp2_lleg_rom", "hrp2_rleg_rom"])
rbprmBuilder.setAffordanceFilter("hrp2_lleg_rom", ["Support"])
rbprmBuilder.setAffordanceFilter("hrp2_rleg_rom", ["Support"])

rbprmBuilder.boundSO3([-0.1, 0.1, -0.65, 0.65, -0.2, 0.2])
rbprmBuilder.client.basic.robot.setDimensionExtraConfigSpace(extraDof)
rbprmBuilder.client.basic.robot.setExtraConfigSpaceBounds([-vMax, vMax, -vMax, vMax, -vMax, vMax, 0, 0, 0, 0, 0, 0])
indexECS = rbprmBuilder.getConfigSize() - rbprmBuilder.client.basic.robot.getDimensionExtraConfigSpace()

ps = ProblemSolver(rbprmBuilder)
ps.client.problem.setParameter("aMax", omniORB.any.to_any(aMax))
ps.client.problem.setParameter("aMaxZ", omniORB.any.to_any(10.0))
ps.client.problem.setParameter("vMax", omniORB.any.to_any(vMax))
ps.client.problem.setParameter("tryJump", omniORB.any.to_any(1.0))
ps.client.problem.setParameter("sizeFootX", omniORB.any.to_any(0.24))
ps.client.problem.setParameter("sizeFootY", omniORB.any.to_any(0.14))
r = Viewer(ps)

from hpp.corbaserver.affordance.affordance import AffordanceTool
afftool = AffordanceTool()
afftool.setAffordanceConfig("Support", [0.5, 0.03, 0.00005])
afftool.loadObstacleModel(packageName, "plane_hole", "planning", r)
afftool.visualiseAffordances("Support", r, [0.25, 0.5, 0.5])
r.addLandmark(r.sceneName, 1)

q_init = rbprmBuilder.getCurrentConfig()
q_init[3:7] = [1, 0, 0, 0]
q_init[8] = -0.2
q_init[0:3] = [0.2, 0, 0.55]
r(q_init)

rbprmBuilder.setCurrentConfig(q_init)
q_goal = q_init[::]
q_goal[3:7] = [1, 0, 0, 0]
q_goal[8] = 0
q_goal[0:3] = [7, 0, 0.55]
r(q_goal)

ps.setInitialConfig(q_init)
ps.addGoalConfig(q_goal)

ps.client.problem.selectConFigurationShooter("RbprmShooter")
ps.client.problem.selectPathValidation("RbprmPathValidation", 0.05)

ps.selectSteeringMethod("RBPRMKinodynamic")
ps.selectDistance("KinodynamicDistance")
ps.selectPathPlanner("DynamicPlanner")

r(q_init)

ps.client.problem.prepareSolveStepByStep()
pbCl = rbprmBuilder.client.basic.problem

pbCl.directPath(q_init, q_goal, False)
pbCl.addEdgeToRoadmap(q_init, q_goal, 0, False)

ps.client.problem.finishSolveStepByStep()

from hpp.gepetto import PathPlayer
pp = PathPlayer(rbprmBuilder.client.basic, r)
pp.dt = 0.03
pp.speed = 0.1
pp.displayVelocityPath(0)
r.client.gui.setVisibility("path_0_root", "ALWAYS_ON_TOP")



q_far = q_init[::]
q_far[2] = -3
r(q_far)