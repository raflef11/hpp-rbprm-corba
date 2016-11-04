import matplotlib
#~ matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from cwc import cone_optimization
from obj_to_constraints import ineq_from_file, rotate_inequalities
import numpy as np
import math
from numpy.linalg import norm
import time

import hpp.corbaserver.rbprm.data.com_inequalities as ine

ineqPath = ine.__path__[0] +"/"

# epsilon for testing whether a number is close to zero
_EPS = np.finfo(float).eps * 4.0

def quaternion_matrix(quaternion):
    q = np.array(quaternion, dtype=np.float64, copy=True)
    n = np.dot(q, q)
    if n < _EPS:
        return np.identity(4)
    q *= math.sqrt(2.0 / n)
    q = np.outer(q, q)
    return np.array([
        [1.0-q[2, 2]-q[3, 3],     q[1, 2]-q[3, 0],     q[1, 3]+q[2, 0], 0.0],
        [    q[1, 2]+q[3, 0], 1.0-q[1, 1]-q[3, 3],     q[2, 3]-q[1, 0], 0.0],
        [    q[1, 3]-q[2, 0],     q[2, 3]+q[1, 0], 1.0-q[1, 1]-q[2, 2], 0.0],
        [                0.0,                 0.0,                 0.0, 1.0]])

def __get_com(robot, config):
	save = robot.getCurrentConfig()
	robot.setCurrentConfig(config)
	com = robot.getCenterOfMass()
	robot.setCurrentConfig(save)
	return com

constraintsComLoaded = {}

lastspeed = np.array([0,0,0])

def __get_com_constraint(fullBody, state, config, limbsCOMConstraints, interm = False):
	global constraintsLoaded
	As = [];	bs = []
	fullBody.setCurrentConfig(config)
	contacts = []
	for i, v in limbsCOMConstraints.iteritems():
		if not constraintsComLoaded.has_key(i):
			constraintsComLoaded[i] = ineq_from_file(ineqPath+v['file'])
		contact = (interm and fullBody.isLimbInContactIntermediary(i, state)) or (not interm and fullBody.isLimbInContact(i, state))
		if contact:
			ineq = constraintsComLoaded[i]
			qEffector = fullBody.getJointPosition(v['effector'])
			tr = quaternion_matrix(qEffector[3:7])			
			tr[:3,3] = np.array(qEffector[0:3])
			ineq_r = rotate_inequalities(ineq, tr)
			As.append(ineq_r.A); bs.append(ineq_r.b);
			contacts.append(v['effector'])
	return [np.vstack(As), np.hstack(bs)]
		
def compute_state_info(fullBody,states, state_id, phase_dt, mu, computeCones, limbsCOMConstraints):
	init_com = __get_com(fullBody, states[state_id])
	end_com = __get_com(fullBody, states[state_id+1])
	p, N = fullBody.computeContactPoints(state_id)
	fly_time = phase_dt [1]
	support_time = phase_dt [0]
	t_end_phases = [0]
	[t_end_phases.append(t_end_phases[-1]+fly_time) for _ in range(len(p))]
	if(len(t_end_phases) == 4):
		t_end_phases[1] = support_time
		t_end_phases[2] = t_end_phases[1] + fly_time
		t_end_phases[3] = t_end_phases[2] + support_time
		t_end_phases = [float((int)(math.ceil(el*10.))) / 10. for el in t_end_phases]
	cones = None
	if(computeCones):
		cones = [fullBody.getContactCone(state_id, mu)[0]]
		if(len(p) > 2):
			cones.append(fullBody.getContactIntermediateCone(state_id, mu)[0])
		if(len(p) > len(cones)):
			cones.append(fullBody.getContactCone(state_id+1, mu)[0])		
	COMConstraints = None
	if(not (limbsCOMConstraints == None)):
		COMConstraints = [__get_com_constraint(fullBody, state_id, states[state_id], limbsCOMConstraints)]
		if(len(p) > 2):
			COMConstraints.append(__get_com_constraint(fullBody, state_id, states[state_id], limbsCOMConstraints, True))
		if(len(p) > len(COMConstraints)):
			COMConstraints.append(__get_com_constraint(fullBody, state_id + 1, states[state_id + 1], limbsCOMConstraints))
	return p, N, init_com, end_com, t_end_phases, cones, COMConstraints


def gen_trajectory(fullBody, states, state_id, computeCones = False, mu = 1, dt=0.2, phase_dt = [0.4, 1],
reduce_ineq = True, verbose = False, limbsCOMConstraints = None, profile = False, use_window = 0):	
	global lastspeed
	use_window = max(0, min(use_window,  (len(states) - 1) - (state_id + 2))) # can't use preview if last state is reached	
	assert( len(phase_dt) >= 2 +  use_window * 2 ), "phase_dt does not describe all phases"
	
	constraints = ['cones_constraint', 'end_reached_constraint','end_speed_constraint']
	#~ constraints = ['cones_constraint', 'end_reached_constraint','end_speed_constraint', 'com_kinematic_constraint']
	param_constraints = []	
	mass = fullBody.getMass()
	
	p, N, init_com, end_com, t_end_phases, cones, COMConstraints = compute_state_info(fullBody,states, state_id, phase_dt[:2], mu, computeCones, limbsCOMConstraints)
	if(use_window > 0):
		init_waypoint_time = int(np.round(t_end_phases[-1]/ dt)) - 1
		init_end_com = end_com[:]
	for w in range(1,use_window+1):
		waypoint = end_com[:]
		waypoint_time = int(np.round(t_end_phases[-1]/ dt)) - 1
		# trying not to apply constraint
		#~ param_constraints += [("waypoint_reached_constraint",(waypoint_time, waypoint))]
		p1, N1, init_com1, end_com1, t_end_phases1, cones1, COMConstraints1 = compute_state_info(fullBody,states, state_id+w, phase_dt[2*w:], mu, computeCones, limbsCOMConstraints)
		p+=p1;
		N+=N1;
		end_com = end_com1;
		cones += cones1;
		if(COMConstraints != None and COMConstraints1 != None):
			COMConstraints += COMConstraints1;
		t_end_phases += [t_end_phases[-1] + t for t in t_end_phases1[1:]]
	
	if (not profile):
			print "num cones ", len(cones)
			print "end_phases", t_end_phases
	
	timeelapsed = 0		
	if (profile):
		start = time.clock() 
	var_final, params = cone_optimization(p, N, [init_com + lastspeed.tolist(), end_com + [0,0,0]], t_end_phases[1:], dt, cones, COMConstraints, mu, mass, 9.81, reduce_ineq, verbose,
	constraints, param_constraints)	
	#~ print "end_com ", end_com , "computed end come", var_final['c'][-1], var_final['c_end']
	if (profile):
		end = time.clock() 
		timeelapsed = (end - start) * 1000
		#~ print "solving time", timeelapsed
	if(use_window > 0):
		var_final['c_old'] = var_final['c'][:]
		var_final['dc_old'] = var_final['dc'][:]
		var_final['ddc_old'] = var_final['ddc'][:]
		var_final['c'] = var_final['c'][:init_waypoint_time+1]
		var_final['dc'] = var_final['dc'][:init_waypoint_time+1]
		var_final['ddc'] = var_final['ddc'][:init_waypoint_time+1]
		params["t_init_phases"] = params["t_init_phases"][:-3*use_window]
		print "trying to project on com (from, to) ", init_end_com, var_final['c'][-1]
		if (fullBody.projectStateToCOM(state_id+1, (var_final['c'][-1]).tolist())):
			#~ print "PROJECTED", init_end_com, var_final['c'][-1]
			states[state_id+1] = fullBody.getConfigAtState(state_id+1) #updating config from python side)
			lastspeed = var_final['dc'][init_waypoint_time]		
			print "init speed", lastspeed
		else:
			print "reached com is not good, restarting problem with 0 window"
			return gen_trajectory(fullBody, states, state_id, computeCones, mu, dt, phase_dt[:2], reduce_ineq, verbose, limbsCOMConstraints, profile, use_window = 0)			
	else:		
		if norm(np.array(var_final['c'][-1]) - np.array(__get_com(fullBody, states[state_id+1]))) > 0.00001:
			print "error in com desired: obtained ", __get_com(fullBody, states[state_id+1]), var_final['c'][-1]
		lastspeed = np.array([0,0,0])
		
	return var_final, params, timeelapsed

def draw_trajectory(fullBody, states, state_id, computeCones = False, mu = 1,  dt=0.2, phase_dt = [0.4, 1], reduce_ineq = True, verbose = False, limbsCOMConstraints = None, use_window = 0):
	var_final, params, elapsed = gen_trajectory(fullBody, states, state_id, computeCones, mu , dt, phase_dt, reduce_ineq, verbose, limbsCOMConstraints, False, use_window = use_window)
	p, N = fullBody.computeContactPoints(state_id)
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	n = 100
	points = var_final['x']
	xs = [points[i] for i in range(0,len(points),6)]
	ys = [points[i] for i in range(1,len(points),6)]
	zs = [points[i] for i in range(2,len(points),6)]
	ax.scatter(xs, ys, zs, c='b')

	colors = ["r", "b", "g"]
	#print contact points of first phase
	for id_c, points in enumerate(p):
		xs = [point[0] for point in points]
		ys = [point[1] for point in points]
		zs = [point[2] for point in points]
		ax.scatter(xs, ys, zs, c=colors[id_c])
		
	ax.set_xlabel('X Label')
	ax.set_ylabel('Y Label')
	ax.set_zlabel('Z Label')

	#~ plt.show()
	plt.savefig('/tmp/c'+ str(state_id)+ '.png')
	
	print "plotting speed "
	print "end target ",  params['x_end']
	fig = plt.figure()
	ax = fig.add_subplot(111)
	if(use_window > 0):
		#~ points = var_final['dc_old']
		points = var_final['dc']
	else:
		points = var_final['dc']
		
	#~ print "points", points
	ys = [norm(el) * el[0] / abs(el[0]+ _EPS) for el in points]
	xs = [i * params['dt'] for i in range(0,len(points))]
	ax.scatter(xs, ys, c='b')


	#~ plt.show()
	plt.savefig('/tmp/dc'+ str(state_id)+ '.png')
	
	print "plotting acceleration "
	fig = plt.figure()
	ax = fig.add_subplot(111)
	if(use_window > 0):
		#~ points = var_final['ddc_old']
		points = var_final['ddc']
	else:
		points = var_final['ddc']
	ys = [norm(el) * el[0] / abs(el[0]+ _EPS) for el in points]
	xs = [i * params['dt'] for i in range(0,len(points))]
	ax.scatter(xs, ys, c='b')


		#~ plt.show()
	plt.savefig('/tmp/ddc'+ str(state_id)+ '.png')
	
	print "plotting Dl "
	fig = plt.figure()
	ax = fig.add_subplot(111)
	points = var_final['dL']
	ys = [norm(el) * el[0] / abs(el[0]+ _EPS) for el in points]
	xs = [i * params['dt'] for i in range(0,len(points))]
	ax.scatter(xs, ys, c='b')


	#~ plt.show()
	plt.savefig('/tmp/dL'+ str(state_id)+ '.png')
	return var_final, params, elapsed
	
def __cVarPerPhase(var, dt, t, final_state, addValue):
	varVals = addValue + [v.tolist() for v in final_state[var]]
	varPerPhase = [[varVals[(int)(round(t_id/dt)) ] for t_id in np.arange(t[index],t[index+1]-_EPS,dt)] for index, _ in enumerate(t[:-1])  ]
	varPerPhase[2].append(varVals[-1])
	if(not var == "ddc"):
		assert len(varVals) == len(varPerPhase[0]) + len(varPerPhase[1]) + len(varPerPhase[2]), mess
		
	if var == "dc":
		varPerPhase[2] = varPerPhase[2][:-1] # not relevant for computation
	else:
		varPerPhase[0].append(varPerPhase[1][0]) # end pos of state is the same as the previous one
		varPerPhase[1].append(varPerPhase[2][0])
	if var == "ddc": #acceleration: remove first
		varPerPhase = [v[1:] for v in varPerPhase]
		assert len(final_state[var]) == len(varPerPhase[0]) + len(varPerPhase[1]) + len(varPerPhase[2]), "incorrect num of ddc"		
	return varPerPhase
	
def __optim__threading_ok(fullBody, states, state_id, computeCones = False, mu = 1, dt =0.1, phase_dt = [0.4, 1], reduce_ineq = True,
 num_optims = 0, draw = False, verbose = False, limbsCOMConstraints = None, profile = False, use_window = 0):
	print "callgin gen ",state_id
	if(draw):
		res = draw_trajectory(fullBody, states, state_id, computeCones, mu, dt, phase_dt, reduce_ineq, verbose, limbsCOMConstraints, use_window)		
	else:
		res = gen_trajectory(fullBody, states, state_id, computeCones, mu, dt, phase_dt, reduce_ineq, verbose, limbsCOMConstraints, profile, use_window)
	t = res[1]["t_init_phases"];
	dt = res[1]["dt"];
	final_state = res[0]
	c0 =  res[1]["x_init"][0:3]
	dc0 = res[1]["x_init"][3:7]
	comPosPerPhase = __cVarPerPhase('c'  , dt, t, final_state, [c0])
	comVelPerPhase = __cVarPerPhase('dc' , dt, t, final_state, [dc0])
	comAccPerPhase = __cVarPerPhase('ddc', dt, t, final_state, [[0,0,0]])
		
	#now compute com trajectorirs
	comTrajIds = [fullBody.generateComTraj(comPosPerPhase[i], comVelPerPhase[i], comAccPerPhase[i], dt) for i in range(0,3)]
	return comTrajIds, res[2], final_state #res[2] is timeelapsed

def solve_com_RRT(fullBody, states, state_id, computeCones = False, mu = 1, dt =0.1, phase_dt = [0.4, 1],
reduce_ineq = True, num_optims = 0, draw = False, verbose = False, limbsCOMConstraints = None, profile = False, use_window = 0, trackedEffectors = []):
	comPosPerPhase, timeElapsed, final_state = __optim__threading_ok(fullBody, states, state_id, computeCones, mu, dt, phase_dt,
	reduce_ineq, num_optims, draw, verbose, limbsCOMConstraints, profile, use_window)
	print "done. generating state trajectory ",state_id	
	paths_ids = [int(el) for el in fullBody.comRRTFromPos(state_id,comPosPerPhase[0],comPosPerPhase[1],comPosPerPhase[2],num_optims)]
	print "done. computing final trajectory to display ",state_id, "path ids ", paths_ids[-1], " ," , paths_ids[:-1]
	return paths_ids[-1], paths_ids[:-1], timeElapsed, final_state
	
def solve_effector_RRT(fullBody, states, state_id, computeCones = False, mu = 1, dt =0.1, phase_dt = [0.4, 1],
reduce_ineq = True, num_optims = 0, draw = False, verbose = False, limbsCOMConstraints = None, profile = False, use_window = 0, trackedEffectors = []):
	comPosPerPhase, timeElapsed, final_state = __optim__threading_ok(fullBody, states, state_id, computeCones, mu, dt, phase_dt,
	reduce_ineq, num_optims, draw, verbose, limbsCOMConstraints, profile, use_window)
	print "done. generating state trajectory ",state_id		
	if(len(trackedEffectors) == 0):
		paths_ids = [int(el) for el in fullBody.effectorRRT(state_id,comPosPerPhase[0],comPosPerPhase[1],comPosPerPhase[2],num_optims)]
	else:
		print "handling extra effector constraints"
		refPathId = trackedEffectors[0]; path_start = trackedEffectors[1]; path_to  = trackedEffectors[2]; effectorstracked = trackedEffectors[3]
		paths_ids = [int(el) for el in fullBody.effectorRRTFromPath(state_id, refPathId, path_start, path_to, comPosPerPhase[0],comPosPerPhase[1],comPosPerPhase[2],num_optims, effectorstracked)]
	print "done. computing final trajectory to display ",state_id, "path ids ", paths_ids[-1], " ," , paths_ids[:-1]
	return paths_ids[-1], paths_ids[:-1], timeElapsed, final_state
	