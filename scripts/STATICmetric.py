"""Loads the grasping model, moves the robot to the first grasp found to mug4
	and raises the mug

	I've editted /usr/local/lib/python2.7/dist-packages/openravepy/_openravepy_/interfaces/BaseManipulation.py
	to return 0 whenever the planner fails to find a valid trajectory
"""
from openravepy import *
import numpy, time
from math import fabs, sqrt
import pdb

env=Environment()
env.Load('data/wam_cabinet.env.xml')
env.SetViewer('qtcoin')
robot = env.GetRobots()[0]
# pdb.set_trace()
target = env.GetKinBody('plasticmugb1')
# pdb.set_trace()
start = time.time()
gmodel = databases.grasping.GraspingModel(robot,target)
gmodel.autogenerate()
# if not gmodel.load():
#     gmodel.autogenerate()

initialvalues = robot.GetDOFValues(gmodel.manip.GetArmIndices())
validgrasps, validindicees = gmodel.computeValidGrasps(returnnum=2)
gmodel.moveToPreshape(validgrasps[0])
Tgoal = gmodel.getGlobalGraspTransform(validgrasps[0],collisionfree=True)
falsegoal = Tgoal 
basemanip = interfaces.BaseManipulation(robot)


#for evaluating score of trajectory
traj = basemanip.MoveToHandPosition(matrices=[Tgoal],timelimit=0, outputtrajobj=True)
'''
or below for constructing binary D matrix
traj = basemanip.MoveToHandPosition(matrices=[falsegoal], maxiter= 10000,execute=False)
'''
elapsed = time.time()-start


print "TIME ELAPSED: %0.4f"%elapsed
scores = []; times = []
total_duration = traj.GetDuration()
duration_array = []

# for i in range(0,total_duration, 1/total_duration):
# 	duration_array.append(i)


score = 0


#Score function based on equation 4 in BOX paper
if traj != 0:
	# for timed in duration_array:
	# 	score = score + fabs(sum(traj.GetWaypoint(i+1) - traj.GetWaypoint(i)))
	# 	scores.append(score*-1)

	for i in range(traj.GetNumWaypoints()-1):
		# print repr(traj.GetWaypoint(i))
		score = score + sqrt(fabs(sum(traj.GetWaypoint(i+1) - traj.GetWaypoint(i))))

	score *=-1   #Negative score values are feasible scores. Goal is to minimize scores

else:
	score = -1000  #infeasible plan. Score is given a highly negative value
print "SCORE IS: %.3f"%score
print "NUM WAYPOINTS: %d"%traj.GetNumWaypoints()
# pdb.set_trace()
robot.WaitForController(0)
taskmanip = interfaces.TaskManipulation(robot)
taskmanip.CloseFingers()
robot.WaitForController(0)
robot.Grab(target)
robot.WaitForController(0)
basemanip.MoveManipulator(initialvalues)
time.sleep(10)

'''
Time for entire planning:  579 seconds
Score for plan is consistently -27.631 after anything above 21 max iterations
25 waypoints

'''