"""Loads the grasping model, moves the robot to the first grasp found to mug4
	and raises the mug

	I've editted /usr/local/lib/python2.7/dist-packages/openravepy/_openravepy_/interfaces/BaseManipulation.py
	to return 0 whenever the planner fails to find a valid trajectory
"""
from openravepy import *
import numpy, time
from math import fabs
# import pdb

env=Environment()
env.Load('data/lab1.env.xml')
# env.SetViewer('qtcoin')
robot = env.GetRobots()[0]
target = env.GetKinBody('mug4')
# pdb.set_trace()
gmodel = databases.grasping.GraspingModel(robot,target)
if not gmodel.load():
    gmodel.autogenerate()

initialvalues = robot.GetDOFValues(gmodel.manip.GetArmIndices())
validgrasps, validindicees = gmodel.computeValidGrasps(returnnum=1)
gmodel.moveToPreshape(validgrasps[0])
Tgoal = gmodel.getGlobalGraspTransform(validgrasps[0],collisionfree=True)
falsegoal = Tgoal 
basemanip = interfaces.BaseManipulation(robot)
start = time.time()

#for evaluating score of trajectory
traj = basemanip.MoveToHandPosition(matrices=[falsegoal], maxiter= 22,execute=False, outputtrajobj=True)
'''
or below for constructing binary D matrix
traj = basemanip.MoveToHandPosition(matrices=[falsegoal], maxiter= 10000,execute=False)
'''
elapsed = time.time()-start
print "TIME ELAPSED: %0.4f"%elapsed
score = 0

#Score function based on equation 4 in BOX paper
if traj != 0:
	for i in range(traj.GetNumWaypoints()-1):
		# print repr(traj.GetWaypoint(i))
		score = score + fabs(sum(traj.GetWaypoint(i+1) - traj.GetWaypoint(i)))

	score *=-1   #Negative score values are feasible scores. Goal is to minimize score

else:
	score = -1000  #infeasible plan. Score is given a highly negative value
print "SCORE IS: %.3f"%score

robot.WaitForController(0)
taskmanip = interfaces.TaskManipulation(robot)
taskmanip.CloseFingers()
robot.WaitForController(0)
robot.Grab(target)
robot.WaitForController(0)
basemanip.MoveManipulator(initialvalues)
# time.sleep(10)

'''
Time for entire planning: 5.2016 seconds
Score for plan is consistently -27.631 after anything above 22 max iterations

'''