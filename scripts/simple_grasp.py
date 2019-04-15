"""Loads the grasping model and moves the robot to the first grasp found
"""
from openravepy import *
import numpy, time
# import pdb
env=Environment()
env.Load('data/lab1.env.xml')
env.SetViewer('qtcoin')
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
basemanip = interfaces.BaseManipulation(robot)
start = time.time()
basemanip.MoveToHandPosition(matrices=[Tgoal])
elapsed = time.time()-start
print "TIME ELAPSED: %0.4f"%elapsed
robot.WaitForController(0)
taskmanip = interfaces.TaskManipulation(robot)
taskmanip.CloseFingers()
robot.WaitForController(0)
# robot.Grab(target)
# robot.WaitForController(0)
# basemanip.MoveManipulator(initialvalues)
# time.sleep(10)