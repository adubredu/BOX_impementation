"""Loads the grasping model, moves the robot to the first grasp found to mug4
	and raises the mug
"""
from openravepy import *
import numpy, time
import pdb
env=Environment()
env.Load('data/wam_cabinet.env.xml')
env.SetViewer('qtcoin')
robot = env.GetRobots()[0]
target = env.GetKinBody('mug4')
time.sleep(300)
# pdb.set_trace()
begin = time.time()
gmodel = databases.grasping.GraspingModel(robot,target)
if not gmodel.load():
    gmodel.autogenerate()
initialvalues = robot.GetDOFValues(gmodel.manip.GetArmIndices())

validgrasps, validindicees = gmodel.computeValidGrasps(returnnum=1)
end = time.time() - begin
print "COMPUTE GRASP TIME: %0.4f"%end
# pdb.set_trace()
gmodel.moveToPreshape(validgrasps[0])
Tgoal = gmodel.getGlobalGraspTransform(validgrasps[0],collisionfree=True)
basemanip = interfaces.BaseManipulation(robot,plannername='BiRRT')
time.sleep(10)
start = time.time()
basemanip.MoveToHandPosition(matrices=[Tgoal],timelimit=2)
elapsed = time.time()-start
print "TIME ELAPSED: %0.4f"%elapsed
robot.WaitForController(0)
taskmanip = interfaces.TaskManipulation(robot)
taskmanip.CloseFingers()
robot.WaitForController(0)
robot.Grab(target)
robot.WaitForController(0)
basemanip.MoveManipulator(initialvalues)
time.sleep(10)