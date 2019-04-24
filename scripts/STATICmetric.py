"""Loads the grasping model, moves the robot to the first grasp found to mug4
	and raises the mug

	I've editted /usr/local/lib/python2.7/dist-packages/openravepy/_openravepy_/interfaces/BaseManipulation.py
	to return 0 whenever the planner fails to find a valid trajectory
"""
from openravepy import *
import numpy as np, time
from math import fabs, sqrt
import pdb
import pickle

env=Environment()
env.Load('data/testwamcamera.env.xml')
env.SetViewer('qtcoin')
robot = env.GetRobots()[0]

dmatrix_file = 'D_MATRIX.pkl'
constraint_file = 'constraints.pkl'

f = open(dmatrix_file,'r')
D = pickle.load(f)
f.close()

g = open(constraint_file,'r')
validgrasps = pickle.load(g)
g.close()
#STATIC Algorithm
D = D[0:3]  #Ignore scores for mug4. D = 4 x 10 matrix
D = np.asarray(D,dtype=np.float32)
mu = np.sum(D,axis=0)/3 #find average of scores. Eq. 1 in paper
min_index = np.argmax(mu) #get index of constraint with least negative score


#This is the selected constraint. We'd use it to pick new mug
constraint = validgrasps[min_index] 


#Performing task with chosen constraint
target = env.GetKinBody('mug3')
gmodel = databases.grasping.GraspingModel(robot,target)
if not gmodel.load():
	gmodel.autogenerate()

#start timing
start = time.time()
gmodel.moveToPreshape(constraint)
Tgoal = gmodel.getGlobalGraspTransform(constraint, collisionfree=True)
basemanip = interfaces.BaseManipulation(robot)
traj = basemanip.MoveToHandPosition(matrices=[Tgoal],timelimit=15,
	outputtrajobj=True)
#end timing
duration = time.time() - start

score = 0

if traj != 0:
	for i in range(traj.GetNumWaypoints()-1):
		score = score + sqrt(fabs(sum(traj.GetWaypoint(i+1) - traj.GetWaypoint(i))))
		
else:
	print "INFEASIBLE GRASP!"

robot.WaitForController(0)
taskmanip = interfaces.TaskManipulation(robot)
taskmanip.CloseFingers()
robot.WaitForController(0)
# basemanip.MoveManipulator()

print "SCORE IS: %.4f"%(score*-1)
print "PLANNING DURATION: %.4f"%duration
time.sleep(10)




'''
RESULTS:
_________

SCORE: -6.1166
DURATION: 0.2982


'''