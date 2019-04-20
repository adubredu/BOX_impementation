"""
Creates the D-matrix
"""
from openravepy import *
import numpy, time
from math import fabs, sqrt
import pdb
import pickle

env=Environment()
env.Load('data/wam_cabinet.env.xml')
env.SetViewer('qtcoin')
robot = env.GetRobots()[0]
# pdb.set_trace()

# pdb.set_trace()

constraint_file = "constraints.pkl"
num_constraints = 10

target = env.GetKinBody('plasticmugb3')
gmodel = databases.grasping.GraspingModel(robot,target)
gmodel.load()
initialvalues = robot.GetDOFValues(gmodel.manip.GetArmIndices())

'''
######################
#GENERATE VALID GRASPS (constraints)
######################

gmodel = databases.grasping.GraspingModel(robot,target)#.autogenerate()
if not gmodel.load():
	gmodel.autogenerate()

initialvalues = robot.GetDOFValues(gmodel.manip.GetArmIndices())
validgrasps, validindices = gmodel.computeValidGrasps(returnnum=num_constraints)


f = open(constraint_file,'wb')
pickle.dump(validgrasps, f)
f.close()
'''
f = open(constraint_file, 'r')
validgrasps = pickle.load(f)
D = []

for j in range(4):
	target = env.GetKinBody('plasticmugb'+str(j+1))
	gmodel = databases.grasping.GraspingModel(robot,target)
	gmodel.load()
	# initialvalues = robot.GetDOFValues(gmodel.manip.GetArmIndices())
	scores = []
	for i in range(num_constraints):

		gmodel.moveToPreshape(validgrasps[i])
		Tgoal = gmodel.getGlobalGraspTransform(validgrasps[i],collisionfree=True)
		falsegoal = Tgoal 
		basemanip = interfaces.BaseManipulation(robot)
		traj = basemanip.MoveToHandPosition(matrices=[Tgoal],timelimit=15, outputtrajobj=True)

		

		score = 0


		#Score function based on equation 4 in BOX paper
		if traj != 0:
			for i in range(traj.GetNumWaypoints()-1):
				score = score + sqrt(fabs(sum(traj.GetWaypoint(i+1) - traj.GetWaypoint(i))))

			score *=-1   #Negative score values are feasible scores. Goal is to minimize scores
			scores.append(score)
		else:
			score = -1000  #infeasible plan. Score is given a highly negative value
			scores.append(score)
		print('append')

		robot.WaitForController(0)
		taskmanip = interfaces.TaskManipulation(robot)
		taskmanip.CloseFingers()
		robot.WaitForController(0)
		basemanip.MoveManipulator(initialvalues)
		time.sleep(1)
	D.append(scores)
	print('D')

g = open('D_MATRIX.pkl','wb')
pickle.dump(D, g)
f.close()

'''
Time for entire planning: 5.2016 seconds
Score for plan is consistently -27.631 after anything above 21 max iterations
25 waypoints
constraints were from mug3

'''