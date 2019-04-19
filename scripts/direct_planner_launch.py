"""Launch a planner directly by creating its interface and configuring the PlannerParameters structures.
"""
from openravepy import *
import time
import pdb
import RaveCreateModule

from numpy import pi
env = Environment() # create openrave environment
env.SetViewer('qtcoin')
env.Load('data/lab1.env.xml')
robot = env.GetRobots()[0]
prob = RaveCreateModule(env,'BaseManipulation')
# manip = robot.SetActivateManipulator('right_arm')
# pdb.set_trace()
robot.SetActiveDOFs(range(4)) # set joints the first 4 dofs
params = Planner.PlannerParameters()
pdb.set_trace()
params.SetRobotActiveJoints(robot)
params.SetGoalConfig([0,pi/2,pi/2,pi/2]) # set goal to all ones
# forces parabolic planning with 40 iterations
# params.SetExtraParameters("""<_postprocessing planner="parabolicsmoother">
    # <_nmaxiterations>40</_nmaxiterations>
# </_postprocessing>""")
params.SetExtraParameters('<time_limit>2</time_limit>')
params.SetMaxIterations(0)
planner=RaveCreatePlanner(env,'birrt')
planner.InitPlan(robot, params)

traj = RaveCreateTrajectory(env,'')
# pdb.set_trace()
planner.PlanPath(traj)
trajectory = [traj.GetWaypoint(i).tolist() for i in range(traj.GetNumWaypoints())]


for t in trajectory:
        robot.SetDOFValues(t, robot.GetActiveManipulator().GetArmIndices())
        time.sleep(0.1)
        # flag = env.CheckCollision(kin.GetLinks()[3], kin.GetLinks()[6])        # Checks for collisions between 2 cylinder arms of the robot
        # print(flag)
# for i in range(traj.GetNumWaypoints()):
#     # get the waypoint values, this holds velocites, time stamps, etc
#     data=traj.GetWaypoint(i)
#     # extract the robot joint values only
#     dofvalues = traj.GetConfigurationSpecification().ExtractJointValues(data,robot,robot.GetActiveDOFIndices())
#     raveLogInfo('waypint %d is %s'%(i,dofvalues))
time.sleep(10)