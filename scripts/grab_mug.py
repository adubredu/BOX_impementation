#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""

Description
-----------
This script launches a mug picking environment and gets a mobile manipulator robot to pick
mug1
"""
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'

import pdb
import time
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

class ConstraintPlanning:
    def __init__(self,robot,randomize=False,dests=None,switchpatterns=None,plannername=None):
    	#sets the environment
        self.envreal = robot.GetEnv() 

        #sets the robot
        self.robot = robot 			  

        #gets the manipulator of the robot
        self.manip = self.robot.GetActiveManipulator()	

        #gets an inverse kinematics model of the robot. If none is found, an IK model is auto-generated
        self.ikmodel = databases.inversekinematics.InverseKinematicsModel(robot=robot,iktype=IkParameterization.Type.Transform6D)
        if not self.ikmodel.load():
            self.ikmodel.autogenerate()

        #Dynamically generate/load the grasping set for a robot manipulator and target object
        #gets a grasp model of the target object (mug1). If none is found, a grasp model is auto-generated
        self.gmodel = databases.grasping.GraspingModel(robot=self.robot,target=self.envreal.GetKinBody('mug3'))
        if not self.gmodel.load():
            self.gmodel.autogenerate()

        #gets a base manipulator object for the robot. This is to perform path planning. Gets a task manipulator object for task planning
        self.basemanip = interfaces.BaseManipulation(self.robot,plannername=plannername)
        self.taskmanip = interfaces.TaskManipulation(self.robot,graspername=self.gmodel.grasper.plannername,plannername=plannername)



    #This function is responsible for running the entire experiment
    def graspAndMove(self,showgoalcup=True):

    	#gets the target object
        target = self.gmodel.target
        print 'grasping %s'%target.GetName()

        #Generates 10 valid grasp configurations
        #Chooses one at random
        # only use one grasp since preshape can change
        start = time.time()
        validgrasps,validindices = self.gmodel.computeValidGrasps(returnnum=1)
        validgrasp=validgrasps[random.randint(len(validgrasps))]
        elapsed = time.time() - start
        print 'TIME ELAPSED: %0.5f seconds'%elapsed
        initialvalues = self.robot.GetDOFValues(self.gmodel.manip.GetArmIndices())
        #set preshape of the grasp and get joint values
        with self.robot:
            self.gmodel.setPreshape(validgrasp)
            jointvalues = self.robot.GetDOFValues()

        #set the robot's controller to the joint values
        self.robot.GetController().SetDesired(jointvalues)

        #wait for controller to load
        self.robot.WaitForController(0)

        #get the transform of the valid grasp chosen. Store it in an array. Array would have
        #just one element. It is necessary to put transform in an array
        matrices = [self.gmodel.getGlobalGraspTransform(validgrasp,collisionfree=True)]

        #move the robot to the grasp (robot's hands moves into position to grasp the object)
        self.basemanip.MoveToHandPosition(matrices=matrices,maxiter=1000,maxtries=1,seedik=10)

        #wait til previous task is complete
        self.robot.WaitForController(0)

        #close robot's fingers
        self.taskmanip.CloseFingers()

        #wait til previous task is complete
        self.robot.WaitForController(0)

        #grab target object
        self.robot.Grab(target)

        # raw_input('press any key to release')
        # self.taskmanip.ReleaseFingers(target=target)
        self.robot.WaitForController(10)
        self.basemanip.MoveManipulator(initialvalues)
        time.sleep(10)

        
def main(env,options):
    "Main example code."
    env.Load(options.scene)
    robot = env.GetRobots()[0]
    env.UpdatePublishedBodies()
    time.sleep(0.1) # give time for environment to update
    self = ConstraintPlanning(robot)
    self.graspAndMove()

from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments

@openravepy.with_destroy
def run(args=None):


    #param args: arguments for script to parse, if not specified will use sys.argv

    parser = OptionParser(description='RRT motion planning with constraints on the robot end effector.')
    OpenRAVEGlobalArguments.addOptions(parser)
    parser.add_option('--scene',
                      action="store",type='string',dest='scene',default='data/lab1.env.xml',
                      help='Scene file to load (default=%default)')
    (options, leftargs) = parser.parse_args(args=args)
    OpenRAVEGlobalArguments.parseAndCreateThreadedUser(options,main,defaultviewer=True)

if __name__ == "__main__":
    run()