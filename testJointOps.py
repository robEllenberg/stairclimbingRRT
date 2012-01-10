# -*- coding: utf-8 -*-
'''Test program to show reaching poses for Jaemi Hubo Right arm'''

from openravepy import *
from numpy import *
import time
import sys


if __name__ == "__main__":
      
    #load the environment if it is not already loaded
    try:
        orEnv
    except NameError:
        orEnv = Environment()
        orEnv.SetViewer('qtcoin')
    
    orEnv.Reset()
    orEnv.Load('openHubo/jaemiHubo.fullDOF.robot.xml')
    
    robot = orEnv.GetRobots()[0]   
    kinbody =  orEnv.GetKinBody('jaemiHubo')
    print 'Output of command GetDependenyOrderedJoints'
    print kinbody.GetDependencyOrderedJoints()
    print 'Finding information about joints using GetJoint'
    LHY = kinbody.GetJoint('LHY')
    print LHY.GetDOF()
    print LHY.GetDOFIndex()

    
    #set printing, display options, and collision checker
    orEnv.SetDebugLevel(DebugLevel.Info)
    colchecker = RaveCreateCollisionChecker(orEnv,'ode')
    orEnv.SetCollisionChecker(colchecker)

    print "Press return to exit."
    sys.stdin.readline()
