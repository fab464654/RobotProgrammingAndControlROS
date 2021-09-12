#!/usr/bin/env python3

from __future__ import print_function
import rospy
import roslib
from math import pi
import PyKDL as kdl
import kdl_parser_py.urdf as kdl_parser
from geometry_msgs.msg import Quaternion, Pose

from homework_b1.srv import inverseKinematics

def handleInverseKinematics(req):
    print("\n---------------------------------------")
    print("-> Echo the user provided target pose: x=%f, y=%f, z=%f, phi=%f"%(req.ee_x, req.ee_y, req.ee_z, req.ee_phi))

    (status, tree) = kdl_parser.treeFromFile("/home/fabio/catkin_ws/src/homework_b1/urdf/scara.urdf")
    print("-> Successfully parsed urdf file and constructed kdl tree!" if status else "Failed to parse urdf file to kdl tree")
    print("---------------------------------------")
    
    chain = tree.getChain("link1", "left_tip")    #get links chain from urdf
    invKin_vel = kdl.ChainIkSolverVel_pinv(chain) #inverse kinematics velocity
    dirKin = kdl.ChainFkSolverPos_recursive(chain) 
    
    invKin = kdl.ChainIkSolverPos_NR(chain, dirKin, invKin_vel)
    
    foundError = checkTargetPosition(req.ee_x, req.ee_y, req.ee_z) #check if the provided target coordinates are acceptable 
    
    if not foundError:	
        position = kdl.Vector(req.ee_x, req.ee_y, req.ee_z); #position vector
        rotation = kdl.Rotation()		                     #rotation angle
        rotation.RotZ(req.ee_phi)
    
        targetPose = kdl.Frame(rotation, position)
    
        jointAngles = kdl.JntArray(5)
        jointAngles[0] = 0
        jointAngles[1] = 0
        jointAngles[2] = 0
        jointAngles[3] = 0
        jointAngles[4] = 0
    
        thetaAngles = kdl.JntArray(5) #joint angles needed to reach the target pose (only 5)
    
        invKin.CartToJnt(jointAngles, targetPose, thetaAngles)  #from cartesian to joint coordinates
    else:
        thetaAngles = [0, 0, 0, 0, 0] #default position
    
    
    return normalizeAngles(thetaAngles)


def normalizeAngles(thetaAngles):  #needed to get angles in the right angular range

    jointLimits = [[-2.5, 2.5], [-2, 2], [0.001, 0.45], [-3, 3], [0.001, 0.55]] #0.001 is to avoid zero division exception
    for i, angle in enumerate(thetaAngles):

        if angle < 0 and angle < jointLimits[i][0]:
            thetaAngles[i] = angle % jointLimits[i][0]
        elif angle > 0 and angle > jointLimits[i][1]:
    	    thetaAngles[i] = angle % jointLimits[i][1]
    	    
    return thetaAngles[0], thetaAngles[1], thetaAngles[2], thetaAngles[3], thetaAngles[4] 



def checkTargetPosition(x, y, z):
    
    #Approximated (manually computed and measured in RViz) workspace limits
    workspaceLimits = [[0.05, 0.4], [0.32, 0.6], [0.32, 0.6]]
    
    foundError = False
    providedCoords = [x, y, z]
    coordsNames = ["x", "y", "z"]
    for i, coord in enumerate(providedCoords):
    	if coord < workspaceLimits[i][0] or coord > workspaceLimits[i][1]:
            print("\n---------------------------------------\n" if not foundError else "", end="")
            print("* Error: provided coordinate", coordsNames[i], "("+str(coord)+") is outside the", workspaceLimits[i], "limit!")
            foundError = True
    
    print("---------------------------------------\n" if foundError else "", end="")
       
    return foundError




def inverseKinematics_server():
    rospy.init_node('inverseKinematics_server')  #initialize the inverseKinematics_server node
    s = rospy.Service('inverseKinematics', inverseKinematics, handleInverseKinematics)
    print("Ready to calculate inverse kinematics.")
    rospy.spin()  #wait untile the service is called


if __name__ == "__main__":
    inverseKinematics_server()
