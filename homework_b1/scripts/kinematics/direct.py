#!/usr/bin/env python3
from __future__ import print_function

import roslib
import rospy
import PyKDL as kdl
import kdl_parser_py.urdf as kdl_parser
from geometry_msgs.msg import Quaternion, Pose

from homework_b1.srv import directKinematics

def handleDirectKinematics(req):
    print("\n---------------------------------------")
    print("-> Echo the user provided joint angles: %f, %f, %f, %f, %f"%(req.joint_1, req.joint_2, req.joint_3, req.joint_4, req.left_gripper_joint))

    (status, tree) = kdl_parser.treeFromFile("/home/fabio/catkin_ws/src/homework_b1/urdf/scara.urdf")
    print("-> Successfully parsed urdf file and constructed kdl tree!" if status else "Failed to parse urdf file to kdl tree")
    print("---------------------------------------")
    
    chain = tree.getChain("link1", "left_tip") #get urdf links chain

    dirKin = kdl.ChainFkSolverPos_recursive(chain)
    jointAngles = kdl.JntArray(5)
    jointAngles[0] = req.joint_1
    jointAngles[1] = req.joint_2
    jointAngles[2] = req.joint_3
    jointAngles[3] = req.joint_4
    jointAngles[4] = req.left_gripper_joint #only the left is kept due to chain computation
    
    foundError = checkJointAngles(jointAngles) #check if the provided angles are acceptable (within limits)
    
    dirKin_pose = Pose() #Pose object (position + orientation)
    finalFrame = kdl.Frame()
    
    if not foundError:	
        dirKin.JntToCart(jointAngles, finalFrame) #from joint to cartesian space
        
        #position:
        dirKin_pose.position.x = finalFrame.p.x()
        dirKin_pose.position.y = finalFrame.p.y()
        dirKin_pose.position.z = finalFrame.p.z()
        x,y,z,w = finalFrame.M.GetQuaternion()
	
        #orientation:
        dirKin_pose.orientation.x = x
        dirKin_pose.orientation.y = y
        dirKin_pose.orientation.z = z
        dirKin_pose.orientation.w = w
    else:
        #default position:
        dirKin_pose.position.x = 0
        dirKin_pose.position.y = 0
        dirKin_pose.position.z = 0
        x,y,z,w = finalFrame.M.GetQuaternion()
	    
        #default orientation:
        dirKin_pose.orientation.x = x
        dirKin_pose.orientation.y = y
        dirKin_pose.orientation.z = z
        dirKin_pose.orientation.w = w

    return dirKin_pose


def checkJointAngles(jointAngles):
    #account for joint's angles limits
    jointLimits = [[-2.5, 2.5], [-2, 2], [0, 0.45], [-3, 3], [0, 0.55]]
    foundError = False
    for i, providedAngle in enumerate(jointAngles):
    	if providedAngle < jointLimits[i][0] or providedAngle > jointLimits[i][1]:
            print("\n---------------------------------------\n" if not foundError else "", end="")
            print("* Error: provided angle for joint", i+1, "("+str(providedAngle)+") is outside the", jointLimits[i], "limit!")
            foundError = True
    
    print("---------------------------------------\n" if foundError else "", end="")
       
    return foundError
        


def directKinematics_server():
    rospy.init_node('directKinematics_server')  #initialize the directKinematics_server node
    s = rospy.Service('directKinematics', directKinematics, handleDirectKinematics)
    print("Ready to compute direct kinematics of the robot.\n")
    rospy.spin()  #wait until the service is called

if __name__ == "__main__":
    directKinematics_server()
