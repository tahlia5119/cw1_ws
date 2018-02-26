#!/usr/bin/env python
import rospy
import timeit
import numpy as np
from geometry_msgs.msg  import Twist
from geometry_msgs.msg  import Pose2D
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from math import pow,atan2,sqrt,pi
from planned_path import PlannedPath
import time
import math

# This class defines a possible base of what the robot controller
# could do.

class ControllerBase(object):

    def __init__(self, occupancyGrid):

        rospy.wait_for_message('/robot0/odom', Odometry)

        # Create the node, publishers and subscriber
        self.velocityPublisher = rospy.Publisher('/robot0/cmd_vel', Twist, queue_size=10)
        self.currentOdometrySubscriber = rospy.Subscriber('/robot0/odom', Odometry, self.odometryCallback)

        # Specification of accuracy. The first is the Euclidean
        # distance from the target within which the robot is assumed
        # to be there. The second is the angle. The latter is turned
        # into radians for ease of the controller.
        self.distanceErrorTolerance = rospy.get_param('distance_error_tolerance', 0.05)
        self.goalAngleErrorTolerance = math.radians(rospy.get_param('goal_angle_error_tolerance', 0.1))

        # Set the pose to an initial value to stop things crashing
        self.pose = Pose2D()

        # Store the occupany grid - used to transform from cell
        # coordinates to world driving coordinates.
        self.occupancyGrid = occupancyGrid
        
        # This is the rate at which we broadcast updates to the simulator in Hz.
        self.rate = rospy.Rate(10)

    # Get the pose of the robot. Store this in a Pose2D structure because
    # this is easy to use. Use radians for angles because these are used
    # inside the control system.
    def odometryCallback(self, odometry):
        odometryPose = odometry.pose.pose

        pose = Pose2D()

        position = odometryPose.position
        orientation = odometryPose.orientation
        
        pose.x = position.x
        pose.y = position.y
        pose.theta = 2 * atan2(orientation.z, orientation.w)
        self.pose = pose

    # Return the most up-to-date pose of the robot
    def getCurrentPose(self):
        return self.pose

    # Handle the logic of driving the robot to the next waypoint
    def driveToWaypoint(self, waypoint):
        raise NotImplementedError()

    # Handle the logic of rotating the robot to its final orientation
    def rotateToGoalOrientation(self, waypoint):
        raise NotImplementedError()


    def shortestAngularDistance(self, fromAngle, toAngle):
        delta = toAngle - fromAngle
        if (delta < -math.pi):
            delta = delta + 2.0*math.pi
        elif(delta > math.pi):
            delta = delta - 2.0*math.pi
        return delta
            
    # Drive to each waypoint in turn. Unfortunately we have to add
    # the planner drawer because we have to keep updating it to
    # make sure the graphics are redrawn properly.
    def drivePathToGoal(self, path, goal, plannerDrawer):
        #Define variables to store distance, time, and angle
        distanceTravelled = 0
        angleTurned = 0
        prevTheta = 0
	
        self.plannerDrawer = plannerDrawer

 	"""
	The following is added code to reduce the number of waypoints
	"""
	cellPath = []
	direction = 1
	prevDir = 1

	for num in range(0,len(path.waypoints)-1):
	    prevDir = direction
	    cell = path.waypoints[num].coords
	    cell1 = path.waypoints[num+1].coords
	    
	    #Horizontal direction
	    if cell[0] == cell1[0]:
		direction = 1
	    #Vertical direction
	    elif cell[1] == cell1[1]:
		direction = 2
	    else:
	    #Diagonal direction
		direction = 3

	    if direction != prevDir:
		cellPath.append(path.waypoints[num])
	
	    #Adding the final waypoint
	    if num == len(path.waypoints)-2:
		cellPath.append(path.waypoints[len(path.waypoints)-1])

	"""
	End of added code
	"""
        rospy.loginfo('Driving path to goal with ' + str(len(cellPath)) + ' waypoint(s)')
        rospy.loginfo('Orignal number of waypoints: '+str(len(path.waypoints)))      
        start = time.time()

        # Drive to each waypoint in turn
        for waypointNumber in range(0, len(cellPath)):
	    
            cell = cellPath[waypointNumber]
            waypoint = self.occupancyGrid.getWorldCoordinatesFromCellCoordinates(cell.coords)
            rospy.loginfo("Driving to waypoint (%f, %f)", waypoint[0], waypoint[1])
		
	    #Calculating distance travelled
            dX = waypoint[0] - self.pose.x
            dY = waypoint[1] - self.pose.y
            distanceTravelled += sqrt(dX*dX+dY*dY)

	    #Calculating angle turned
            prevTheta = self.pose.theta 

            self.driveToWaypoint(waypoint)
	    
            angleTurned += abs(self.shortestAngularDistance(prevTheta,self.pose.theta))

            # Handle ^C
            if rospy.is_shutdown() is True:
                break

	prevTheta = self.pose.theta

        # Finish off by rotating the robot to the final configuration
	self.rotateToGoalOrientation(goal)
	
	angleTurned += abs(self.shortestAngularDistance(prevTheta,self.pose.theta))

        end = time.time()

        rospy.loginfo('Rotating to goal orientation')

        print("Distance Travelled: {}".format(distanceTravelled))
        print ("Total Angle Turned: {}".format(angleTurned*180/math.pi))
        print ("Travel time: {}".format(end-start))
        raw_input('Press enter to continue')

	
        

        
 
