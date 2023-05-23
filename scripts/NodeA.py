#! /usr/bin/env python

"""
module: NodeA
platform: Unix
synopsis: Python module to set the desired robot position via keyboard

module author: Matteo Cappellini <s4822622@studenti.unige.it>

This node implements an action client for the user to input the desired goal position for the mobile robot or to cancel the previous one
This node also publishes the current robot position and velocity via a custom message based on the values retrieved from the topic /odom

Subscriber:
	/odom
	
Publisher:
	/robot_info
	
"""

import rospy
import actionlib
import actionlib.msg
import assignment_2_2022
import assignment_2_2022.msg

from std_srvs.srv import *
from geometry_msgs.msg import Point, Pose, Twist
from nav_msgs.msg import Odometry
from assignment_2_2022.msg import InfoMsg

def callback(msg):

	"""
	Callback function to publish position and velocity of the robot taken via the custom message InfoMsg based on the information retreived from the topic /odom
	
	Args: 
		msg of type Odometry: contains the odometry informations of the robot
	"""

	global pub
	
	#get position and linear velocity from msg
	position = msg.pose.pose.position
	velocity = msg.twist.twist.linear
	
	#create custom msg
	robot_info = InfoMsg()
	robot_info.x = position.x
	robot_info.y = position.y
	robot_info.velX = velocity.x
	robot_info.velY = velocity.y
	
	#publish robot_info
	pub.publish(robot_info)
	
def client():

	"""
	Function that implements the ation client and gives to the user the possiblity to set/cancel goals
	
	When accessed, this function initializes the action client and waits for an input by the user, if the input is acceptable a new goal is set by sending the inserted coordinates to the action server that the user can then decide to cancel by sending a cancel request to the action server	
	"""

	#creates the action client
	client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2022.msg.PlanningAction)
	
	#waits for the server to be ready
	client.wait_for_server()
	
	print("Welcome to the Robot Control Interface \n")
	
	while not rospy.is_shutdown():
	
		#user interface
		print("Insert the desired position you want to reach \n")
		
		#check whether the input is a number or not
		try:
		
			x = float(input("x: "))
			y = float(input("y: "))
			
			print("\n")
			
			#check the inserted coordinates, such that the robot doesn't get stuck on a wall
			if -9.0 <= x <= 9.0 and -9.0 <= y <= 9.0:
				
				#set the goal position with the previously entered coordinates
				goal = assignment_2_2022.msg.PlanningGoal()
				goal.target_pose.pose.position.x = x
				goal.target_pose.pose.position.y = y
			    
				#send the goal to the action server
				client.send_goal(goal)
				
				print("The goal coordinates have been successfully set! \n")
				
				cancel = input("Enter 'c' to cancel the goal, or press 'enter' to set the next goal: \n")
				
				if (cancel == 'c'):
					
					#cancel goal
					client.cancel_goal()
					print("The goal was successfully cancelled! \n") 	
				
			else:
			
				print("Error!! The inserted values are out of bound, retry! \n")
		
		except ValueError: 
		
			print("Error!! The input must be a number, retry! \n")	
		
def main():
	
	"""
	Main function
	
	This function initializes the publisher and the subscriber and then calls the function client() 
	"""
	
	global pub
	
	#initialize NodeA
	rospy.init_node("NodeA")
	
	#custom msg publisher
	pub = rospy.Publisher("/robot_info", InfoMsg, queue_size=1)
	
	#subscriber to /odom, get position and speed of the robot
	sub_odom = rospy.Subscriber('/odom', Odometry, callback)
	
	#start client service
	client()

if __name__ == "__main__":
	main()	
