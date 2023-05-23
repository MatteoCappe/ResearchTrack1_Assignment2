#! /usr/bin/env python

"""
module: NodeC
platform: Unix
synopsis: Python module to print the robot's infos

module author: Matteo Cappellini <s4822622@studenti.unige.it>

This node prints the distance from the robot's current position and the goal's coordinates and it's average speed, the frequency at which these informations are printed is set in the launch file

Subscriber:
	/robot_info
	
Parameters:
	/des_pos_x
	/des_pos_y
	/freq
"""

import rospy
import math
import time

from assignment_2_2022.msg import InfoMsg

def callback(msg):

	"""
	Callback function that calculates the distance of the robot from the goal and the average speed based on the values retrieved from the topic /robot_info
	
	Args: 
		custom msg of type InfoMsg: contains the current coordinates and the velocity of the robot
	"""

	#get the goal position
	desX = rospy.get_param("des_pos_x")
	desY = rospy.get_param("des_pos_y")
		
	#get the robot's position and velocity
	x = msg.x
	y = msg.y
	velX = msg.velX
	velY = msg.velY
	
	#distance between the current robot's position and the desired one
	distance = math.sqrt(pow(desX - x, 2) + pow(desY - y, 2))
	
	#calculate the velocity   
	speed = math.sqrt(pow(velX, 2) + pow(velY, 2))
	
	#print infos
	print("The distance from the goal position is: ", distance, "\n")
	print("The robot's average speed is: ", speed)
	
def main():

	"""
	Main function
	
	This function initializes the subscriber and sets the rate at which the informations are printed based on the value freq retrieved from the launch file 
	"""

	global freq

	#initialize NodeC
	rospy.init_node("NodeC")
	
	#set the publishing rate
	freq = rospy.get_param("freq")
	rate = rospy.Rate(freq)
	
	#subscribe to the topic InfoMsg to get the robot's infos
	rospy.Subscriber("/robot_info", InfoMsg, callback)
	
	while not rospy.is_shutdown():
		rate.sleep()
	
	#keep the node running
	rospy.spin()
	
if __name__ == "__main__":
	main()	 
