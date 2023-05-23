#! /usr/bin/env python

"""
module: NodeB
platform: Unix
synopsis: Python module to get the number of goals reached and cancelled

module author: Matteo Cappellini <s4822622@studenti.unige.it>

This node prints the two counters related to the number of goals reached and cancelled by the robot

Subscriber:
	/reaching_goal/result
"""

import rospy
import actionlib
import actionlib.msg
import assignment_2_2022
import assignment_2_2022.msg

#initialize global variables to keep track of the number of goals cancelled and reached
cancelled = 0
reached = 0

def callback(msg):

	"""
	Callback function to update the counters everytime a new status is reached and to print them on screen
	
	Args: 
		msg of type assignment_2_2022.msg.PlanningActionResult: contains the results related to the action
	"""

	global cancelled, reached
	
	#check the status and update the two counters based on the result
	if msg.status.status == 2:
		cancelled += 1
		
	elif msg.status.status == 3:
		reached += 1
		
	#print the counters relative to the goals
	print("Cancelled Counter: ", cancelled)
	print("Reached Counter: ", reached, "\n")
	
def main():

	"""
	Main function
	
	This function initializes the subscriber 
	"""

	#initialize NodeB
	rospy.init_node("NodeB")
	
	#subscriber to to get the goals status
	sub = rospy.Subscriber('/reaching_goal/result', assignment_2_2022.msg.PlanningActionResult, callback)
	
	#keep the node running
	rospy.spin()
	
if __name__ == "__main__":
	main()	
