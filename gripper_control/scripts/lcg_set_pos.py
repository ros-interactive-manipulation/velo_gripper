#!/usr/bin/env python
import argparse
import roslib
roslib.load_manifest('gripper_control')
import rospy
import pr2_controllers_msgs.msg

import os

def setLCGPosition(desired_position, force):
	print "LCGPosition"
	pub = rospy.Publisher('/l_gripper_controller/command', pr2_controllers_msgs.msg.Pr2GripperCommand, latch=True)
	rospy.init_node('lcg_setpoint')
	rospy.sleep(1.0)	
	pub.publish(position = desired_position, max_effort=force)
	print "LCG set position: %.04f, force %.03f" % (desired_position, force)

if __name__ == '__main__':

	os.environ['ROS_MASTER_URI'] = 'http://prq1:11311'

	parser = argparse.ArgumentParser(description='Set point and max force for a PR2 low cost gripper.')
	parser.add_argument('-f', type=float, help='Desired gripping force.', required=True)
	parser.add_argument('-p', type=float, help='Desired position setpoint (0.0 to 0.135 on the LCG).', required=True)

	args = parser.parse_args()

	try:
		setLCGPosition(args.p,args.f)
	except rospy.ROSInterruptException: 
		print "ROSInterruptException"
		pass
