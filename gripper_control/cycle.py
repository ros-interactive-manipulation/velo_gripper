#! /usr/bin/python
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# This script cycles the position of the Matei Gripper
#
# Author: Bob Holmberg

CONTROLLER_NAME = "cycle_controller"

import sys,os,signal
import time
import random
import re
import pexpect

import roslib
roslib.load_manifest('robot_mechanism_controllers')
import rospy

from subprocess import Popen,PIPE,STDOUT
from optparse import OptionParser

from std_msgs.msg import *
from pr2_controller_manager import pr2_controller_manager_interface

# GLOBAL BECAUSE THIS NEEDS TO EXIST OR roscore IS KILLED.
p_launch = None

def rosInfra():
    global p_launch
    p_ros = Popen(['rostopic','list'],stderr=STDOUT,stdout=PIPE)
    time.sleep(2)
    (pout,perr) = p_ros.communicate()
    if re.search('/cycle_controller/command\n',pout):
       return

    # DIDN'T FIND THE RIGHT SERVICES, SO LAUNCH cycle_controller
    p_launch = pexpect.spawn('roslaunch gripper_controller cycle.launch')
    msg = 'Started controllers: cycle_controller'
    ans = 2
    while ans == 2:
        ans = p_launch.expect([msg,'\[FATAL\].*','[\r\n]+',
                               pexpect.TIMEOUT], timeout=30)
        if   ans == 0: print(p_launch.before + p_launch.after + '\n')
        elif ans == 2: print(p_launch.before)
        elif ans == 1:
            mo = re.search(' pid: ([0-9]+)',p_launch.after)
            if mo:
                os.kill( int(mo.groups()[0]), signal.SIGKILL )
            print("\nRestarting ROS...\n\n")
            rosInfra()
        else:
            buf = p_launch.before + p_launch.after
            time.sleep(4)
            try:
                buf += p_launch.read_nonblocking(timeout=0)
            except ExceptionPecpect.EOF:
                pass
            print(buf)
            p_launch.kill(9)
            time.sleep(2)
            print("\nTIMEOUT occurred, check your setup and try again.\n")
            sys.exit(0)


def main():

    # SOME BASIC CONSTANTS 
    gearSign  = -1     # -1 for no idler; +1 for idler in gear train.
    depth_min = 0.0018
    depth_max = 0.0135
    # Absolute magnitude of stopping point randomization 
    # so we don't stop on exactly the same tooth each time.
    # 0.00325/14.5 = 0.000224 = 1 motor rev (with 14.5 & 3.25mm pitch)
    epsilon_mag = 0.000224/2

    # PARSE INPUT!
    # THEY MIGHT BE ASKING FOR -h HELP, SO DON'T START ROS YET.
    parser = OptionParser()
    parser.add_option("-n", dest="num_cycles", metavar="num_cycles", 
                      type="int", default=1000,
                      help="Number of cycles" )
    parser.add_option("-a", dest="depth_min", metavar="min_depth",
                      type="float", default=depth_min,
                      help="Min depth for cycles. (default is %4.1f mm)"%(1000*depth_min) )
    parser.add_option("-d", dest="depth_max", metavar="max_depth",
                      type="float", default=depth_max,
                      help="Max depth for cycles. (default is %4.1f mm)"%(1000*depth_max) )
    parser.add_option("-t", dest="cycletime", metavar="cycletime",
                      type="float", default=6.0,
                      help="Time for one cycle" )
    (options, args) = parser.parse_args()
    # STANDARDIZE INPUT sign AND NORMALIZE TO SI UNITS
    depth_min = max( abs(options.depth_min), 2*epsilon_mag )
    if depth_min > 0.020:  depth_min /= 1000.0
    depth_max = abs(options.depth_max)
    if depth_max > 0.020:  depth_max /= 1000.0

    # CHECK FOR roscore AND cycle_controller
    rosInfra()

    # NOW SETUP ROS NODE
    joint = "gripper_joint"
    rospy.init_node('cycle', anonymous=True)
    pub = rospy.Publisher("%s/command" % CONTROLLER_NAME, Float64)

    # ECHO OPERATING PARAMETERS AFTER STARTING NODE
    print("")
    print "num_cycles = %d" % options.num_cycles
    print "min_depth  = %4.1f mm" % (1000*depth_min)
    print "max_depth  = %4.1f mm" % (1000*depth_max)
    print "cycletime  = %4.1f sec" % options.cycletime

    goal = depth_min
    for n in range(2*options.num_cycles):
        epsilon = epsilon_mag * (2*random.random()-1)
        pub.publish(Float64(gearSign*abs(goal+epsilon)))
        if goal != depth_min:
            goal = depth_min
            sys.stdout.write("\rCycle # %4d   " % (n/2+1))
            sys.stdout.flush()
        else:
            goal = depth_max
        time.sleep(options.cycletime/2.0)

        if rospy.is_shutdown():
            break

    pub.publish(Float64(gearSign*abs(depth_min)))
    print("\n")

if __name__ == '__main__':
    main()
