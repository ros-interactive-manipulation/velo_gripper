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

import sys
import time
import random

import roslib
roslib.load_manifest('robot_mechanism_controllers')
import rospy

from optparse import OptionParser

from std_msgs.msg import *
from pr2_controller_manager import pr2_controller_manager_interface

prev_handler = None

def main():
    joint = "gripper_joint"

    rospy.init_node('cycle', anonymous=True)

    pub = rospy.Publisher("%s/command" % CONTROLLER_NAME, Float64)

    gearSign  = -1     # -1 for no idler; +1 for idler in gear train.
    depth_min = 0.0018
    depth_max = 0.0135
    # Absolute magnitude of stopping point randomization 
    # so we don't stop on exactly the same tooth each time.
    # 0.00325/14.5 = 0.000224 = 1 motor rev (with 14.5 & 3.25mm pitch)
    epsilon_mag = 0.000224/2

    # TAKE NUMBER OF CYCLES AS INPUT ARG
    
    parser = OptionParser()
    parser.add_option("-n","--num_cycles", dest="num_cycles", metavar="num_cycles", 
                      type="int", default=1000,
                      help="Number of cycles" )
    parser.add_option("-d","--depth", dest="depth", metavar="depth",
                      type="float", default=depth_max,
                      help="Max depth for cycles. (default is %5.3f m)"%depth_max )
    parser.add_option("-t","--cycletime", dest="cycletime", metavar="cycletime",
                      type="float", default=6.0,
                      help="Time for one cycle" )
    (options, args) = parser.parse_args()

    depth_max = abs(options.depth)
    if depth_max > 1.0:  depth_max /= 1000.0  # Force to mm

    print "num_cycles = %d" % options.num_cycles
    print "depth      = %5.3f m" % options.depth
    print "cycletime  = %3.1f sec" % options.cycletime

    goal = depth_min
    for n in range(2*options.num_cycles):
        epsilon = epsilon_mag * (2*random.random()-1)
        pub.publish(Float64(gearSign*abs(goal+epsilon)))
        if goal != depth_min:
            goal = depth_min
            print "Cycle # %4d" % (n/2+1)
        else:
            goal = depth_max
        time.sleep(options.cycletime/2.0)

        if rospy.is_shutdown():
            break

    pub.publish(Float64(gearSign*abs(depth_min)))

if __name__ == '__main__':
    main()
