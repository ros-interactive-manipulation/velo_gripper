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
import roslib
roslib.load_manifest('robot_mechanism_controllers')
import rospy

from std_msgs.msg import *
from pr2_controller_manager import pr2_controller_manager_interface

prev_handler = None

def main():
    joint = "gripper_joint"

    rospy.init_node('cycle', anonymous=True)

    pub = rospy.Publisher("%s/command" % CONTROLLER_NAME, Float64)

    goal = 0.0

    # TAKE NUMBER OF CYCLES AS INPUT ARG
    try:      num_cycles = int( sys.argv[1] )
    except:   num_cycles = 1000
    print "num_cycles = %d" % num_cycles
    depth = 0.014
    try:      depth = min(depth,eval( sys.argv[2] ))
    except:   depth = depth
    print "depth = %5.3f m" % depth
    try:      cycletime = float( sys.argv[3] )
    except:   cycletime = 6.0
    print "cycletime = %3.1f sec" % cycletime

    for n in range(2*num_cycles):
        pub.publish(Float64(goal))
        if goal > .003:
            goal = 0.002
            print "Cycle # %4d" % (n/2+1)
        else:
            goal = depth
            print "%5.3f" % goal
        time.sleep(cycletime/2.0)

        if rospy.is_shutdown():
            break

    pub.publish(Float64(0.0))

if __name__ == '__main__':
    main()
