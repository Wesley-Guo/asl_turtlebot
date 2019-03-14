#!/usr/bin/env python

# Adapted from https://github.com/turtlebot/turtlebot/blob/kinetic/turtlebot_teleop/scripts/turtlebot_teleop_key

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
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

import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from asl_turtlebot_project_team13.msg import DetectedObject, DetectedObjectList
import numpy as np

import sys, select, termios, tty

msg = """
Keyboard tester for simulated state machine transition
---------------------------
e : move to explore mode
s : stop sign detected
d : send new delivery request
f : send simulated object list with one object
g : send simulated object list with three objects


CTRL-C to quit
"""

sim_obj_list = ['apple', 'banana', 'blue raspberry', 'elephant']
sim_obj_dists = [1, 2, 1.5, .75]
sim_obj_thetas = [np.pi/3, 0, -np.pi/3, 5*np.pi/4]

delivery_list = 'cake'

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def makeFakeDetected(id_num=0, name='fake_obj', dist=1.0, side_len=0.5, theta_offset=0):
    fake_obj = DetectedObject()
    fake_obj.id = id_num
    fake_obj.name = name
    fake_obj.confidence = 100.0
    fake_obj.distance = dist
    fake_obj.thetaleft = np.arcsin(side_len/dist) - theta_offset
    fake_obj.thetaright = np.arcsin(side_len/dist) + theta_offset
    fake_obj.corners = [-side_len,-side_len,side_len,side_len] # corners format = [ymin,xmin,ymax,xmax]
    return fake_obj

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('keyboard_tester')
    stop_pub = rospy.Publisher('/detector/stop_sign', DetectedObject, queue_size=5)
    object_pub = rospy.Publisher('/detector/objects', DetectedObjectList, queue_size=5)
    request_pub = rospy.Publisher('/delivery_request', String, queue_size=5)
    # explore_pub = rospy.Publisher()
    start_explore_pub = rospy.Publisher('/start_explore', String, queue_size=5)

    try:
        print(msg)
        obj_count = 0
        while(1):
            key = getKey()
            if key == 'e':
                start_explore_pub.publish('Explore!')
            if key == 's':
                obj_count+=1
                fake_sign = makeFakeDetected(id_num=obj_count, name='fake_sign_'+str(obj_count))
                stop_pub.publish(fake_sign)
            elif key == 'd':
                request_pub.publish(delivery_list)
                print "Request for Delivery!"
            elif key == 'f':
                obj_count+=1
                # apple_name = 'apple'+str(obj_count)
                apple_name = 'elephant'                
                fake_apple = makeFakeDetected(id_num=obj_count, name=apple_name)
                fake_list = DetectedObjectList()
                fake_list.objects.append(apple_name)
                fake_list.ob_msgs.append(fake_apple)
                object_pub.publish(fake_list)
            elif key == 'g':
                fake_list = DetectedObjectList()
                for sim_name, sim_dist, sim_offset in zip(sim_obj_list, sim_obj_dists, sim_obj_thetas):
                    obj_count+=1
                    enum_sim_name = sim_name
                    new_sim_obj = makeFakeDetected(id_num=obj_count, name=enum_sim_name, dist=sim_dist, theta_offset=sim_offset)
                    fake_list.objects.append(enum_sim_name)
                    fake_list.ob_msgs.append(new_sim_obj)
                print fake_list
                object_pub.publish(fake_list)
            else:
                if (key == '\x03'):
                    break

    except Exception as e:
        print(e)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
