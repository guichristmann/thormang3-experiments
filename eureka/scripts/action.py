#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from robotis_controller_msgs.msg import JointCtrlModule
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32,String

from utils import engine

class talker:
    def __init__(self):
        self.JointName = []
        
        
        name2num = { 
                          'gripON_L'  : "gripON_L",
                          'gripOFF_L' : "gripOFF_L",
                          'gripON_R'  : "gripON_R",
                          'gripOFF_R' : "gripOFF_R",
                          'fLL'  : 34,
                          'bLL'  : 35,
                          'fRR'  : 36,
                          'bRR'  : 37,
                          'fML'  : 38,
                          'bML'  : 39,
                          'fMR'  : 40,
                          'bMR' : 41,
                          'PourStart' : 30,
                          'PourEnd'   : 31,
                          'PouringRtoL' : 32,
                          'PouringLtoR' : 33,
        }
        
        self.engine = engine.Engine(name2num)

    def getJointName(self,data):
        print("read joint name")
        self.JointName = data.name
        self.subJointName.unregister()
    def run(self):
        
        rospy.init_node('pyAction', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        sleep = rospy.Rate(200) # 10hz
          

        while not rospy.is_shutdown():
            page = raw_input('Enter page: ')
            if page == "pickLL":
                plan = ["gripOFF_L","fLL","gripON_L","bLL"]
                print("Start")
                self.engine.setPlan(plan)
                while self.engine.isRunning:
                    self.engine.run()

            if page == "placeLL":
                plan = ["fLL","gripOFF_L","bLL","gripON_L"]
                print("Start")
                self.engine.setPlan(plan)
                while self.engine.isRunning:
                    self.engine.run()

            if page == "pickRR":
                plan = ["gripOFF_R","fRR","gripON_R","bRR"]
                print("Start")
                self.engine.setPlan(plan)
                while self.engine.isRunning:
                    self.engine.run()

            if page == "placeRR":
                plan = ["fRR","gripOFF_R","bRR","gripON_R"]
                print("Start")
                self.engine.setPlan(plan)
                while self.engine.isRunning:
                    self.engine.run()

            if page == "pickML":
                plan = ["gripOFF_L","fML","gripON_L","bML"]
                print("Start")
                self.engine.setPlan(plan)
                while self.engine.isRunning:
                    self.engine.run()

            if page == "placeML":
                plan = ["fML","gripOFF_L","bML","gripON_L"]
                print("Start")
                self.engine.setPlan(plan)
                while self.engine.isRunning:
                    self.engine.run()

            if page == "pickMR":
                plan = ["gripOFF_R","fMR","gripON_R","bMR"]
                print("Start")
                self.engine.setPlan(plan)
                while self.engine.isRunning:
                    self.engine.run()

            if page == "placeMR":
                plan = ["fMR","gripOFF_R","bMR","gripON_R"]
                print("Start")
                self.engine.setPlan(plan)
                while self.engine.isRunning:
                    self.engine.run()

            if page == "init":
                plan = ["PourEnd"]
                print("Start")
                self.engine.setPlan(plan)
                while self.engine.isRunning:
                    self.engine.run()

            if page == "pourRL":
                plan = ["PourStart","PouringRtoL","PourEnd"]
                print("Start")
                self.engine.setPlan(plan)
                while self.engine.isRunning:
                    self.engine.run()

            if page == "pourLR":
                plan = ["PourStart","PouringLtoR","PourEnd"]
                print("Start")
                self.engine.setPlan(plan)
                while self.engine.isRunning:
                    self.engine.run()



         


            #pub_action.publish(Int32(page))
            rate.sleep()
        

if __name__ == '__main__':
    try:
        t = talker()
        t.run()
    except rospy.ROSInterruptException:
        pass
