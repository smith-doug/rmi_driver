#!/usr/bin/python2

# /*
#  * Copyright (c) 2017, Doug Smith, KEBA Corp
#  * All rights reserved.
#  *
#  * Redistribution and use in source and binary forms, with or without
#  * modification, are permitted provided that the following conditions are met:
#  *
#  * 1. Redistributions of source code must retain the above copyright notice, this
#  *    list of conditions and the following disclaimer.
#  * 2. Redistributions in binary form must reproduce the above copyright notice,
#  *    this list of conditions and the following disclaimer in the documentation
#  *    and/or other materials provided with the distribution.
#  *
#  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
#  * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
#  * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
#  * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
#  * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
#  * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
#  * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#  *
#  *
#  *  Created on: October 4, 2017
#  *      Author: Doug Smith
#  */


#This can be used to more easily test the supported commands.
#You should use 'git update-index --skip-worktree' on this file if you want to change it.

import rospy
import keba_rmi

from keba_rmi import *

   
   

if __name__ == '__main__':    

    rospy.init_node('talker', anonymous=True)  
    
    rospy.logout('Starting rmi python commander')
    
    
    rob = keba_rmi.RobotPost()
    
    
    #settings
    dFast = keba_rmi.RmiDyn([100, 100, 100, 100, 500, 1000, 1000, 10000, 1000, 10000, 10000, 100000])
    dMedium = RmiDyn([50, 50, 50, 50, 250, 1000, 1000, 10000, 1000, 10000, 10000, 100000])
    dSlow = keba_rmi.RmiDyn([10, 10, 10, 100, 50, 1000, 1000, 10000, 1000, 10000, 10000, 100000])
    
    os200 = RmiOvlSuppos(200)       
    os0 = RmiOvlSuppos(0)
   
    
    #Positions
    apHome = RmiPosJoints([0.0, -2.0999999046325684, -1.2999999523162842, -1.399999976158142, 1.5, 0.0, -0.30000001192092896])    
    qPos1 = RmiPosQuaternion([0.3, -0.6, 0.365, 0, 0, 1, 0], ['aux1:-100'])
    
    
    #Program    
    rob.ProgStart()
    
    rob.Settings(dMedium, os200)
    rob.MoveJ(apHome)    
    rob.MoveJ(qPos1)
   
    rob.Settings(overlap=os0)
    rob.MoveJ(keba_rmi.RmiPosJoints([1.0, -1.1, -1.3, -1.4, 1.5, 0, -0.3]))    
    
    
    rob.MoveL(qPos1, dMedium)
    
    rob.ProgRun()
    
    
    #Give the topic a chance to publish
    rospy.sleep(2)
    







