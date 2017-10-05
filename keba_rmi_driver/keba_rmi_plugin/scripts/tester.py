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


# This can be used to more easily test the supported commands.
# You should use 'git update-index --skip-worktree' on this file if you want to change it.

#import rospy
import keba_rmi

from keba_rmi import *

#===============================================================================
# Global kairo variables (_globalvars.tid)
#===============================================================================

# settings
dFast = DYNAMIC([100, 100, 100, 100, 500, 1000, 1000, 10000, 1000, 10000, 10000, 100000])
dMedium = DYNAMIC([50, 50, 50, 50, 250, 1000, 1000, 10000, 1000, 10000, 10000, 100000])
dSlow = DYNAMIC([10, 10, 10, 100, 50, 1000, 1000, 10000, 1000, 10000, 10000, 100000])

os200 = OVLSUPPOS(200)
os0 = OVLSUPPOS(0)
or200 = OVLREL(200)
oa10 = OVLABS([10, 360, 10, 10, 0])

# Positions
apHome = RmiPosJoints([0.0, -2.0999999046325684, -1.2999999523162842, -1.399999976158142, 1.5, 0.0, -0.30000001192092896])
qPos1 = RmiPosQuaternion([0.3, -0.6, 0.365, 0, 0, 1, 0], ['aux1:-100'])
qPos2 = RmiPosQuaternion([0.5, -0.6, 0.365, 0, 0, 1, 0], ['aux1:-100'])

# The robot
rob = keba_rmi.default_rob


#===============================================================================
# Programs
#===============================================================================


def move_square_generic():
    '''
    Move the robot in a square.  You can change ovlToUse/dynToUse to test.  Uses the more generic function names. 
    '''

    ovlToUse = oa10
    dynToUse = dMedium

    qSquare1 = RmiPosQuaternion([0.3, -0.6, 0.365, 0, 0, 1, 0], ['aux1:-300'])
    qSquare2 = RmiPosQuaternion([0.6, -0.6, 0.365, 0, 0, 1, 0], ['aux1:-300'])
    qSquare3 = RmiPosQuaternion([0.6, -0.3, 0.365, 0, 0, 1, 0], ['aux1:-300'])
    qSquare4 = RmiPosQuaternion([0.3, -0.3, 0.365, 0, 0, 1, 0], ['aux1:-300'])

    rob.ProgStart()

    rob.Settings(dMedium, os200)    # Dyn(dMedium) and Ovl(os200)
    rob.MoveJ(apHome)  # PTP(apHome)

    rob.MoveJ(qSquare1, dynToUse, ovlToUse)  # PTP(qSquare1, dynToUse, ovlToUse)
    rob.MoveL(qSquare2, dynToUse, ovlToUse)  # Lin(qSquare2, dynToUse, ovlToUse)
    rob.MoveL(qSquare3, dynToUse, ovlToUse)  # Lin(qSquare3, dynToUse, ovlToUse)
    rob.MoveL(qSquare4, dynToUse, ovlToUse)  # Lin(qSquare4, dynToUse, ovlToUse)
    rob.MoveL(qSquare1, dynToUse, ovlToUse)  # Lin(qSquare1, dynToUse, ovlToUse)

    rob.ProgRun()


def move_square():
    '''
    Move the robot in a square.  You can change ovlToUse/dynToUse to test.
    '''

    ovlToUse = oa10
    dynToUse = dMedium

    qSquare1 = QUATPOS([0.3, -0.6, 0.365, 0, 0, 1, 0], ['aux1:-300'])
    qSquare2 = QUATPOS([0.6, -0.6, 0.365, 0, 0, 1, 0], ['aux1:-300'])
    qSquare3 = QUATPOS([0.6, -0.3, 0.365, 0, 0, 1, 0], ['aux1:-300'])
    qSquare4 = QUATPOS([0.3, -0.3, 0.365, 0, 0, 1, 0], ['aux1:-300'])

    rob.ProgStart()

    Dyn(dMedium)
    Ovl(os200)

    PTP(apHome)  # PTP(apHome)

    PTP(qSquare1, dynToUse, ovlToUse)  # PTP(qSquare1, dynToUse, ovlToUse)
    Lin(qSquare2, dynToUse, ovlToUse)  # Lin(qSquare2, dynToUse, ovlToUse)
    Lin(qSquare3, dynToUse, ovlToUse)  # Lin(qSquare3, dynToUse, ovlToUse)
    Lin(qSquare4, dynToUse, ovlToUse)  # Lin(qSquare4, dynToUse, ovlToUse)
    WaitIsFinished()
    Lin(qSquare1, dynToUse, ovlToUse)  # Lin(qSquare1, dynToUse, ovlToUse)

    rob.ProgRun()


if __name__ == '__main__':

    rospy.init_node('talker', anonymous=True)

    rospy.logout('Starting rmi python commander')

    move_square()
