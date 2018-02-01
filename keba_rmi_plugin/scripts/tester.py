#!/usr/bin/python2
# PYTHON_ARGCOMPLETE_OK

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
import argparse
import keba_rmi
import copy
import threading


from keba_rmi import *
from genpy import dynamic

argcomplete_available = False
try:
    import argcomplete
    argcomplete_available = True
except ImportError:
    pass


#===============================================================================
# Global kairo variables (_globalvars.tid)
#===============================================================================

# settings
dFast = DYNAMIC([100, 100, 100, 100, 1000, 5000, 5000, 100000, 1000, 10000, 10000, 100000])
dMedium = DYNAMIC([50, 50, 50, 50, 250, 1000, 1000, 10000, 1000, 10000, 10000, 100000])
dSlow = DYNAMIC([10, 10, 10, 100, 50, 1000, 1000, 10000, 1000, 10000, 10000, 100000])

os200 = OVLSUPPOS(200)
os0 = OVLSUPPOS(0)
or200 = OVLREL(200)
oa10 = OVLABS([30, 360, 40, 4, 0])

# Positions
apHome = RmiPosJoints([0.0000, -2.100, -1.300, -1.400, 1.5000, 0.0000, -0.300])
apHomeRob2 = RmiPosJoints([0.0000, -2.100, -1.300, -1.400, 1.5000, 0.0000])
qPos1 = RmiPosQuaternion([0.3, -0.6, 0.365, 0, 0, 1, 0], ['aux1:-100'])
qPos2 = RmiPosQuaternion([0.5, -0.6, 0.365, 0, 0, 1, 0], ['aux1:-100'])

# The robots
rob = keba_rmi.default_rob
rob2 = keba_rmi.RobotPost('rob2/command_list', 'rob2/command_result')


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
    qSquare2 = QUATPOS([0.6, -0.6, 0.365, 0, 0, 1, 0], ['aux1:-200'])
    qSquare3 = QUATPOS([0.6, -0.3, 0.365, 0, 0, 1, 0], ['aux1:-100'])
    qSquare4 = QUATPOS([0.3, -0.3, 0.365, 0, 0, 1, 0], ['aux1:-300'])

    rob.ProgStart()

    # Begin KAIRO commands.  These will be changed into
    # robot_movement_interface Commands and published to the /command_list topic

    Dyn(dFast)
    Ovl(oa10)

    PTP(apHome)

    PTP(qSquare1, dynToUse, ovlToUse)
    Lin(qSquare2, dynToUse, ovlToUse)
    Lin(qSquare3, dynToUse, ovlToUse)
    Lin(qSquare4, dynToUse, ovlToUse)
    WaitIsFinished()
    Lin(qSquare1, dynToUse, ovlToUse)

    rob.ProgRun()


def do_something():

    rob2.ProgStart()

    apTemp = RmiPosJoints([1.0000, -1.100, -1.300, -1.400, 1.5000, 1.0000])

    rob2.Settings(dFast, oa10)
    rob2.MoveJ(apHomeRob2)
    rob2.MoveJ(apTemp)
    rob2.MoveJ(apHomeRob2)
    rob2.ProgRun()

    rob.ProgStart()
    Dyn(dFast)
    Ovl(oa10)

    apHomeTemp = RmiPosJoints([2.9502, -0.9177, -1.347, -2.5049, 0.7632, 3.512, -0.3])
    PTP(apHome)
    PTP(apHomeTemp)
    rob.ProgRun()


def do_settings_rob1():
    rob.ProgStart()
    Dyn(dFast)
    Ovl(oa10)
    rob.ProgRun()


def do_settings_rob2():
    rob2.ProgStart()
    rob2.Settings(dFast, os200)
    rob2.ProgRun()


def do_settings_both():
    t1 = threading.Thread(target=do_settings_rob1)
    t2 = threading.Thread(target=do_settings_rob2)
    t1.start()
    t2.start()
    t1.join()
    t2.join()


def home_rob1():
    rob.ProgStart()
    PTP(apHome, dFast)
    rob.ProgRun()


def home_rob2():
    print 'asfsa'
    rob2.ProgStart()
    rob2.MoveJ(apHomeRob2, dynamic=dFast)
    rob2.ProgRun()


def home_comau():
    apHomeComau = RmiPosJoints([0.0, 0.1487, -1.5906, -0.0, 1.4023, 0.0])
    rob.ProgStart()
    rob.MoveJ(apHomeComau, dynamic=dFast)
    rob.ProgRun()


def do_settings_comau():
    rob.ProgStart()
    Dyn(dFast)
    Ovl(oa10)
    t0 = RmiFrame([0.0, 0.0, 0.030, 0, 0, 0])
    rob.Tool(t0)
    rob.ProgRun()


def home_both():
    t1 = threading.Thread(target=home_rob1)
    t2 = threading.Thread(target=home_rob2)
    t1.start()
    t2.start()
    t1.join()
    t2.join()


def cause_error():

    apBad = RmiPosJoints([0.0000, -2.100, -1.300, -1.400, 1.5000, 0.0000, -0.300])
    # apBad.pose.append(42)  # Make the pose too long (8 axes)
    apBad.pose[0] = 100

    rob.ProgStart()
    PTP(apHome, dFast)
    PTP(apBad)
    PTP(apHome, dFast)
    rob.WaitIsFinished()
    rob.ProgRun()


def rob2_do_stuff():
    rob2.ProgStart()

    ap0 = RmiPosJoints([1.0000, -1.100, -1.300, -1.400, 1.5000, 0.0000])
    ap1 = RmiPosJoints([1.0000, -2.100, -1.300, -1.400, 1.5000, 0.0000])

    qp0 = RmiPosQuaternion([0.4515, 0.1684, 0.4100, 0.0000, 0.0000, 1.0000, 0.0000])
    qp1 = RmiPosQuaternion([0.2920, -0.000, 0.4100, 0.0000, 0.0000, 1.0000, 0.0000])

    rob2.MoveJ(RmiPosJoints([1.0000, -1.100, -1.300, -1.400, 1.5000, 0.0000]))
    rob2.MoveJ(apHomeRob2)
    rob2.ProgRun()


def tool_rob1():
    rob.ProgStart()
    t0 = RmiFrame([0.0, 0.0, 0.030, 0, 0, 0])
    rob.Tool(t0)
    rob.ProgRun()


def abort_rob1():
    rob.Abort()


def abort_rob2():
    rob2.Abort()


def abort_both():
    t1 = threading.Thread(target=abort_rob1)
    t2 = threading.Thread(target=abort_rob2)
    t1.start()
    t2.start()
    t1.join()
    t2.join()


def stress_rob1():
    for i in range(0, 100):
        print 'Stress loop # ' + str(i)
        move_square()

        if rospy.is_shutdown():
            return


# function_map = {
#     'move_square': move_square,
#     'do_settings_rob1': do_settings_rob1,
#     'do_settings_rob2': do_settings_rob2,
#     'do_settings_both': do_settings_both,
#     'do_something': do_something,
#     'home_rob1': home_rob1,
#     'home_rob2': home_rob2,
#     'home_both': home_both,
#     'cause_error': cause_error,
#     'rob2_do_stuff': rob2_do_stuff,
#     'abort_rob1': abort_rob1,
#     'stress_rob1': stress_rob1,
#     'tool_rob1': tool_rob1,
#     'home_comau': home_comau,
#     'do_settings_comau': do_settings_comau,
# }

function_map_comau = {
    'do_settings': do_settings_comau,
    'home': home_comau,
    'tool': tool_rob1,
}

function_map_rob1 = {
    'do_settings_rob1': do_settings_rob1,
    'home_rob1': home_rob1,
    'abort_rob1': abort_rob1,
}

parser = argparse.ArgumentParser(description='Python tester.')

# func_names = list(function_map.keys())
# func_names.sort()


class SubCmdBase(object):
    '''
    SubCmdBase borrowed from industrial_experimental   
    '''

    def __init__(self):
        self.function_map = {}
        self.name = 'not_set'

    def add_to_subparser(self, subparser):
        parser_support = subparser.add_parser(self.name)
        parser_support.add_argument('command', nargs='?', choices=self.function_map)
        parser_support.set_defaults(func=self._execute)

    def _execute(self, args):
        print(self.name + ' ' + args.command)

        if args.command in self.function_map.keys():
            func = self.function_map[args.command]
            func()
        else:
            print 'Error function not found'


class ComauSubCmd(SubCmdBase):
    def __init__(self):
        self.name = 'comau'
        self.function_map = {
            'do_settings': do_settings_comau,
            'home': home_comau,
            'abort': abort_rob1,
            'tool': tool_rob1,
        }


class Rob1SubCmd(SubCmdBase):
    def __init__(self):
        self.name = 'rob1'
        self.function_map = {
            'move_square': move_square,
            'do_settings_rob1': do_settings_rob1,
            'home': home_rob1,
            'cause_error': cause_error,
            'abort': abort_rob1,
            'stress': stress_rob1,
            'tool': tool_rob1,
        }


class Rob2SubCmd(SubCmdBase):
    def __init__(self):
        self.name = 'rob2'
        self.function_map = {
            'do_settings': do_settings_rob2,
            'home': home_rob2,
            'abort': abort_rob2,
        }


class BothUR5sSubCmd(SubCmdBase):
    def __init__(self):
        self.name = 'both'
        self.function_map = {
            'do_settings': do_settings_both,
            'do_something': do_something,
            'home': home_both,
        }


sub_robots = parser.add_subparsers(title='Robots')

ComauSubCmd().add_to_subparser(sub_robots)
Rob1SubCmd().add_to_subparser(sub_robots)
Rob2SubCmd().add_to_subparser(sub_robots)
BothUR5sSubCmd().add_to_subparser(sub_robots)


if argcomplete_available:
    argcomplete.autocomplete(parser)

if __name__ == '__main__':

    rospy.init_node('talker', anonymous=True)
    rospy.logout('Starting rmi python commander')

    args = parser.parse_args()
    cmd = args.command
    if cmd is not None:
        args.func(args)

    else:
        print "Missing argument"


#------------------------------------------------------------------------------
