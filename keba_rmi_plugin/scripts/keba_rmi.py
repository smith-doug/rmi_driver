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


# This started as an implementation of the robodk processor but it wasn't flexible enough.
# See the global define section at the bottom for Kairo compatibility

# Todo: Stuff.  This was made in a couple hours.
from robot_movement_interface import msg as rmi_msg
import rospy
import functools

# ----------------------------------------------------


# Positions


class RmiPos(object):
    '''

    '''

    def __init__(self, pose, pose_type='', aux_values=None):
        '''
        A position

        :param pose: Position values
        :type pose: list[float]
        :param pose_type: 
        :type pose_type: str
        :param aux_values:
        :type aux_values: list[str]        
        '''

        self.pose = pose
        self.pose_type = pose_type

        self.aux_values = []
        if aux_values is not None:
            self.aux_values = aux_values

    def SetCmd(self, cmd):
        '''
        Update cmd with the data in this pose
        :param cmd: The Command to update
        :type cmd: rmi_msg.Command
        '''

        # assert isinstance(cmd, Command)
        cmd.pose = self.pose
        cmd.pose_type = self.pose_type

        for aux in self.aux_values:
            cmd.additional_parameters.append(aux)


class RmiPosJoints(RmiPos):
    def __init__(self, pose):
        # Joint positions don't have aux values
        super(RmiPosJoints, self).__init__(pose, 'JOINTS')


class RmiPosQuaternion(RmiPos):
    def __init__(self, pose, aux_values=None):
        super(RmiPosQuaternion, self).__init__(pose, 'QUATERNION', aux_values)


class RmiPosEulerZyx(RmiPos):
    def __init__(self, pose, aux_values=None):
        super(RmiPosEulerZyx, self).__init__(pose, 'EULER_INTRINSIC_ZYX', aux_values)


# Dynamics

class RmiVelo(object):
    def __init__(self, velocity, velocity_type):
        self.velocity = velocity
        self.velocity_type = velocity_type

    def SetCmd(self, cmd):
        ':param rmi_msg.Command cmd:'
        #assert(isinstance(cmd, rmi_msg.Command))


class RmiDyn(RmiVelo):
    def __init__(self, dynamic):
        super(RmiDyn, self).__init__(dynamic, 'DYN')

    def SetCmd(self, cmd):
        RmiVelo.SetCmd(self, cmd)
        cmd.velocity_type = self.velocity_type
        cmd.velocity = self.velocity


# Overlapping
class RmiBlending(object):
    def __init__(self, blending, blending_type):
        self.blending_type = blending_type
        self.blending = blending

    def SetCmd(self, cmd):
        ':param rmi_msg.Command cmd:'
        cmd.blending_type = self.blending_type
        cmd.blending = self.blending


class RmiOvlRel(RmiBlending):
    def __init__(self, percent):
        ':type percent: int'
        RmiBlending.__init__(self, [percent], 'OVLREL')


class RmiOvlSuppos(RmiBlending):
    def __init__(self, percent):
        ':type percent: int'
        RmiBlending.__init__(self, [percent], 'OVLSUPPOS')


class RmiOvlAbs(RmiBlending):
    def __init__(self, blending):
        'type blending: list[float]'
        RmiBlending.__init__(self, blending, 'OVLABS')


# ----------------------------------------------------
# Object class that handles the robot instructions/syntax


class RobotPost(object):

    def __init__(self, topic_command='command_list', topic_result='command_result'):

        # Incremented at the start of every method that adds a command.  If it
        # doesn't match the cmd_list.commands len there was a problem.
        self.num_commands = 0
        self.num_results = 0

        self.cmd_list = rmi_msg.CommandList()

        self.pub = rospy.Publisher(topic_command, rmi_msg.CommandList, latch=True, queue_size=10)
        self.sub = rospy.Subscriber(topic_result, rmi_msg.Result, callback=self.CommandResultCb)

    def CommandResultCb(self, data):
        '''
        Subscriber callback
        :param data: The result
        :type data: rmi_msg.Result
        '''

        res_str = 'OK(0)'
        log_func = rospy.logout  # info by default

        if data.result_code != 0:  # some kind of error
            res_str = 'ERROR(' + str(data.result_code) + '), ' + data.additional_information
            log_func = rospy.logerr  # log to error
            self.num_results = self.num_commands  # Bail out immediately

        log_func('Result ' + str(data.command_id) + ': ' + res_str)
        self.num_results += 1

    def ProgStart(self):
        self.cmd_list = rmi_msg.CommandList()
        self.num_commands = 0

    def ProgRun(self):
        """Publish the command list"""

        self.num_results = 0

        assert(self.num_commands == len(self.cmd_list.commands))
        rospy.logout('Publishing a CommandList with ' + str(len(self.cmd_list.commands)) + ' commands')

        for cmd_num, cmd in enumerate(self.cmd_list.commands):  # : :type cmd: rmi_msg.Command
            rospy.logout('Command ' + str(cmd_num) + ': ' + cmd.command_type)

        self.pub.publish(self.cmd_list)

        while(self.num_results < self.num_commands and not rospy.is_shutdown()):
            rospy.sleep(0.5)

        rospy.logout('ProgRun exiting')

    def AddCommand(self, cmd):
        '''
        Add a command.  It will set command_id to self.num_commands and then increment self.num_commands.  
        :param cmd: The command to add
        :type cmd: rmi_msg.Command
        '''

        cmd.command_id = self.num_commands
        self.cmd_list.commands.append(cmd)

        self.num_commands += 1

    def MoveJ(self, pose, dynamic=None, overlap=None):
        '''
        PTP moves
        :param pose: A position (joints, quaternion, euler)
        :type pose: RmiPos
        :param dynamic: Dynamic containing velo/accel
        :type dynamic: RmiVelo
        :param overlap: Blending
        :type overlap: RmiBlending
        '''

        #self.num_commands += 1

        cmd = rmi_msg.Command()
        cmd.command_type = "PTP"

        assert isinstance(pose, RmiPos)
        pose.SetCmd(cmd)

        if(isinstance(dynamic, RmiVelo)):
            dynamic.SetCmd(cmd)

        if isinstance(overlap, RmiBlending):
            overlap.SetCmd(cmd)

        if(len(cmd.pose_type) < 1):
            assert(False)
        else:
            self.AddCommand(cmd)
            # self.cmd_list.commands.append(cmd)

    def MoveL(self, pose, dynamic=None, overlap=None):
        '''
        Linear moves
        :param pose: A position (joints, quaternion, euler)
        :type pose: RmiPos
        :param dynamic: Dynamic containing velo/accel
        :type dynamic: RmiVelo
        :param overlap: Blending
        :type overlap: RmiBlending
        '''

        #self.num_commands += 1

        cmd = rmi_msg.Command()
        cmd.command_type = "LIN"

        assert isinstance(pose, RmiPos)
        pose.SetCmd(cmd)

        if isinstance(dynamic, RmiVelo):
            dynamic.SetCmd(cmd)

        if isinstance(overlap, RmiBlending):
            overlap.SetCmd(cmd)

        if(len(cmd.pose_type) < 1):
            pass
        else:
            self.AddCommand(cmd)
            # self.cmd_list.commands.append(cmd)

    def Settings(self, dynamic=None, overlap=None):
        '@type dynamic: RmiVelo'
        '@type overlap: RmiBlending'

        #self.num_commands += 1

        # Give me something
        assert(dynamic is not None or overlap is not None)

        cmd = rmi_msg.Command()
        cmd.command_type = "SETTING"

        if(isinstance(dynamic, RmiVelo)):
            dynamic.SetCmd(cmd)

        if isinstance(overlap, RmiBlending):
            overlap.SetCmd(cmd)

        self.AddCommand(cmd)
        # self.cmd_list.commands.append(cmd)

    def WaitIsFinished(self):
        '''
        Call WaitIsFinished()
        '''

        #self.num_commands += 1

        cmd = rmi_msg.Command()
        cmd.command_type = 'WAIT'
        cmd.pose_type = 'IS_FINISHED'

        self.AddCommand(cmd)
        # self.cmd_list.commands.append(cmd)



#===============================================================================
# Global defines for Kairo look and feel
#===============================================================================
default_rob = RobotPost()

# Positions
CARTPOS = RmiPosEulerZyx  # ([x, y, z, a, b, c], aux_values=None)
QUATPOS = RmiPosQuaternion  # ([x, y, z, rw, rx, ry, rz], aux_values=None)
AXISPOS = RmiPosJoints  # ([a1, a2, ...], aux_values=None)

#Dynamics and Overlaps
OVLABS = RmiOvlAbs  # ([posDist, oriDist, linAxDist, rotAxDist, vConst(boolean)])
OVLSUPPOS = RmiOvlSuppos  # (ovl %)
OVLREL = RmiOvlRel  # (ovl %)
# ([velAxis(0..100->), accAxis, decAxis, jerkAxis, vel(mm/s->), acc, dec, jerk, velOri(deg/s->), accOri, decOri, jerkOri])
DYNAMIC = RmiDyn

# Commands
Lin = default_rob.MoveL  # (pose, dynamic=None, overlap=None)
PTP = default_rob.MoveJ  # (pose, dynamic=None, overlap=None)
Settings = default_rob.Settings  # (dynamic=None, overlap=None)

WaitIsFinished = default_rob.WaitIsFinished


def Dyn(dynamic):
    '''
    calls Dyn
    :param dynamic: the dynamic
    :type dynamic: RmiDyn
    '''
    default_rob.Settings(dynamic=dynamic)


def Ovl(overlap):
    '''
    calls Ovl
    :param overlap: OVERLAP_
    :type overlap: RmiBlending
    '''
    default_rob.Settings(overlap=overlap)


#------------------------------------------------------------------------------
