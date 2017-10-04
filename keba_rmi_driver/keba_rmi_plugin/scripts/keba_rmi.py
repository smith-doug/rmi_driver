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

# Todo: Stuff.  This was made in a couple hours.
from robot_movement_interface import msg as rmi_msg
import rospy



# ----------------------------------------------------


# Positions
class RmiPos(object):
    def __init__(self, pose, pose_type='', aux_values=None):
        ':type aux_values: list[str]'
        self.pose = pose
        self.pose_type = pose_type
        
        self.aux_values = []
        if aux_values is not None:
            self.aux_values = aux_values
    
    def SetCmd(self, cmd):
        ':param rmi_msg.Command cmd:'
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
        assert(isinstance(cmd, rmi_msg.Command))       
        
        
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
        'type blending: list[int]'
        RmiBlending.__init__(self, blending, 'OBLABS')

# ----------------------------------------------------    
# Object class that handles the robot instructions/syntax
class RobotPost(object):    
    
    def __init__(self, topic='command_list'):        
        
        self.num_commands = 0  # Incremented at the start of every method that adds a command.  If it doesn't match the cmd_list.commands len there was a problem.
        
        self.cmd_list = rmi_msg.CommandList()
        
        self.pub = rospy.Publisher(topic, rmi_msg.CommandList, latch=True, queue_size=10)
        
    def ProgStart(self):   
        self.cmd_list = rmi_msg.CommandList()
        
    def ProgRun(self):
        """Publish the command list"""
      
        assert(self.num_commands == len(self.cmd_list.commands))
        rospy.logout('Publishing a CommandList with ' + str(len(self.cmd_list.commands)) + ' commands')
        self.pub.publish(self.cmd_list)
        
    
        
    def MoveJ(self, pose, dynamic=None, overlap=None):
        ':type pose: RmiPos'
        ':type dynamic: RmiVelo'
        ':type overlap: RmiBlending'
        ':type additional: RmiAdditionalParams'
        
        """PTP moves
        
        """
        
        self.num_commands += 1
      
        
        cmd = rmi_msg.Command()        
        cmd.command_type = "PTP"
        
        if(isinstance(pose, RmiPos)):
            cmd.pose_type = pose.pose_type
            cmd.pose = pose.pose                      
        
        if(isinstance(dynamic, RmiVelo)):     
                 
            cmd.velocity = dynamic.velocity
            cmd.velocity_type = dynamic.velocity_type
            
        
            
        if(len(cmd.pose_type) < 1):
            assert(False)
        else:
            self.cmd_list.commands.append(cmd)
        
        
                    
    def MoveL(self, pose, dynamic=None, overlap=None):
        ':param RmiPos pose:'
        
        """
        Lin moves
        """
 
        self.num_commands += 1
        
        cmd = rmi_msg.Command()        
        cmd.command_type = "LIN"
        
        if(isinstance(pose, RmiPos)):
            # cmd.pose_type = pose.pose_type
            # cmd.pose = pose.pose
            pose.SetCmd(cmd)
                                  
                                  
        
        if(isinstance(dynamic, RmiVelo)):           
            cmd.velocity = dynamic.velocity
            cmd.velocity_type = dynamic.velocity_type    
            
        if(len(cmd.pose_type) < 1):
            pass
        else:
            self.cmd_list.commands.append(cmd)
        
    
    def Settings(self, dynamic=None, overlap=None):
        ':type dynamic: RmiVelo'
        ':type overlap: RmiBlending'
        
        self.num_commands += 1
        
        # Give me something
        assert(dynamic is not None or overlap is not None)
        
        cmd = rmi_msg.Command()  
        cmd.command_type = "SETTING"
        
        if(dynamic is not None):
            assert(isinstance(dynamic, RmiVelo))
            dynamic.SetCmd(cmd)
            
        
        if(overlap is not None):
            assert(isinstance(overlap, RmiBlending))
            cmd.blending = overlap.blending
            cmd.blending_type = overlap.blending_type
            
            
        self.cmd_list.commands.append(cmd)
    
        
