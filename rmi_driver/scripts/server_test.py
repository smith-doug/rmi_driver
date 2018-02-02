#!/usr/bin/env python
import socket
import sys
import signal
import rospy
import copy

import threading
import time

import SocketServer


class RobotData():
    def __init__(self, num_joints):
        self.joint_pos = ''
        for i in range(0, num_joints):
            self.joint_pos += '0.' + str(i) + ' '
        self.joint_pos = self.joint_pos.strip()

        self.target_pos = copy.deepcopy(self.joint_pos)
        self.is_moving = False
        self.inter_step = 0
        self.start_pos = copy.deepcopy(self.joint_pos)


class RmiServer(SocketServer.StreamRequestHandler):
    allow_reuse_address = True

    def __init__(self, request, client_address, server):

        self.step_target = 20.0  # number of steps to use in interpolation

        try:
            SocketServer.StreamRequestHandler.__init__(self, request, client_address, server)
        except:
            pass

    def interpolate(self, target_pt, start_pt, step):
        '''
        Very basic interpolator so that the model won't jump instantly        
        '''
        if step >= self.step_target:
            self.server.robot_data.is_moving = False
            return target_pt

        start_num = [float(n) for n in start_pt.split()]
        target_num = [float(n) for n in target_pt.split()]

        inter_pt = []
        for target_joint, start_joint in zip(target_num, start_num):
            inter_pt.append(start_joint + (step / self.step_target) * (target_joint - start_joint))

        ret_val = ''
        for pt in inter_pt:
            ret_val += str(pt) + ' '

        return ret_val.strip()

    def handle(self):
        print "New connection from " + str(self.client_address)

        while True:
            try:
                data = self.rfile.readline().strip()

                if not data.startswith('get'):
                    print data

                send_data = 'OK'
                if data.startswith('abort'):
                    self.server.robot_data.is_moving = False
                    self.server.robot_data.target_pos = self.server.robot_data.joint_pos
                    self.server.robot_data.inter_step = 10000.0

                    send_data = 'aborted'

                if data.startswith('get version'):
                    send_data = '0.0.8'
                elif data.startswith('get joint position'):

                    self.server.robot_data.joint_pos = self.interpolate(self.server.robot_data.target_pos,
                                                                        self.server.robot_data.start_pos, self.server.robot_data.inter_step)

                    if self.server.robot_data.inter_step <= self.step_target:
                        self.server.robot_data.inter_step += 1

                    send_data = self.server.robot_data.joint_pos

                elif data.startswith('get tool frame ros'):
                    send_data = '0.0 0.1 0.2 0.3 0.4 0.5'

                elif data.startswith('ptp joints'):
                    words = data.split(';')
                    values = words[0].split(':')[1].strip()

                    self.server.robot_data.start_pos = copy.deepcopy(self.server.robot_data.joint_pos)
                    self.server.robot_data.inter_step = 0
                    self.server.robot_data.target_pos = values
                    self.server.robot_data.is_moving = True

                    while self.server.robot_data.inter_step < (self.step_target / 1.2):
                        time.sleep(0.001)

                    # continue

                    #self.server.robot_data.joint_pos = values
                elif data.startswith('ping'):
                    send_data = 'pong'
                elif data.startswith('wait is_finished'):
                    while self.server.robot_data.is_moving:
                        time.sleep(0.001)
                    send_data = 'OK'
                else:
                    send_data = 'ERROR not implemented'

                if not send_data.endswith('\n'):
                    send_data += '\n'

                self.wfile.write(send_data)
            except Exception as ex:
                # print 'except in RmiServer handle ' + str(ex)

                break


class ThreadedTCPServer(SocketServer.ThreadingMixIn, SocketServer.TCPServer):
    daemon_threads = True

    def __init__(self, server_address, RequestHandlerClass, robot_data):
        self.robot_data = robot_data

        SocketServer.TCPServer.__init__(self, server_address, RmiServer)

    pass


class RobotInstance():

    def __init__(self, ip, port, num_joints):

        self.robot_data = RobotData(num_joints)
        self.server_cmd = ThreadedTCPServer((ip, port), RmiServer, self.robot_data)
        self.server_get = ThreadedTCPServer((ip, port + 1), RmiServer, self.robot_data)

    def start(self):
        self.thread_cmd = threading.Thread(target=self.server_cmd.serve_forever)
        self.thread_get = threading.Thread(target=self.server_get.serve_forever)

        self.thread_cmd.daemon = True
        self.thread_get.daemon = True

        self.thread_cmd.start()
        self.thread_get.start()

    def shutdown(self):
        self.server_cmd.shutdown()
        self.server_get.shutdown()

        self.server_cmd.server_close()
        self.server_get.server_close()


rob_list = []

try:
    SocketServer.TCPServer.allow_reuse_address = True

    driver_map = rospy.get_param("/rmi_driver_map")

    for driver in driver_map:
        ip = '0.0.0.0'
        port = driver['port']
        num_joints = len(driver['joints'])
        print "Creating a robot at " + ip + ":" + str(port) + " with " + str(num_joints) + " joints"
        rob = RobotInstance(ip, port, num_joints)
        rob.start()
        rob_list.append(rob)

    while(1):
        time.sleep(1)

    # server_thread.join()
    # server2_thread.join()
except KeyboardInterrupt:
    print 'KeyboardInterrupt'
    for rob in rob_list:
        rob.shutdown()
