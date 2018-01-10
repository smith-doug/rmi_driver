#!/usr/bin/env python
import socket
import sys
import signal

import threading
import time

import SocketServer


class RobotData():
    def __init__(self, num_joints):
        self.joint_pos = ''
        for i in range(0, num_joints):
            self.joint_pos += '0.' + str(i) + ' '
        self.joint_pos = self.joint_pos.strip()


class RmiServer(SocketServer.StreamRequestHandler):
    allow_reuse_address = True

    # def __init__(self, request, client_address, server):

    def handle(self):

        while True:
            try:
                data = self.rfile.readline().strip()

                if not data.startswith('get'):
                    print data

                send_data = 'OK'
                if data.startswith('get version'):
                    send_data = '0.0.7'
                elif data.startswith('get joint position'):
                    send_data = self.server.robot_data.joint_pos
                elif data.startswith('get tool frame ros'):
                    send_data = '0.0 0.1 0.2 0.3 0.4 0.5'

                elif data.startswith('ptp joints'):
                    words = data.split(';')
                    values = words[0].split(':')[1].strip()
                    self.server.robot_data.joint_pos = values

                if not send_data.endswith('\n'):
                    send_data += '\n'

                self.wfile.write(send_data)
            except:
                print 'except in RmiServer handle'
                break


class ThreadedTCPServer(SocketServer.ThreadingMixIn, SocketServer.TCPServer):
    daemon_threads = True
    #     allow_reuse_address = True
    #

    def __init__(self, server_address, RequestHandlerClass, robot_data):
        self.robot_data = robot_data

        SocketServer.TCPServer.__init__(self, server_address, RmiServer)

    pass


class RobotInstance():

    def __init__(self, ip, port):

        self.robot_data = RobotData(7)
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


TCP_IP = '0.0.0.0'
TCP_PORT = 30000


threads = []


try:
    SocketServer.TCPServer.allow_reuse_address = True

#     server = ThreadedTCPServer((TCP_IP, TCP_PORT), RmiServer)
#
#     server2 = ThreadedTCPServer((TCP_IP, TCP_PORT + 1), RmiServer)
#
#     server_thread = threading.Thread(target=server.serve_forever)
#     server_thread.daemon = True
#     server_thread.start()
#
#     server2_thread = threading.Thread(target=server2.serve_forever)
#     server2_thread.daemon = True
#     server2_thread.start()

    rob = RobotInstance(TCP_IP, TCP_PORT)
    rob.start()

    while(1):
        time.sleep(1)

    # server_thread.join()
    # server2_thread.join()
except KeyboardInterrupt:
    print 'KeyboardInterrupt'


# server.serve_forever()


# for t in threads:
#    t.join()
