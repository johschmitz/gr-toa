#
# Copyright 2013 Free Software Foundation, Inc.
#
# This file is part of GNU Radio.
#
# This is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
#
# This software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this software; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.
#

import zmq
import threading


class zmq_manager():
    def __init__(self):
        self.zmq_context = zmq.Context()
        self.poller = zmq.Poller()
        self.poll_time_ms = 100
        self.interfaces = []
        self.shutdown = False

    def add_socket(self, id, address, callback_func):
        print("Adding socket with id \"%s\" and address %s" % (id, address))
        socket = self.zmq_context.socket(zmq.SUB)
        socket.connect(address)
        socket.setsockopt_string(zmq.SUBSCRIBE, u"")
        # Use a tuple to store interface elements
        # interface[0] : id
        # interface[1] : socket
        # interface[2] : address
        # interface[3] : callback_func
        self.interfaces.append((id, socket, address, callback_func))
        self.poller.register(socket, zmq.POLLIN)

    def remove_socket(self, id):
        for i in range(0,len(self.interfaces)):
            if self.interfaces[i][0] == id:
                print("Removing socket with id \"%s\" and address %s" % (id, self.interfaces[i][2]))
                self.interfaces[i][1].close()
                del self.interfaces[i]
                return

    def poll_socket(self):
        # poll expects ms
        poll = dict(self.poller.poll(self.poll_time_ms))
        for i in self.interfaces:
            # i in [0:3] -> (id, socket, address, callback_func)
            if poll.get(i[1]) == zmq.POLLIN:
                # receive data
                msg_packed = i[1].recv()
                # invoke callback function (i[0] -> id)
                i[3](i[0],msg_packed)

    def watcher(self):
        while(True):
            self.poll_socket()
            if self.shutdown:
                return

    def start_watcher(self, poll_time_ms):
        self.poll_time_ms = poll_time_ms
        self.watcher_thread = threading.Thread(target=self.watcher)
        self.watcher_thread.daemon = True
        self.watcher_thread.start()

    def stop_watcher(self):
        self.shutdown = True
