#!/usr/bin/env python3

# Copyright 2026 Christopher Newport University
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Background-thread ROS 2 subscriber that feeds WGCFStatus messages into a queue for the Pygame main loop."""

from datetime import datetime
import queue
import threading
import traceback

import rclpy
from rclpy.node import Node


class MsgSubscriber(Node):
    """Message Subscriber class."""

    def __init__(self, node_name, topic_name, msg_type, callback):
        """Create a ROS 2 node and subscribe to topic_name, routing each message to callback."""
        super().__init__(node_name)
        self.subscription = self.create_subscription(
            msg_type,
            topic_name,
            callback,
            10)
        self.subscription  # prevent unused variable warning


class MsgHandler(threading.Thread):
    """Base class to manage communication with ROS."""

    def __init__(self, node_name, topic_name, msg_type, args):
        """Initialize ROS 2 message handler."""
        print(f'MsgHandler: args={args}', flush=True)

        threading.Thread.__init__(self)
        self.daemon = True
        self.read_queue = queue.Queue()
        self._shutdown_started = threading.Event()

        # Pass any ROS command line args
        if not rclpy.ok():
            rclpy.init(args=args)

        self.node = MsgSubscriber(node_name, topic_name, msg_type, self.callback)

    def __del__(self):
        """Force shutdown on deletion."""
        # Don't use logger during shutdown
        try:
            print('Shutting down the msg_handler thread ...', flush=True)
            self.shutdown()
        except Exception as exc:
            print(type(exc), exc, flush=True)

    def shutdown(self):
        """Make sure ROS connection is closed and thread is stopped."""
        if self._shutdown_started.is_set():
            return
        self._shutdown_started.set()

        try:
            rclpy.try_shutdown()
        except Exception as exc:  # pylint: disable=W0703
            print(f'{datetime.now()} - Exception from rclpy.shutdown'
                  f' for msg handler: {type(exc)}\n{exc}', flush=True)
            print(f"{traceback.format_exc().replace('%', '%%')}", flush=True)

    @property
    def is_empty(self):
        """Check queue empty."""
        return self.read_queue.empty()

    def callback(self, msg):
        """Put message in queue."""
        self.read_queue.put(msg)

    def data_generator(self):
        """Get msg from queue in thread safe manner."""
        try:
            while True:
                # Get the latest data from serial input
                # get blocks until more data is available
                yield self.read_queue.get()

        except KeyboardInterrupt as kbi:
            print('data_generator: Keyboard interrupt detected')
            self.shutdown()
            raise kbi
        except Exception as exc:
            print(exc)
            raise exc

    def run(self):
        """Run the message handler."""
        try:
            print('Running the msg_handler ...', flush=True)
            rclpy.spin(self.node)
        except KeyboardInterrupt:
            print(f'Keyboard interrupt request! {datetime.now()}', flush=True)
        except rclpy.executors.ExternalShutdownException:  # pylint: disable=W0703
            print(f'shutdown {datetime.now()}', flush=True)
        except Exception as exc:  # pylint: disable=W0703
            print(f'shutdown {datetime.now()} - Exception from msg handler '
                  f'spin: {type(exc)}\n{exc}', flush=True)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        run_owns_shutdown = not self._shutdown_started.is_set()
        if run_owns_shutdown:
            self._shutdown_started.set()
        print('Destroy msg_handler ROS node ...', flush=True)
        self.node.destroy_node()
        try:
            if run_owns_shutdown and rclpy.ok():
                rclpy.shutdown()
        except Exception as exc:  # pylint: disable=W0703
            print(f'{datetime.now()} - Exception from rclpy.shutdown'
                  f' for msg handler: {type(exc)}\n{exc}', flush=True)
            print(f"{traceback.format_exc().replace('%', '%%')}", flush=True)
        print('Done with msg_handler!', flush=True)
