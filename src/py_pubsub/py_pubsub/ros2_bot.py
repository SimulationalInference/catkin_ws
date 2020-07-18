# Copyright 2016 Open Source Robotics Foundation, Inc.
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

import rclpy
from rclpy.node import Node
import numpy

from std_msgs.msg import String

# import backtrader as bt
import json
import datetime

import queue
import threading

from .bt_thread import BacktraderThread, data_queue

class SubscriberThread(threading.Thread):
    def __init__(self, topic_name="test"):
        threading.Thread.__init__(self)
        self.topic_name = topic_name
    def run(self):    
        rclpy.init()

        subscriber = ROS2Subscriber(self.topic_name)
        rclpy.spin(subscriber)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        subscriber.destroy_node()
        rclpy.shutdown()


class ROS2Subscriber(Node):
    def __init__(self, topic_name):
        super().__init__('ros2_subscriber')
        self.get_logger().info('test')
        self.subscription = self.create_subscription(
            String,
            topic_name,
            self.listener_callback,
            10)
        
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        global data_queue
        data_queue.put(msg.data)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    a = BacktraderThread()
    b = SubscriberThread()
    a.start()
    b.start()
    a.join()
    b.join()



if __name__ == '__main__':
    main()
