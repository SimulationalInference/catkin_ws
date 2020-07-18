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

from std_msgs.msg import String



class ROS2Publisher(Node):

    def __init__(self, topic_name):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, topic_name, 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.msgs = [
            '{"ev":"XA","pair":"BTC-USD","v":725.09209352,"vw":9114.502,"z":0.65,"o":9111.14,"c":9134.3,"h":9134.7,"l":9106.5,"s":1594917720000,"e":1594917780000}',
            '{"ev":"XA","pair":"BTC-USD","v":251.92763934,"vw":9116.4002,"z":0.27,"o":9113.52,"c":9134.1,"h":9134.10645159,"l":9106.6,"s":1594917780000,"e":1594917840000}',
            '{"ev":"XA","pair":"BTC-USD","v":101.86543115,"vw":9119.478,"z":0.17,"o":9111.95,"c":9134.1,"h":9134.1,"l":9106.9,"s":1594917840000,"e":1594917900000}',
            '{"ev":"XA","pair":"BTC-USD","v":98.45410564,"vw":9118.3075,"z":0.16,"o":9116.85,"c":9119.69,"h":9133.83889483,"l":9108.4,"s":1594917900000,"e":1594917960000}',
            '{"ev":"XA","pair":"BTC-USD","v":725.19209352,"vw":9114.5047,"z":0.65,"o":9111.14,"c":9134.2,"h":9134.7,"l":9106.5,"s":1594917720000,"e":1594917780000}'
        ]

    def timer_callback(self):
        msg = String()
        msg.data = self.msgs[self.i]
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = ROS2Publisher("test")

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
