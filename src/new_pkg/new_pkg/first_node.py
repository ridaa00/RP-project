 #!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class MyNode(Node): #this is to define now a times to call some function evey x sec and to keep count of t

    def __init__(self):
        super().__init__("py_node")
        self.counter_ = 0
        self.get_logger().info("Hello Ros2")
        self.create_timer(0.5,self.timer_callback)

    def timer_callback(self):
        self.counter_ += 1
        self.get_logger().info("Hello Ros2" + str(self.counter_))


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()# node name different from python file
    
    # Uncomment the next line if you want the node to keep running until stopped manually
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
