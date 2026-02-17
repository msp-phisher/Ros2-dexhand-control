#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class HandPulser(Node):
    def __init__(self):
        super().__init__('hand_pulser')
        
        # Publisher to the controller topic
        self.publisher_ = self.create_publisher(Float64MultiArray, '/hand_controller/commands', 10)
        
        # Create a timer to run every 2 seconds
        self.timer = self.create_timer(2.0, self.timer_callback)
        
        self.is_contracted = False
        self.get_logger().info("Hand Pulser Node Started. Controlling 5 joints...")

    def timer_callback(self):
        msg = Float64MultiArray()
        
        # We have 5 joints in our YAML list:
        # [Index, Middle, Ring, Pinky, Thumb]
        
        if self.is_contracted:
            # RELAX: Set all joints to 0.0 radians
            msg.data = [0.0, 0.0, 0.0, 0.0, 0.0]
            self.get_logger().info('Action: Relaxing (Opening Hand)')
        else:
            # CONTRACT: Set all joints to 1.0 radians (approx 60 degrees)
            msg.data = [1.0, 1.0, 1.0, 1.0, 1.0]
            self.get_logger().info('Action: Contracting (Closing Hand)')

        self.publisher_.publish(msg)
        
        # Toggle state for next time
        self.is_contracted = not self.is_contracted

def main(args=None):
    rclpy.init(args=args)
    node = HandPulser()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
