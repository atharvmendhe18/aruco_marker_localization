#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from math import degrees
import tf_transformations  

class ImuToYawNode(Node):
    def __init__(self):
        super().__init__('imu_to_angle')
        self.publisher_ = self.create_publisher(Float32, '/yaw', 4)
        self.subscription = self.create_subscription(
            Imu,
            '/imu_plugin/out',
            self.imu_callback,
            4
        )
        self.msg_float = Float32()
    def imu_callback(self, msg):
        x, y, z, w = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        euler = tf_transformations.euler_from_quaternion([x, y, z, w])
        yaw = degrees(euler[2])
        if yaw < 0:
            yaw += 360
        print(yaw)    
        self.msg_float.data = float(yaw)
        self.publisher_.publish(self.msg_float)

def main(args=None):
    rclpy.init(args=args)
    node = ImuToYawNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
