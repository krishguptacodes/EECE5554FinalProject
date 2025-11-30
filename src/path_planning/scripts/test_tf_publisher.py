#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math


class TestTFPublisher(Node):
    """
    发布静态 TF 变换用于测试路径规划
    模拟一个静态机器人位置
    """
    def __init__(self):
        super().__init__('test_tf_publisher')
        
        # 创建 TF 广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 创建定时器，10Hz 发布 TF
        self.timer = self.create_timer(0.1, self.publish_tf)
        
        # 机器人初始位置
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        
        self.get_logger().info("Test TF Publisher initialized - Publishing map->base_link transform")
        self.get_logger().info(f"Robot position: x={self.robot_x}, y={self.robot_y}, yaw={self.robot_yaw}")
    
    def publish_tf(self):
        """发布 map -> base_link 变换"""
        t = TransformStamped()
        
        # 设置时间戳和坐标系
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        
        # 设置位置
        t.transform.translation.x = self.robot_x
        t.transform.translation.y = self.robot_y
        t.transform.translation.z = 0.0
        
        # 设置方向 (从欧拉角转换为四元数)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.robot_yaw / 2.0)
        t.transform.rotation.w = math.cos(self.robot_yaw / 2.0)
        
        # 广播变换
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    
    publisher = TestTFPublisher()
    
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

