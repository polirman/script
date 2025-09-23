#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from plan2control_msgs.msg import Trajectory, TrajNode
from geometry_msgs.msg import Point, Quaternion, Twist, Vector3
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

class StraightTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('straight_trajectory_publisher')
        
        # 定义 Best Effort QoS 策略
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # 尽力传输
            durability=QoSDurabilityPolicy.VOLATILE,        # 非持久化
            depth=10                                       # 队列大小
        )
        
        # 创建轨迹发布者（使用 Best Effort QoS）
        self.publisher = self.create_publisher(
            Trajectory, 
            '/global_path/traj_plan',
            qos_profile=qos_profile
        )
        
        # 创建10Hz定时器
        self.timer = self.create_timer(0.1, self.publish_straight_trajectory)
        
        self.get_logger().info("Straight trajectory publisher started (Best Effort QoS)")

    def publish_straight_trajectory(self):
        """发布间隔0.3米、共30个点的直线轨迹"""
        traj = Trajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.header.frame_id = 'ego_frame'
        traj.issued_acc = 0.0
        traj.delay_index = 0
        traj.notpassable = False
        traj.overtake_flag = False
        
        for i in range(30):
            node = TrajNode()
            node.id = i
            node.forward = True
            node.parkingpoint = False
            node.position = Point(x=0.3 * i, y=0.0, z=0.0)
            node.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            node.curvature = 0.0
            
            node.velocity = Twist()
            node.velocity.linear = Vector3(
                x=10.0 if i < 29 else 0.0,
                y=0.0,
                z=0.0
            )
            node.velocity.angular = Vector3(x=0.0, y=0.0, z=0.0)
            node.clearance = 10.0
            
            traj.points.append(node)
        
        self.publisher.publish(traj)
        self.get_logger().info(
            "Published trajectory (Best Effort)",
            throttle_duration_sec=1.0
        )

def main(args=None):
    rclpy.init(args=args)
    node = StraightTrajectoryPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Publisher stopped")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
