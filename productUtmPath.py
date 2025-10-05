#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from lanelet_map_msgs.msg import Way, Node as WayNode
from geometry_msgs.msg import Point
import math
from pyproj import Proj

class GlobalPathPublisher(Node):
    def __init__(self):
        super().__init__('global_path_publisher')
        
        # 创建发布者，QoS设置为10（保持默认）
        self.publisher_ = self.create_publisher(Way, '/topology_global_path', 10)
        
        # 定义WGS84经纬度原点（中国吉林省延边地区）
        origin_lat = 42.39701299
        origin_lon = 126.70376794
        
        # 创建UTM投影（自动确定UTM区域）
        utm_zone = self.calculate_utm_zone(origin_lon)
        utm_proj = Proj(proj='utm', zone=utm_zone, ellps='WGS84')
        
        # 将原点经纬度转换为UTM坐标
        origin_x, origin_y = utm_proj(origin_lon, origin_lat)
        self.get_logger().info(f'Origin UTM coordinates: Easting={origin_x:.2f}, Northing={origin_y:.2f}, Zone={utm_zone}')
        
        # 初始化路径消息
        self.path_msg = self.create_path_message(origin_x, origin_y)
        
        # 创建10Hz定时器（0.1秒间隔）
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Started publishing global path at 10Hz')
    
    def create_path_message(self, origin_x, origin_y):
        """创建路径消息"""
        msg = Way()
        # 基本参数配置
        msg.task_area = "navigation"
        msg.is_forward = 1
        msg.open_concave_obs_det = 1
        msg.open_dynamic_obs_det = 1
        msg.open_foggy_det = 0
        msg.open_water_det = 0
        msg.foggy_area = 0
        msg.lawn_area = 0
        msg.water_area = 0
        msg.wall_area = 0
        msg.ditch_area = 0
        msg.vel_limit = 5.0    # 速度限制5m/s
        msg.road_width = 3.0   # 道路宽度3米
        
        # 创建路径点 (沿0度方向/正东方向延伸40米)
        num_points = 241
        yaw_angle = math.radians(90)  # 0度表示正东方向
        delta_x = 0.04
        
        for i in range(num_points):
            way_node = WayNode()
            way_node.id = i
            way_node.type = "path_point"
            way_node.vlimit = 5.0
            
            # 计算UTM坐标增量
            distance = 120.0 * (i / (num_points - 1))
            #delta_x = distance * math.cos(yaw_angle)
            delta_x = delta_x * -1
            delta_y = distance * math.sin(yaw_angle)
            
            # 设置点坐标
            way_node.point.x = origin_x + delta_x
            way_node.point.y = origin_y + delta_y 
            way_node.point.z = 0.0  # 2D路径，z设为0
            
            msg.points.append(way_node)
        
        return msg
    
    def timer_callback(self):
        """定时器回调函数，10Hz发布消息"""
        self.publisher_.publish(self.path_msg)
        self.get_logger().debug('Publishing path message', throttle_duration_sec=1.0)  # 限频日志输出
    
    def calculate_utm_zone(self, longitude):
        """计算给定经度的UTM区域号"""
        return int((longitude + 180) / 6) + 1

def main(args=None):
    rclpy.init(args=args)
    publisher = GlobalPathPublisher()
    
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        publisher.get_logger().info('Shutting down...')
    finally:
        publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
