#!/usr/bin/env python3

import rclpy
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from sensor_driver_msgs.msg import GpswithHeading

def process_bag(bag_path, output_file):
    # 设置存储选项
    storage_options = StorageOptions(
        uri=bag_path,
        storage_id='sqlite3'
    )
    
    # 设置转换选项
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    
    # 创建读取器
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    
    # 打开输出文件
    with open(output_file, 'w') as f:
        # 读取bag中的所有消息
        while reader.has_next():
            (topic, data, timestamp) = reader.read_next()
            
            if topic == '/gpsdata':
                # 反序列化消息
                msg = deserialize_message(data, GpswithHeading)
                
                # 提取纬度和经度
                latitude = msg.gps.latitude
                longitude = msg.gps.longitude
                
                # 写入文件，格式为"纬度 经度"
                f.write(f"{latitude},{longitude}\n")
    
    print(f"数据已成功写入 {output_file}")

if __name__ == '__main__':
    # 替换为你的bag包路径和输出文件路径
    bag_path = '/home/polirman/autonomous/record_data/0923/rosbag2_2025_09_23-15_59_50'  # bag包目录路径（不含.db3扩展名）
    output_file = 'gps_coordinates.txt'
    
    process_bag(bag_path, output_file)
