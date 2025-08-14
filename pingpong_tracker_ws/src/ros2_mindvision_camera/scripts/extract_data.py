#!/usr/bin/env python3
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message_type_from_name

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import pandas as pd
import os

def extract_data_from_bag(bag_base_path):
    """
    从ROS2 bag文件中提取图像和camera_info，并保存为数据集。
    :param bag_base_path: 录制bag文件的基础路径，不包含db3或zst后缀。
    """
    print(f"正在处理压缩的 bag 文件: {bag_base_path}")

    storage_options = rosbag2_py.StorageOptions(
        uri=bag_base_path,
        storage_id='sqlite3',
        # 自动解压文件
        compressed_file_uri=bag_base_path + '_0.db3.zst'
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

    bridge = CvBridge()
    camera_info = None
    output_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'extracted_data')
    os.makedirs(output_dir, exist_ok=True)

    data_records = []
    image_count = 0

    while reader.has_next():
        (topic_name, data, timestamp) = reader.read_next()

        # 只处理 camera_info，因为它在录制中通常是静态的
        if topic_name == '/camera_info':
            msg_type = get_message_type_from_name(type_map[topic_name])
            camera_info = deserialize_message(data, msg_type)
            print("已成功提取 CameraInfo。")

        # 当有 camera_info 后，处理 image
        elif topic_name == '/image_raw' and camera_info is not None:
            msg_type = get_message_type_from_name(type_map[topic_name])
            image_msg = deserialize_message(data, msg_type)

            try:
                cv_image = bridge.imgmsg_to_cv2(image_msg, "bgr8")
            except Exception as e:
                print(f"转换图像失败: {e}")
                continue

            # 进行畸变矫正
            camera_matrix = np.array(camera_info.k).reshape((3, 3))
            dist_coeffs = np.array(camera_info.d)
            undistorted_image = cv2.undistort(cv_image, camera_matrix, dist_coeffs)

            # 保存畸变矫正后的图像
            image_filename = f"frame_{image_count:05d}.jpg"
            image_path = os.path.join(output_dir, image_filename)
            cv2.imwrite(image_path, undistorted_image)

            # 记录数据
            data_records.append({
                'timestamp': timestamp,
                'image_file': image_filename,
                'camera_info_k': list(camera_info.k),
                'camera_info_d': list(camera_info.d)
            })

            image_count += 1
            if image_count % 100 == 0:
                print(f"已处理并保存 {image_count} 张图像。")

    # 保存为 CSV 文件
    if data_records:
        df = pd.DataFrame(data_records)
        csv_path = os.path.join(output_dir, "dataset.csv")
        df.to_csv(csv_path, index=False)
        print(f"所有数据已保存到 {csv_path}")
    else:
        print("未找到有效数据进行保存。")

if __name__ == '__main__':
    # 请替换为你的 bag 文件的基础路径
    # 例如: "/tmp/camera_recordings/2025-08-14_10-55-43"
    bag_path = "/tmp/camera_recordings/2025-08-14_10-55-43"
    extract_data_from_bag(bag_path)