#!/usr/bin/env python3

# Copyright 2024 TIER IV, Inc.
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

import time
import unittest

# TODO: remove unused later
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import TwistWithCovarianceStamped
import launch
import launch.actions
from launch.logging import get_logger
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import launch_testing.tools
import numpy as np
import pytest
import rclpy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

logger = get_logger(__name__)

INPUT_LIDAR_TOPICS = [
    "/test/sensing/lidar/top/pointcloud",
    "/test/sensing/lidar/left/pointcloud",
    "/test/sensing/lidar/right/pointcloud",
]
FRAME_ID_LISTS = [
    "top_lidar",
    "left_lidar",
    "right_lidar",
]

INPUT_LIDAR_TOPICS_OFFSET = [0.025, 0.025, 0.01]
TIMEOUT_SEC = 0.075
NUM_OF_POINTS = 3
# write a code that can fixed this value without moving as the number of point is always 3
TIMESTAMP_OFFSET_PER_POINT = 0.01


@pytest.mark.launch_test
def generate_test_description():
    nodes = []

    nodes.append(
        ComposableNode(
            package="pointcloud_preprocessor",
            plugin="pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerComponent",
            name="concatenate_data",
            remappings=[
                ("~/input/twist", "/test/sensing/vehicle_velocity_converter/twist_with_covariance"),
                ("output", "/test/sensing/lidar/concatenated/pointcloud"),
            ],
            parameters=[
                {
                    "input_topics": INPUT_LIDAR_TOPICS,
                    "input_offset": INPUT_LIDAR_TOPICS_OFFSET,
                    "timeout_sec": TIMEOUT_SEC,
                    "output_frame": "base_link",
                    "input_twist_topic_type": "twist",
                }
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        )
    )

    container = ComposableNodeContainer(
        name="test_concateante_data_container",
        namespace="pointcloud_preprocessor",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=nodes,
        output="screen",
    )

    return launch.LaunchDescription(
        [
            container,
            # Start tests right away - no need to wait for anything
            launch_testing.actions.ReadyToTest(),
        ]
    )


def create_header(frame_id_index):
    header = Header()
    header.stamp = rclpy.clock.Clock().now().to_msg()
    header.frame_id = FRAME_ID_LISTS[frame_id_index]

    return header


def create_points(flag: bool):
    """
    Create a list of points based on the flag argument.

    Args:
    flag (bool): Determines the content of the list.

    Returns:
    # TODO
    # list: A list containing points if flag is True, otherwise an empty list.
    """
    if flag:
        list_points = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    else:
        list_points = []

    np_points = np.array(list_points, dtype=np.float32).reshape(-1, 3)
    points = np_points.tobytes()
    return points


def create_timestamps(flag: bool, header):
    """
    Create a list of timestamps based on the flag argument.

    Args:
    flag (bool): Determines the content of the list.

    Returns:
    # TODO

    """
    if flag:
        np_timestamps = np.array(
            [
                header.stamp.sec + header.stamp.nanosec * 1e-9 + TIMESTAMP_OFFSET_PER_POINT * i
                for i in range(NUM_OF_POINTS)
            ]
        ).astype(np.float64)
    else:
        np_timestamps = np.array([]).astype(np.float64)

    timestamps = np_timestamps.tobytes()
    return timestamps


def get_pointcloud_msg(is_generate_points: bool, frame_id_index: int):
    """
    Create ros2 point cloud message from list of points.

    Args:
        pts (list): list of points [[x, y, z], ...]

    Returns:
        PointCloud2: ros2 point cloud message
    """
    header = create_header(frame_id_index)
    points = create_points(is_generate_points)
    timestamps = create_timestamps(is_generate_points, header)

    pointcloud_data = b"".join(
        points[i * 12 : i * 12 + 12] + timestamps[i * 8 : i * 8 + 8] for i in range(len(points))
    )
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="time_stamp", offset=12, datatype=PointField.FLOAT64, count=1),
    ]
    pointcloud_msg = PointCloud2(
        header=header,
        height=1,
        width=10,
        is_dense=True,
        is_bigendian=False,
        point_step=20,  # 4 float32 fields * 4 bytes/field
        row_step=20 * NUM_OF_POINTS,  # point_step * width
        fields=fields,
        data=pointcloud_data,
    )

    return pointcloud_msg


def generate_static_transform_msg():
    """
    Generate static transform message from base_link to map.

    Returns:
        TransformStamped: static transform message
    """
    tf_top_lidar_msg = TransformStamped()
    tf_top_lidar_msg.header.stamp = rclpy.clock.Clock().now().to_msg()
    tf_top_lidar_msg.header.frame_id = "base_link"
    tf_top_lidar_msg.child_frame_id = FRAME_ID_LISTS[0]
    tf_top_lidar_msg.transform.translation.x = 0.0
    tf_top_lidar_msg.transform.translation.y = 0.0
    tf_top_lidar_msg.transform.translation.z = 5.0
    tf_top_lidar_msg.transform.rotation.x = 0.0
    tf_top_lidar_msg.transform.rotation.y = 0.0
    tf_top_lidar_msg.transform.rotation.z = 0.0
    tf_top_lidar_msg.transform.rotation.w = 1.0

    tf_right_lidar_msg = TransformStamped()
    tf_right_lidar_msg.header.stamp = rclpy.clock.Clock().now().to_msg()
    tf_right_lidar_msg.header.frame_id = "base_link"
    tf_right_lidar_msg.child_frame_id = FRAME_ID_LISTS[1]
    tf_right_lidar_msg.transform.translation.x = 0.0
    tf_right_lidar_msg.transform.translation.y = 5.0
    tf_right_lidar_msg.transform.translation.z = 0.0
    tf_right_lidar_msg.transform.rotation.x = 0.0
    tf_right_lidar_msg.transform.rotation.y = 0.0
    tf_right_lidar_msg.transform.rotation.z = 0.0
    tf_right_lidar_msg.transform.rotation.w = 1.0

    tf_left_lidar_msg = TransformStamped()
    tf_left_lidar_msg.header.stamp = rclpy.clock.Clock().now().to_msg()
    tf_left_lidar_msg.header.frame_id = "base_link"
    tf_left_lidar_msg.child_frame_id = FRAME_ID_LISTS[2]
    tf_left_lidar_msg.transform.translation.x = 0.0
    tf_left_lidar_msg.transform.translation.y = -5.0
    tf_left_lidar_msg.transform.translation.z = 0.0
    tf_left_lidar_msg.transform.rotation.x = 0.0
    tf_left_lidar_msg.transform.rotation.y = 0.0
    tf_left_lidar_msg.transform.rotation.z = 0.0
    tf_left_lidar_msg.transform.rotation.w = 1.0

    return [tf_top_lidar_msg, tf_right_lidar_msg, tf_left_lidar_msg]


class TestConcatenateNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Init ROS at once
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        # Shutdown ROS at once
        rclpy.shutdown()

    def setUp(self):
        # called when each test started
        self.node = rclpy.create_node("test_concat_node")
        # send static transform from map to base_link
        tf_msg = generate_static_transform_msg()
        self.tf_broadcaster = StaticTransformBroadcaster(self.node)
        self.tf_broadcaster.sendTransform(tf_msg)

        self.twist_pub, self.pointcloud_pub = self.create_pub_sub()

    def tearDown(self):
        # called when each test finished
        self.node.destroy_node()

    def create_pub_sub(self):
        # QoS profile for sensor data
        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        # Publishers
        twist_pub = self.node.create_publisher(
            TwistWithCovarianceStamped,
            "/test/sensing/vehicle_velocity_converter/twist_with_covariance",
            10,
        )

        pointcloud_pub = {}
        for input_lidar_topic in INPUT_LIDAR_TOPICS:
            pointcloud_pub[input_lidar_topic] = self.node.create_publisher(
                PointCloud2,
                input_lidar_topic,
                qos_profile=sensor_qos,
            )

        return twist_pub, pointcloud_pub

    def test_normal_input(self):
        # wait for the node to be ready
        time.sleep(3)
        pass
