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
    "/test/sensing/lidar/left/pointcloud",
    "/test/sensing/lidar/right/pointcloud",
    "/test/sensing/lidar/top/pointcloud",
]
FRAME_ID_LISTS = [
    "left_lidar",
    "right_lidar",
    "top_lidar",
]

TIMEOUT_SEC = 0.01
NUM_OF_POINTS = 3
# write a code that can fixed this value without moving as the number of point is always 3
TIMESTAMP_OFFSET_PER_POINT = 0.010


@pytest.mark.launch_test
def generate_test_description():
    nodes = []

    nodes.append(
        ComposableNode(
            package="pointcloud_preprocessor",
            plugin="pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerComponent",
            name="synchoronize_concatenate_node",
            remappings=[
                ("~/input/twist", "/test/sensing/vehicle_velocity_converter/twist_with_covariance"),
                ("output", "/test/sensing/lidar/concatenated/pointcloud"),
            ],
            parameters=[
                {
                    "input_topics": INPUT_LIDAR_TOPICS,
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
    if flag:
        list_points = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    else:
        list_points = []

    np_points = np.array(list_points, dtype=np.float32).reshape(-1, 3)
    points = np_points.tobytes()
    return points


def create_timestamps(flag: bool, header):

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
    tf_right_lidar_msg.transform.translation.z = 5.0
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
    tf_left_lidar_msg.transform.translation.z = 5.0
    tf_left_lidar_msg.transform.rotation.x = 0.0
    tf_left_lidar_msg.transform.rotation.y = 0.0
    tf_left_lidar_msg.transform.rotation.z = 0.0
    tf_left_lidar_msg.transform.rotation.w = 1.0

    return [tf_top_lidar_msg, tf_right_lidar_msg, tf_left_lidar_msg]


def generate_twist_msg():
    twist_header = Header()
    twist_header.stamp = rclpy.clock.Clock().now().to_msg()
    twist_header.frame_id = "base_link"
    twist_msg = TwistWithCovarianceStamped()
    twist_msg.header = twist_header
    twist_msg.twist.twist.linear.x = 1.0

    return twist_msg


def calculate_number_of_points(msg):
    # Each point's size (in bytes) is the sum of the sizes of all its fields
    point_step = msg.point_step
    print("point_step: ", point_step)
    # Total data size (in bytes) of the point cloud
    data_size = len(msg.data)
    print("data_size: ", data_size)
    # Number of points is the total data size divided by the size of each point
    num_points = data_size // point_step

    return num_points


def pointcloud2_to_xyz_array(cloud_msg):
    cloud_arr = np.frombuffer(cloud_msg.data, dtype=np.float32)
    cloud_arr = np.reshape(cloud_arr, (cloud_msg.width, int(cloud_msg.point_step / 4)))
    return cloud_arr[:, :3]


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
        self.msg_buffer = []
        self.twist_publisher, self.pointcloud_publishers = self.create_pub_sub()

    def tearDown(self):
        # called when each test finished
        self.node.destroy_node()

    def callback(self, msg: PointCloud2):
        self.msg_buffer.append(msg)

    def create_pub_sub(self):
        # QoS profile for sensor data
        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        # Publishers
        twist_publisher = self.node.create_publisher(
            TwistWithCovarianceStamped,
            "/test/sensing/vehicle_velocity_converter/twist_with_covariance",
            10,
        )

        pointcloud_publishers = {}
        for idx, input_lidar_topic in enumerate(INPUT_LIDAR_TOPICS):
            pointcloud_publishers[idx] = self.node.create_publisher(
                PointCloud2,
                input_lidar_topic,
                qos_profile=sensor_qos,
            )

        # create subscriber
        self.msg_buffer = []
        # subscribe to occupancy grid with buffer
        self.node.create_subscription(
            PointCloud2,
            "/test/sensing/lidar/concatenated/pointcloud",
            self.callback,
            qos_profile=sensor_qos,
        )

        return twist_publisher, pointcloud_publishers

    def test_normal_input_and_tf(self):
        # wait for the node to be ready
        time.sleep(3)

        # fill the twist msg
        twist_msg = generate_twist_msg()
        self.twist_publisher.publish(twist_msg)

        for frame_idx in range(len(INPUT_LIDAR_TOPICS)):
            pointcloud_msg = get_pointcloud_msg(True, frame_idx)
            self.pointcloud_publishers[frame_idx].publish(pointcloud_msg)
            time.sleep(0.001)

        rclpy.spin_once(self.node, timeout_sec=0.1)

        # test normal situation
        self.assertEqual(
            len(self.msg_buffer),
            1,
            "The number of concatenate pointscloud has different number as expected.",
        )
        self.assertEqual(
            calculate_number_of_points(self.msg_buffer[0]),
            NUM_OF_POINTS * len(FRAME_ID_LISTS),
            "The concatendate pointcloud has a different number of point as expected",
        )

        # test tf
        self.assertEqual(
            self.msg_buffer[0].header.frame_id,
            "base_link",
            "The concatendate pointcloud frame id is not base_link",
        )

        # test transformed points
        expected_array = np.array(
            [
                [1.0, 0.0, 5.0],
                [0.0, 1.0, 5.0],
                [0.0, 0.0, 6.0],
                [1.0208441, 5.0, 5.0],
                [0.02084414, 6.0, 5.0],
                [0.02084414, 5.0, 6.0],
                [1.0419736, -5.0, 5.0],
                [0.04197362, -4.0, 5.0],
                [0.04197362, -5.0, 6.0],
            ],
            dtype=np.float32,
        )

        cloud_arr = pointcloud2_to_xyz_array(self.msg_buffer[0])
        print("cloud_arr: ", cloud_arr)
        self.assertTrue(
            np.allclose(cloud_arr, expected_array, atol=1e-3),
            "The concatenation node have wierd output",
        )

    # def test_abnormal_null_pointcloud(self):
    #     """
    #     Test normal situation.
    #     One of the coming pointcloud have no data.
    #     input: pointclouds from three different topics.
    #     output: concatenate pointcloud
    #     """
    #     # wait for the node to be ready
    #     time.sleep(3)

    #     # fill the twist msg
    #     twist_msg = generate_twist_msg()
    #     self.twist_publisher.publish(twist_msg)

    #     for frame_idx in range(len(INPUT_LIDAR_TOPICS) - 1):
    #         pointcloud_msg = get_pointcloud_msg(True, frame_idx)
    #         #print("pointcloud_msg ", str(frame_idx), " : ", pointcloud_msg)
    #         self.pointcloud_publishers[frame_idx].publish(pointcloud_msg)
    #         time.sleep(0.01)

    #     pointcloud_msg = get_pointcloud_msg(False, len(INPUT_LIDAR_TOPICS) - 1)
    #     #print("pointcloud_msg ", str(len(INPUT_LIDAR_TOPICS) - 1), " : ", pointcloud_msg)
    #     self.pointcloud_publishers[len(INPUT_LIDAR_TOPICS) - 1].publish(pointcloud_msg)

    #     rclpy.spin_once(self.node, timeout_sec=0.1)

    #     # test normal situation
    #     self.assertEqual(
    #         len(self.msg_buffer),
    #         1,
    #         "The number of concatenate pointscloud has different number as expected.",
    #     )
    #     self.assertEqual(
    #         calculate_number_of_points(self.msg_buffer[0]),
    #         NUM_OF_POINTS * (len(FRAME_ID_LISTS) - 1),
    #         "The concatendate pointcloud has a different number of point as expected",
    #     )

    # def test_abnormal_pointcloud_drop(self):
    #     """
    #     Test abnormal situation.
    #     One of the pointcloud drop before timeout.
    #     Node should concatenate the remain pointcloud.
    #     input: pointclouds from three different topics.
    #     output: concatenate pointcloud
    #     """
    #     # wait for the node to be ready
    #     time.sleep(3)

    #     # fill the twist msg
    #     twist_msg = generate_twist_msg()
    #     self.twist_publisher.publish(twist_msg)

    #     for frame_idx in range(len(INPUT_LIDAR_TOPICS) - 1):
    #         pointcloud_msg = get_pointcloud_msg(True, frame_idx)
    #         self.pointcloud_publishers[frame_idx].publish(pointcloud_msg)
    #         time.sleep(0.02)

    #     rclpy.spin_once(self.node, timeout_sec=0.1)

    #     # Should receive only one concatenate pointcloud
    #     self.assertEqual(
    #         len(self.msg_buffer),
    #         1,
    #         "The number of concatenate pointscloud has different number as expected.",
    #     )

    #     self.assertEqual(
    #         calculate_number_of_points(self.msg_buffer[0]),
    #         NUM_OF_POINTS * (len(FRAME_ID_LISTS) - 1),
    #         "The concatendate pointcloud has a different number of point as expected",
    #     )

    # def test_abnormal_pointcloud_delay(self):
    #     """Test abnormal situation.
    #     One of the pointcloud come after timeout.
    #     Node should concatenate the remain pointcloud first
    #     and then concatenate the delay pointcloud
    #     input: pointclouds from three different topics.
    #     output: concatenate pointcloud
    #     """
    #     # wait for the node to be ready
    #     time.sleep(3)

    #     # fill the twist msg
    #     twist_msg = generate_twist_msg()
    #     self.twist_publisher.publish(twist_msg)

    #     for frame_idx in range(len(INPUT_LIDAR_TOPICS) - 1):
    #         pointcloud_msg = get_pointcloud_msg(True, frame_idx)
    #         #print("pointcloud_msg ", str(frame_idx), " : ", pointcloud_msg)
    #         self.pointcloud_publishers[frame_idx].publish(pointcloud_msg)
    #         time.sleep(0.02)

    #     time.sleep(0.1)
    #     rclpy.spin_once(self.node, timeout_sec=0.1)

    #     pointcloud_msg = get_pointcloud_msg(True, len(INPUT_LIDAR_TOPICS) - 1)
    #     #print("pointcloud_msg ", str(frame_idx), " : ", pointcloud_msg)
    #     self.pointcloud_publishers[len(INPUT_LIDAR_TOPICS) - 1].publish(pointcloud_msg)
    #     time.sleep(0.1)

    #     rclpy.spin_once(self.node, timeout_sec=0.1)

    #     print("len of msg buffer: ", len(self.msg_buffer))
    #     # Should receive only one concatenate pointcloud
    #     self.assertEqual(
    #         len(self.msg_buffer),
    #         2,
    #         "The number of concatenate pointscloud has different number as expected.",
    #     )

    #     self.assertEqual(
    #         calculate_number_of_points(self.msg_buffer[0]),
    #         NUM_OF_POINTS * (len(FRAME_ID_LISTS) - 1),
    #         "The concatendate pointcloud has a different number of point as expected",
    #     )

    #     self.assertEqual(
    #         calculate_number_of_points(self.msg_buffer[1]),
    #         NUM_OF_POINTS,
    #         "The concatendate pointcloud has a different number of point as expected",
    #     )

    # # Occur error with current software
    # def test_abnormal_pointcloud_drop_continue_normal(self):
    #     """Test abnormal situation.
    #     One of the pointcloud drop. Next pointclouds comes normal
    #     Node should concatenate the remain pointcloud first
    #     and then concatenate the next iteration pointclouds
    #     input: pointclouds from three different topics.
    #     output: concatenate pointcloud
    #     """
    #     # wait for the node to be ready
    #     time.sleep(3)

    #     # fill the twist msg
    #     twist_msg = generate_twist_msg()
    #     self.twist_publisher.publish(twist_msg)

    #     for frame_idx in range(len(INPUT_LIDAR_TOPICS) - 1):
    #         pointcloud_msg = get_pointcloud_msg(True, frame_idx)
    #         #print("pointcloud_msg ", str(frame_idx), " : ", pointcloud_msg)
    #         self.pointcloud_publishers[frame_idx].publish(pointcloud_msg)
    #         time.sleep(0.01)

    #     time.sleep(0.01)

    #     for frame_idx in range(len(INPUT_LIDAR_TOPICS)):
    #         pointcloud_msg = get_pointcloud_msg(True, frame_idx)
    #         #print("pointcloud_msg ", str(frame_idx), " : ", pointcloud_msg)
    #         self.pointcloud_publishers[frame_idx].publish(pointcloud_msg)
    #         time.sleep(0.01)

    #     rclpy.spin_once(self.node)

    #     print("len of msg buffer: ", len(self.msg_buffer))
    #     # Should receive only one concatenate pointcloud
    #     self.assertEqual(
    #         len(self.msg_buffer),
    #         2,
    #         "The number of concatenate pointscloud has different number as expected.",
    #     )

    #     self.assertEqual(
    #         calculate_number_of_points(self.msg_buffer[0]),
    #         NUM_OF_POINTS * (len(FRAME_ID_LISTS) - 1),
    #         "The concatendate pointcloud has a different number of point as expected",
    #     )

    #     self.assertEqual(
    #         calculate_number_of_points(self.msg_buffer[1]),
    #         NUM_OF_POINTS * len(FRAME_ID_LISTS),
    #         "The concatendate pointcloud has a different number of point as expected",
    #     )

    # Unit tests
    # Test convert to xyzi
    # Test computeTransformToAdjustForOldTimestamp
