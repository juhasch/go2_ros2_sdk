# Copyright (c) 2024, RoboVerse community
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


import json
import logging
import os
import threading
import asyncio
import numpy as np


# Monkey-patch aioice.Connection to use a fixed username and password accross all instances.
import aioice

class Connection(aioice.Connection):
    local_username = aioice.utils.random_string(4)
    local_password = aioice.utils.random_string(22)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.local_username = Connection.local_username
        self.local_password = Connection.local_password

aioice.Connection = Connection  # type: ignore

from aiortc import MediaStreamTrack
from cv_bridge import CvBridge

from scripts.go2_constants import ROBOT_CMD, RTC_TOPIC
from scripts.go2_func import gen_command, gen_mov_command
from scripts.go2_lidar_decoder import update_meshes_for_cloud2
from scripts.go2_math import get_robot_joints
from scripts.go2_camerainfo import load_camera_info
from scripts.webrtc_driver import Go2Connection

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos_overriding_options import QoSOverridingOptions

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
from go2_interfaces.msg import Go2State, IMU
from unitree_go.msg import LowState, VoxelMapCompressed, WebRtcReq
from sensor_msgs.msg import PointCloud2, PointField, JointState, Joy
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo


logging.basicConfig(level=logging.WARN)
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class RobotBaseNode(Node):

    def __init__(self):
        super().__init__('go2_driver_node')

        self.declare_parameter('robot_ip', os.getenv(
            'ROBOT_IP', os.getenv('GO2_IP')))
        self.declare_parameter('token', os.getenv(
            'ROBOT_TOKEN', os.getenv('GO2_TOKEN', '')))
        self.declare_parameter('conn_type', os.getenv(
            'CONN_TYPE', os.getenv('CONN_TYPE', '')))
        self.declare_parameter('enable_video', True)
        self.declare_parameter('decode_lidar', True)
        self.declare_parameter('publish_raw_voxel', False)
        self.declare_parameter('obstacle_avoidance', False)
        self.declare_parameter('camera_resolution', '1080')

        self.robot_ip = self.get_parameter(
            'robot_ip').get_parameter_value().string_value
        self.token = self.get_parameter(
            'token').get_parameter_value().string_value
        self.robot_ip_lst = self.robot_ip.replace(" ", "").split(",")
        self.conn_type = self.get_parameter(
            'conn_type').get_parameter_value().string_value
        self.enable_video = self.get_parameter(
            'enable_video').get_parameter_value().bool_value
        self.decode_lidar = self.get_parameter(
            'decode_lidar').get_parameter_value().bool_value
        self.publish_raw_voxel = self.get_parameter(
            'publish_raw_voxel').get_parameter_value().bool_value
        self.obstacle_avoidance = self.get_parameter(
            'obstacle_avoidance').get_parameter_value().bool_value
        self.camera_resolution = self.get_parameter(
            'camera_resolution').get_parameter_value().string_value

        # Obstacle detection state tracking
        self.obstacle_detection_enabled = False
        self.obstacle_detection_remote_control = False

        self.conn_mode = "single" if (
            len(self.robot_ip_lst) == 1 and self.conn_type != "cyclonedds") else "multi"

        self.get_logger().info(f"Received ip list: {self.robot_ip_lst}")
        self.get_logger().info(f"Connection type is {self.conn_type}")
        self.get_logger().info(f"Connection mode is {self.conn_mode}")
        self.get_logger().info(f"Enable video is {self.enable_video}")
        self.get_logger().info(f"Decode lidar is {self.decode_lidar}")
        self.get_logger().info(
            f"Publish raw voxel is {self.publish_raw_voxel}")
        self.get_logger().info(f"Obstacle avoidance is {self.obstacle_avoidance}")
        self.get_logger().info(f"Camera resolution is {self.camera_resolution}")

        self.conn = {}
        qos_profile = QoSProfile(depth=10)
        best_effort_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.joint_pub = []
        self.go2_state_pub = []
        self.go2_lidar_pub = []
        self.go2_odometry_pub = []
        self.imu_pub = []
        self.img_pub = []
        self.camera_info_pub = []
        self.voxel_pub = []

        if self.conn_mode == 'single':
            self.joint_pub.append(self.create_publisher(
                JointState, 'joint_states', qos_profile))
            self.go2_state_pub.append(self.create_publisher(
                Go2State, 'go2_states', qos_profile))
            self.go2_lidar_pub.append(
                self.create_publisher(
                    PointCloud2,
                    'point_cloud2',
                    best_effort_qos,
                    qos_overriding_options=QoSOverridingOptions.with_default_policies()))
            self.go2_odometry_pub.append(
                self.create_publisher(Odometry, 'odom', qos_profile))
            self.imu_pub.append(self.create_publisher(IMU, 'imu', qos_profile))
            if self.enable_video:
                self.img_pub.append(
                    self.create_publisher(
                        Image,
                        'camera/image_raw',
                        best_effort_qos,
                        qos_overriding_options=QoSOverridingOptions.with_default_policies()))
                self.camera_info_pub.append(
                    self.create_publisher(
                        CameraInfo,
                        'camera/camera_info',
                        best_effort_qos,
                        qos_overriding_options=QoSOverridingOptions.with_default_policies()))
            if self.publish_raw_voxel:
                self.voxel_pub.append(
                    self.create_publisher(
                        VoxelMapCompressed,
                        '/utlidar/voxel_map_compressed',
                        best_effort_qos))

        else:
            for i in range(len(self.robot_ip_lst)):
                self.joint_pub.append(self.create_publisher(
                    JointState, f'robot{i}/joint_states', qos_profile))
                self.go2_state_pub.append(self.create_publisher(
                    Go2State, f'robot{i}/go2_states', qos_profile))
                self.go2_lidar_pub.append(
                    self.create_publisher(
                        PointCloud2,
                        f'robot{i}/point_cloud2',
                        best_effort_qos,
                        qos_overriding_options=QoSOverridingOptions.with_default_policies()))
                self.go2_odometry_pub.append(self.create_publisher(
                    Odometry, f'robot{i}/odom', qos_profile))
                self.imu_pub.append(self.create_publisher(
                    IMU, f'robot{i}/imu', qos_profile))
                if self.enable_video:
                    self.img_pub.append(
                        self.create_publisher(
                            Image,
                            f'robot{i}/camera/image_raw',
                            best_effort_qos,
                            qos_overriding_options=QoSOverridingOptions.with_default_policies()))
                    self.camera_info_pub.append(
                        self.create_publisher(
                            CameraInfo,
                            f'robot{i}/camera/camera_info',
                            best_effort_qos,
                            qos_overriding_options=QoSOverridingOptions.with_default_policies()))
                if self.publish_raw_voxel:
                    self.voxel_pub.append(
                        self.create_publisher(
                            VoxelMapCompressed,
                            f'robot{i}/utlidar/voxel_map_compressed',
                            best_effort_qos))

        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)

        self.bridge = CvBridge()
        self.camera_info = load_camera_info(self.camera_resolution)

        self.robot_cmd_vel = {}
        self.robot_odom = {}
        self.robot_low_cmd = {}
        self.robot_sport_state = {}
        self.robot_lidar = {}
        self.webrtc_msgs = asyncio.Queue()

        self.joy_state = Joy()

        if self.conn_mode == 'single':
            self.create_subscription(
                Twist,
                'cmd_vel',
                lambda msg: self.cmd_vel_cb(msg, "0"),
                qos_profile)
            self.create_subscription(
                WebRtcReq,
                'webrtc_req',
                lambda msg: self.webrtc_req_cb(msg, "0"),
                qos_profile)
        else:
            for i in range(len(self.robot_ip_lst)):
                self.create_subscription(
                    Twist,
                    f'robot{str(i)}/cmd_vel_out',
                    lambda msg: self.cmd_vel_cb(msg, str(i)),
                    qos_profile)
                self.create_subscription(
                    WebRtcReq,
                    f'robot{str(i)}/webrtc_req',
                    lambda msg: self.webrtc_req_cb(msg, str(i)),
                    qos_profile)

        self.create_subscription(
            Joy,
            'joy',
            self.joy_cb,
            qos_profile)

        # Support for CycloneDDS (EDU version via ethernet)
        if self.conn_type == 'cyclonedds':
            self.create_subscription(
                LowState,
                'lowstate',
                self.publish_joint_state_cyclonedds,
                qos_profile)

            self.create_subscription(
                PoseStamped,
                '/utlidar/robot_pose',
                self.publish_body_poss_cyclonedds,
                qos_profile)

            self.create_subscription(
                PointCloud2,
                '/utlidar/cloud',
                self.publish_lidar_cyclonedds,
                qos_profile)

    def cmd_vel_cb(self, msg, robot_num):
        x = msg.linear.x
        y = msg.linear.y
        z = msg.angular.z  # This is actually yaw (rotation around Z-axis)
 
        # Allow omni-directional movement
        if x != 0.0 or y != 0.0 or z != 0.0:
            # Use the existing gen_mov_command which handles obstacle avoidance correctly
            generated_command = gen_mov_command(
                round(x, 2), round(y, 2), round(z, 2), self.obstacle_avoidance)
            self.robot_cmd_vel[robot_num] = generated_command
            
            # Debug: Log the actual generated command
            try:
                command_data = json.loads(generated_command)
                topic = command_data.get('topic', 'unknown')
                api_id = command_data.get('data', {}).get('header', {}).get('identity', {}).get('api_id', 'unknown')
                parameters = command_data.get('data', {}).get('parameter', 'unknown')
                logger.info(f"Generated command - Topic: {topic}, API_ID: {api_id}, Parameters: {parameters}")
                logger.info(f"Sending move command with obstacle avoidance: {self.obstacle_avoidance}")
                logger.debug(f"Move command: x={x:.3f}, y={y:.3f}, z(yaw)={z:.3f}, obstacle_avoidance={self.obstacle_avoidance}")
            except Exception as e:
                logger.error(f"Error parsing generated command: {e}")
                logger.info(f"Sending move command with obstacle avoidance: {self.obstacle_avoidance}")
        else:
            # No movement command - ensure robot is stopped
            if self.obstacle_avoidance and self.obstacle_detection_enabled:
                # If obstacle avoidance is enabled and no movement command, send stop command
                logger.debug("No movement command - sending stop command for obstacle avoidance")
                self.robot_cmd_vel[robot_num] = gen_mov_command(0.0, 0.0, 0.0, True)
            elif not self.obstacle_avoidance:
                # If regular mode and no movement command, send stop command
                logger.debug("No movement command - sending stop command for regular mode")
                self.robot_cmd_vel[robot_num] = gen_mov_command(0.0, 0.0, 0.0, False)

    def webrtc_req_cb(self, msg, robot_num):
        parameter_str = msg.parameter if msg.parameter else ""
        try:
            parameter = json.loads(parameter_str)
        except ValueError as e:
            self.get_logger().error(f"Invalid JSON in WebRTC request: {e}")
            parameter = parameter_str
        payload = gen_command(msg.api_id, parameter, msg.topic, msg.id)
        self.get_logger().info(f"Received WebRTC request: {payload[:50]}")
        self.webrtc_msgs.put_nowait(payload)

    def joy_cb(self, msg):
        self.joy_state = msg

    def publish_body_poss_cyclonedds(self, msg):
        odom_trans = TransformStamped()
        odom_trans.header.stamp = self.get_clock().now().to_msg()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = "robot0/base_link"
        odom_trans.transform.translation.x = msg.pose.position.x
        odom_trans.transform.translation.y = msg.pose.position.y
        odom_trans.transform.translation.z = msg.pose.position.z + 0.07
        odom_trans.transform.rotation.x = msg.pose.orientation.x
        odom_trans.transform.rotation.y = msg.pose.orientation.y
        odom_trans.transform.rotation.z = msg.pose.orientation.z
        odom_trans.transform.rotation.w = msg.pose.orientation.w
        self.broadcaster.sendTransform(odom_trans)

    def publish_joint_state_cyclonedds(self, msg):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = [
            'robot0/FL_hip_joint',
            'robot0/FL_thigh_joint',
            'robot0/FL_calf_joint',
            'robot0/FR_hip_joint',
            'robot0/FR_thigh_joint',
            'robot0/FR_calf_joint',
            'robot0/RL_hip_joint',
            'robot0/RL_thigh_joint',
            'robot0/RL_calf_joint',
            'robot0/RR_hip_joint',
            'robot0/RR_thigh_joint',
            'robot0/RR_calf_joint',
        ]
        joint_state.position = [
            msg.motor_state[3].q, msg.motor_state[4].q, msg.motor_state[5].q,
            msg.motor_state[0].q, msg.motor_state[1].q, msg.motor_state[2].q,
            msg.motor_state[9].q, msg.motor_state[10].q, msg.motor_state[11].q,
            msg.motor_state[6].q, msg.motor_state[7].q, msg.motor_state[8].q,
        ]
        self.joint_pub[0].publish(joint_state)

    def publish_lidar_cyclonedds(self, msg):
        msg.header = Header(frame_id="robot0/radar")
        msg.header.stamp = self.get_clock().now().to_msg()
        self.go2_lidar_pub[0].publish(msg)

    def joy_cmd(self, robot_num):
        if robot_num in self.conn and robot_num in self.robot_cmd_vel and self.robot_cmd_vel[
                robot_num] is not None:
            self.get_logger().info("Move")
            # Debug: Log what's being sent
            try:
                command_to_send = self.robot_cmd_vel[robot_num]
                if isinstance(command_to_send, str):
                    command_data = json.loads(command_to_send)
                    topic = command_data.get('topic', 'unknown')
                    api_id = command_data.get('data', {}).get('header', {}).get('identity', {}).get('api_id', 'unknown')
                    self.get_logger().info(f"Sending command - Topic: {topic}, API_ID: {api_id}")
                else:
                    self.get_logger().info(f"Sending command (type: {type(command_to_send)})")
            except Exception as e:
                self.get_logger().error(f"Error parsing command before sending: {e}")
            
            self.conn[robot_num].data_channel.send(
                self.robot_cmd_vel[robot_num])
            self.robot_cmd_vel[robot_num] = None

        if robot_num in self.conn and self.joy_state.buttons and self.joy_state.buttons[1]:
            self.get_logger().info("Stand down")
            stand_down_cmd = gen_command(ROBOT_CMD["StandDown"])
            self.conn[robot_num].data_channel.send(stand_down_cmd)

        if robot_num in self.conn and self.joy_state.buttons and self.joy_state.buttons[0]:
            self.get_logger().info("Stand up")
            stand_up_cmd = gen_command(ROBOT_CMD["StandUp"])
            self.conn[robot_num].data_channel.send(stand_up_cmd)
            move_cmd = gen_command(ROBOT_CMD['BalanceStand'])
            self.conn[robot_num].data_channel.send(move_cmd)

        if robot_num in self.conn and self.joy_state.buttons and self.joy_state.buttons[2]:
            self.get_logger().info("Toggle obstacle detection")
            self.toggle_obstacle_detection(robot_num)
            
        # Add button 3 for testing movement modes (if available)
        if robot_num in self.conn and self.joy_state.buttons and len(self.joy_state.buttons) > 3 and self.joy_state.buttons[3]:
            self.get_logger().info("Testing movement modes")
            self.test_movement_modes(robot_num)
            
        # Add button 4 for force stopping obstacle avoidance (if available)
        if robot_num in self.conn and self.joy_state.buttons and len(self.joy_state.buttons) > 4 and self.joy_state.buttons[4]:
            self.get_logger().info("Force stopping obstacle avoidance")
            self.force_stop_obstacle_avoidance(robot_num)
            
        # Add button 5 for emergency stop (if available)
        if robot_num in self.conn and self.joy_state.buttons and len(self.joy_state.buttons) > 5 and self.joy_state.buttons[5]:
            self.get_logger().warn("EMERGENCY STOP triggered")
            self.emergency_stop(robot_num)

    def on_validated(self, robot_num):
        if robot_num in self.conn:
            for topic in RTC_TOPIC.values():
                self.conn[robot_num].data_channel.send(
                    json.dumps({"type": "subscribe", "topic": topic}))
            
            # Initialize obstacle detection status
            self.get_obstacle_detection_status(robot_num)

    async def on_video_frame(self, track: MediaStreamTrack, robot_num):
        logger.info(f"Video frame received for robot {robot_num}")

        while True:
            frame = await track.recv()
            img = frame.to_ndarray(format="bgr24")

            logger.debug(
                f"Shape: {img.shape}, Dimensions: {img.ndim}, Type: {img.dtype}, Size: {img.size}")

            # Convert the OpenCV image to ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
            ros_image.header.stamp = self.get_clock().now().to_msg()

            # Set the timestamp for both image and camera info
            camera_info = self.camera_info
            camera_info.header.stamp = ros_image.header.stamp

            if self.conn_mode == 'single':
                camera_info.header.frame_id = 'front_camera'
                ros_image.header.frame_id = 'front_camera'
            else:
                camera_info.header.frame_id = f'robot{str(robot_num)}/front_camera'
                ros_image.header.frame_id = f'robot{str(robot_num)}/front_camera'

            # Publish image and camera info
            self.img_pub[robot_num].publish(ros_image)
            self.camera_info_pub[robot_num].publish(camera_info)
            await asyncio.sleep(0)

    def on_data_channel_message(self, _, msg, robot_num):
        """Handle incoming WebRTC data and publish immediately."""
        
        if msg.get('topic') == RTC_TOPIC["ULIDAR_ARRAY"]:
            self.robot_lidar[robot_num] = msg
            # Publish LiDAR data immediately
            if self.conn_type == 'webrtc' and self.decode_lidar:
                self.publish_lidar_webrtc_immediate(robot_num, msg)
            
            # Publish raw voxel data immediately if enabled
            if self.conn_type == 'webrtc' and self.publish_raw_voxel:
                self.publish_voxel_webrtc_immediate(robot_num, msg)

        if msg.get('topic') == RTC_TOPIC['ROBOTODOM']:
            self.robot_odom[robot_num] = msg
            # Publish odometry data immediately
            if self.conn_type == 'webrtc':
                self.publish_odom_webrtc_immediate(robot_num, msg)
                self.publish_odom_topic_webrtc_immediate(robot_num, msg)

        if msg.get('topic') == RTC_TOPIC['LF_SPORT_MOD_STATE']:
            self.robot_sport_state[robot_num] = msg
            # Publish robot state and joint state immediately
            if self.conn_type == 'webrtc':
                self.publish_robot_state_webrtc_immediate(robot_num, msg)
                self.publish_joint_state_webrtc_immediate(robot_num, msg)
                
        if msg.get('topic') == RTC_TOPIC['MULTIPLE_STATE']:
            self.handle_multiple_state_message(robot_num, msg)

        if msg.get('topic') == RTC_TOPIC['LOW_STATE']:
            self.robot_low_cmd[robot_num] = msg
            
        if msg.get('topic') == RTC_TOPIC['OBSTACLES_AVOID']:
            self.get_logger().debug(f"Received OBSTACLES_AVOID message for robot {robot_num}")
            self.handle_obstacle_detection_response(robot_num, msg)

    def publish_webrtc_commands(self, robot_num):
        while True:
            try:
                message = self.webrtc_msgs.get_nowait()
                try:
                    self.conn[robot_num].data_channel.send(message)
                finally:
                    self.webrtc_msgs.task_done()
            except asyncio.QueueEmpty:
                break

    def publish_odom_webrtc_immediate(self, robot_num, msg):
        """Publish odometry transform immediately when received."""
        odom_trans = TransformStamped()
        odom_trans.header.stamp = self.get_clock().now().to_msg()
        odom_trans.header.frame_id = 'odom'

        if self.conn_mode == 'single':
            odom_trans.child_frame_id = "base_link"
        else:
            odom_trans.child_frame_id = f"robot{robot_num}/base_link"

        odom_trans.transform.translation.x = msg['data']['pose']['position']['x']
        odom_trans.transform.translation.y = msg['data']['pose']['position']['y']
        odom_trans.transform.translation.z = msg['data']['pose']['position']['z'] + 0.07
        odom_trans.transform.rotation.x = msg['data']['pose']['orientation']['x']
        odom_trans.transform.rotation.y = msg['data']['pose']['orientation']['y']
        odom_trans.transform.rotation.z = msg['data']['pose']['orientation']['z']
        odom_trans.transform.rotation.w = msg['data']['pose']['orientation']['w']
        self.broadcaster.sendTransform(odom_trans)

    def publish_odom_topic_webrtc_immediate(self, robot_num, msg):
        """Publish odometry topic immediately when received."""
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'

        if self.conn_mode == 'single':
            odom_msg.child_frame_id = "base_link"
        else:
            odom_msg.child_frame_id = f"robot{robot_num}/base_link"

        odom_msg.pose.pose.position.x = msg['data']['pose']['position']['x']
        odom_msg.pose.pose.position.y = msg['data']['pose']['position']['y']
        odom_msg.pose.pose.position.z = msg['data']['pose']['position']['z'] + 0.07
        odom_msg.pose.pose.orientation.x = msg['data']['pose']['orientation']['x']
        odom_msg.pose.pose.orientation.y = msg['data']['pose']['orientation']['y']
        odom_msg.pose.pose.orientation.z = msg['data']['pose']['orientation']['z']
        odom_msg.pose.pose.orientation.w = msg['data']['pose']['orientation']['w']
        self.go2_odometry_pub[int(robot_num)].publish(odom_msg)

    def publish_lidar_webrtc_immediate(self, robot_num, msg):
        """Publish LiDAR point cloud immediately when received."""
        decoded_data = msg["decoded_data"]
        
        # Handle both old mesh format and new point cloud format
        if "points" in decoded_data:
            # New native decoder format - direct point cloud
            points = decoded_data["points"]
            # Use intensity values if available, otherwise use height-based intensity
            if "intensities" in decoded_data and len(decoded_data["intensities"]) > 0:
                intensities = decoded_data["intensities"].astype(np.float32)
            elif len(points) > 0:
                # Fallback: use height-based intensity
                z_coords = points[:, 2]
                z_min, z_max = z_coords.min(), z_coords.max()
                if z_max > z_min:
                    intensities = (z_coords - z_min) / (z_max - z_min)
                else:
                    intensities = np.full(len(points), 0.5, dtype=np.float32)
            else:
                intensities = np.array([], dtype=np.float32)
            
            if len(points) > 0:
                points_with_intensity = np.column_stack([points, intensities])
            else:
                points_with_intensity = np.empty((0, 4), dtype=np.float32)
        else:
            # Old libvoxel format - convert mesh to point cloud
            points = update_meshes_for_cloud2(
                decoded_data["positions"],
                decoded_data["uvs"],
                msg['data']['resolution'],
                msg['data']['origin'],
                0
            )
            points_with_intensity = points
        
        point_cloud = PointCloud2()
        point_cloud.header = Header(frame_id="odom")
        point_cloud.header.stamp = self.get_clock().now().to_msg()
        fields = [
            PointField(name='x', offset=0,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12,
                       datatype=PointField.FLOAT32, count=1),
        ]
        point_cloud = point_cloud2.create_cloud(
            point_cloud.header, fields, points_with_intensity)
        self.go2_lidar_pub[int(robot_num)].publish(point_cloud)

    def publish_voxel_webrtc_immediate(self, robot_num, msg):
        """Publish raw voxel data immediately when received."""
        voxel_msg = VoxelMapCompressed()
        voxel_msg.stamp = float(msg['data']['stamp'])
        voxel_msg.frame_id = 'odom'
        voxel_msg.resolution = msg['data']['resolution']
        voxel_msg.origin = msg['data']['origin']
        voxel_msg.width = msg['data']['width']
        voxel_msg.src_size = msg['data']['src_size']
        voxel_msg.data = msg['compressed_data']
        self.voxel_pub[int(robot_num)].publish(voxel_msg)

    def publish_robot_state_webrtc_immediate(self, robot_num, msg):
        """Publish robot state immediately when received."""
        go2_state = Go2State()
        go2_state.mode = msg["data"]["mode"]
        go2_state.progress = msg["data"]["progress"]
        go2_state.gait_type = msg["data"]["gait_type"]
        go2_state.position = list(map(float, msg["data"]["position"]))
        go2_state.body_height = float(msg["data"]["body_height"])
        go2_state.velocity = msg["data"]["velocity"]
        go2_state.range_obstacle = list(map(float, msg["data"]["range_obstacle"]))
        go2_state.foot_force = msg["data"]["foot_force"]
        go2_state.foot_position_body = list(map(float, msg["data"]["foot_position_body"]))
        go2_state.foot_speed_body = list(map(float, msg["data"]["foot_speed_body"]))
        self.go2_state_pub[int(robot_num)].publish(go2_state)

        imu = IMU()
        imu.quaternion = list(map(float, msg["data"]["imu_state"]["quaternion"]))
        imu.accelerometer = list(map(float, msg["data"]["imu_state"]["accelerometer"]))
        imu.gyroscope = list(map(float, msg["data"]["imu_state"]["gyroscope"]))
        imu.rpy = list(map(float, msg["data"]["imu_state"]["rpy"]))
        imu.temperature = msg["data"]["imu_state"]["temperature"]
        self.imu_pub[int(robot_num)].publish(imu)

    def publish_joint_state_webrtc_immediate(self, robot_num, msg):
        """Publish joint state immediately when received."""
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()

        fl_foot_pos_array = [
            msg["data"]["foot_position_body"][3],
            msg["data"]["foot_position_body"][4],
            msg["data"]["foot_position_body"][5]
        ]
        FL_hip_joint, FL_thigh_joint, FL_calf_joint = get_robot_joints(fl_foot_pos_array, 0)

        fr_foot_pos_array = [
            msg["data"]["foot_position_body"][0],
            msg["data"]["foot_position_body"][1],
            msg["data"]["foot_position_body"][2]
        ]
        FR_hip_joint, FR_thigh_joint, FR_calf_joint = get_robot_joints(fr_foot_pos_array, 1)

        rl_foot_pos_array = [
            msg["data"]["foot_position_body"][9],
            msg["data"]["foot_position_body"][10],
            msg["data"]["foot_position_body"][11]
        ]
        RL_hip_joint, RL_thigh_joint, RL_calf_joint = get_robot_joints(rl_foot_pos_array, 2)

        rr_foot_pos_array = [
            msg["data"]["foot_position_body"][6],
            msg["data"]["foot_position_body"][7],
            msg["data"]["foot_position_body"][8]
        ]
        RR_hip_joint, RR_thigh_joint, RR_calf_joint = get_robot_joints(rr_foot_pos_array, 3)

        if self.conn_mode == 'single':
            joint_state.name = [
                'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
                'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
                'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
                'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
            ]
        else:
            joint_state.name = [
                f'robot{robot_num}/FL_hip_joint',
                f'robot{robot_num}/FL_thigh_joint',
                f'robot{robot_num}/FL_calf_joint',
                f'robot{robot_num}/FR_hip_joint',
                f'robot{robot_num}/FR_thigh_joint',
                f'robot{robot_num}/FR_calf_joint',
                f'robot{robot_num}/RL_hip_joint',
                f'robot{robot_num}/RL_thigh_joint',
                f'robot{robot_num}/RL_calf_joint',
                f'robot{robot_num}/RR_hip_joint',
                f'robot{robot_num}/RR_thigh_joint',
                f'robot{robot_num}/RR_calf_joint']

        joint_state.position = [
            FL_hip_joint, FL_thigh_joint, FL_calf_joint,
            FR_hip_joint, FR_thigh_joint, FR_calf_joint,
            RL_hip_joint, RL_thigh_joint, RL_calf_joint,
            RR_hip_joint, RR_thigh_joint, RR_calf_joint,
        ]

        # Use low state motor data if available for more accurate joint positions
        if self.robot_low_cmd and robot_num in self.robot_low_cmd:
            motors = self.robot_low_cmd[robot_num]['data']['motor_state']
            joint_state.position = [
                motors[3]['q'], motors[4]['q'], motors[5]['q'],
                motors[0]['q'], motors[1]['q'], motors[2]['q'],
                motors[9]['q'], motors[10]['q'], motors[11]['q'],
                motors[6]['q'], motors[7]['q'], motors[8]['q'],
            ]

        self.joint_pub[int(robot_num)].publish(joint_state)

    def toggle_obstacle_detection(self, robot_num):
        """Toggle obstacle detection on/off for the specified robot."""
        if robot_num not in self.conn:
            self.get_logger().warn(f"Robot {robot_num} not connected")
            return
            
        try:
            # First, stop any current movement to prevent double movement
            self.get_logger().info(f"Stopping current movement before mode switch for robot {robot_num}")
            self.stop_all_movement(robot_num)
            
            # Wait a moment for the stop command to take effect
            import time
            time.sleep(0.2)
            
            # Toggle the state
            self.obstacle_detection_enabled = not self.obstacle_detection_enabled
            action = "enabling" if self.obstacle_detection_enabled else "disabling"
            self.get_logger().info(f"{action.capitalize()} obstacle detection for robot {robot_num}")
            
            # Send the command to the robot
            if self.obstacle_detection_enabled:
                self.get_logger().info(f"Calling enable_obstacle_detection for robot {robot_num}")
                self.enable_obstacle_detection(robot_num)
            else:
                self.get_logger().info(f"Calling disable_obstacle_detection for robot {robot_num}")
                self.disable_obstacle_detection(robot_num)
            
            # Update the obstacle avoidance parameter
            self.get_logger().info(f"Calling update_obstacle_avoidance_parameter for robot {robot_num}")
            self.update_obstacle_avoidance_parameter(robot_num)
                
        except Exception as e:
            self.get_logger().error(f"Error toggling obstacle detection: {e}")

    def enable_obstacle_detection(self, robot_num):
        """Enable obstacle detection for the specified robot."""
        try:
            # Send enable command via SwitchSet (api_id=1001)
            command = {
                "type": "msg",
                "topic": RTC_TOPIC["OBSTACLES_AVOID"],
                "data": {
                    "header": {
                        "identity": {
                            "id": int(asyncio.get_event_loop().time() * 1000) % 2147483648,
                            "api_id": 1001
                        }
                    },
                    "parameter": json.dumps({"enable": True})
                }
            }
            
            self.conn[robot_num].data_channel.send(json.dumps(command))
            self.get_logger().info(f"Obstacle detection enabled for robot {robot_num}")
            
            # Enable remote control for obstacle avoidance (required for API commands to work)
            self.get_logger().info(f"Enabling remote control for obstacle avoidance for robot {robot_num}")
            self.set_obstacle_remote_control(robot_num, True)
            
        except Exception as e:
            self.get_logger().error(f"Error enabling obstacle detection: {e}")

    def disable_obstacle_detection(self, robot_num):
        """Disable obstacle detection for the specified robot."""
        try:
            # Send disable command via SwitchSet (api_id=1001)
            command = {
                "type": "msg",
                "topic": RTC_TOPIC["OBSTACLES_AVOID"],
                "data": {
                    "header": {
                        "identity": {
                            "id": int(asyncio.get_event_loop().time() * 1000) % 2147483648,
                            "api_id": 1001
                        }
                    },
                    "parameter": json.dumps({"enable": False})
                }
            }
            
            self.conn[robot_num].data_channel.send(json.dumps(command))
            self.get_logger().info(f"Obstacle detection disabled for robot {robot_num}")
            
        except Exception as e:
            self.get_logger().error(f"Error disabling obstacle detection: {e}")

    def get_obstacle_detection_status(self, robot_num):
        """Get current obstacle detection status for the specified robot."""
        try:
            # Send status query via SwitchGet (api_id=1002)
            command = {
                "type": "msg",
                "topic": RTC_TOPIC["OBSTACLES_AVOID"],
                "data": {
                    "header": {
                        "identity": {
                            "id": int(asyncio.get_event_loop().time() * 1000) % 2147483648,
                            "api_id": 1002
                        }
                    },
                    "parameter": json.dumps({})
                }
            }
            
            self.conn[robot_num].data_channel.send(json.dumps(command))
            self.get_logger().info(f"Obstacle detection status query sent for robot {robot_num}")
            
        except Exception as e:
            self.get_logger().error(f"Error querying obstacle detection status: {e}")

    def set_obstacle_remote_control(self, robot_num, enable=True):
        """Enable/disable remote API control for obstacle avoidance (api_id=1004)."""
        try:
            command = {
                "type": "msg",
                "topic": RTC_TOPIC["OBSTACLES_AVOID"],
                "data": {
                    "header": {
                        "identity": {
                            "id": int(asyncio.get_event_loop().time() * 1000) % 2147483648,
                            "api_id": 1004
                        }
                    },
                    "parameter": json.dumps({"is_remote_commands_from_api": enable})
                }
            }
            
            self.conn[robot_num].data_channel.send(json.dumps(command))
            self.obstacle_detection_remote_control = enable
            action = "enabled" if enable else "disabled"
            self.get_logger().info(f"Remote obstacle control {action} for robot {robot_num}")
            
        except Exception as e:
            self.get_logger().error(f"Error setting remote obstacle control: {e}")

    def handle_obstacle_detection_response(self, robot_num, msg):
        """Handle obstacle detection response messages."""
        try:
            self.get_logger().debug(f"Handling obstacle detection response for robot {robot_num}: {msg}")
            if 'data' in msg and 'header' in msg['data']:
                api_id = msg['data']['header'].get('identity', {}).get('api_id')
                self.get_logger().debug(f"Obstacle detection response API ID: {api_id}")
                
                if api_id == 1002:  # SwitchGet response
                    # Parse the response to get current status
                    if 'data' in msg['data']:
                        try:
                            response_data = json.loads(msg['data']['data']) if isinstance(msg['data']['data'], str) else msg['data']['data']
                            current_status = response_data.get('enable', False)
                            self.obstacle_detection_enabled = current_status
                            status_text = "enabled" if current_status else "disabled"
                            self.get_logger().info(f"Obstacle detection status for robot {robot_num}: {status_text}")
                        except Exception as e:
                            self.get_logger().error(f"Error parsing obstacle detection response: {e}")
                            
                elif api_id == 1001:  # SwitchSet response
                    # Check if the command was successful
                    status = msg['data']['header'].get('status', {})
                    if status.get('code') == 0:
                        action = "enabled" if self.obstacle_detection_enabled else "disabled"
                        self.get_logger().info(f"Obstacle detection {action} successfully for robot {robot_num}")
                    else:
                        error_msg = status.get('msg', 'Unknown error')
                        self.get_logger().error(f"Obstacle detection command failed for robot {robot_num}: {error_msg}")
                        
                elif api_id == 1004:  # Remote control response
                    self.get_logger().info(f"Remote control response received for robot {robot_num}")
                        
        except Exception as e:
            self.get_logger().error(f"Error handling obstacle detection response: {e}")

    def handle_multiple_state_message(self, robot_num, msg):
        """Handle multiple state messages for obstacle detection status."""
        try:
            if 'data' in msg:
                data = msg['data']
                # Check if this is a string that needs to be parsed
                if isinstance(data, str):
                    try:
                        data = json.loads(data)
                    except json.JSONDecodeError:
                        return
                
                # Extract obstacle detection status from obstaclesAvoidSwitch
                if 'obstaclesAvoidSwitch' in data:
                    current_status = bool(data['obstaclesAvoidSwitch'])
                    if current_status != self.obstacle_detection_enabled:
                        self.obstacle_detection_enabled = current_status
                        status_text = "enabled" if current_status else "disabled"
                        self.get_logger().info(f"Obstacle detection status updated from MULTIPLE_STATE for robot {robot_num}: {status_text}")
                        # Update the obstacle avoidance parameter
                        self.update_obstacle_avoidance_parameter(robot_num)
                        
        except Exception as e:
            self.get_logger().error(f"Error handling multiple state message: {e}")

    def send_obstacle_aware_move(self, robot_num, x, y, yaw, mode=0):
        """Send obstacle-aware velocity command (api_id=1003)."""
        try:
            command = {
                "type": "msg",
                "topic": RTC_TOPIC["OBSTACLES_AVOID"],
                "data": {
                    "header": {
                        "identity": {
                            "id": int(asyncio.get_event_loop().time() * 1000) % 2147483648,
                            "api_id": 1003
                        }
                    },
                    "parameter": json.dumps({"x": x, "y": y, "yaw": yaw, "mode": mode})
                }
            }
            
            self.conn[robot_num].data_channel.send(json.dumps(command))
            self.get_logger().debug(f"Obstacle-aware move command sent: x={x}, y={y}, yaw={yaw}, mode={mode}")
            
        except Exception as e:
            self.get_logger().error(f"Error sending obstacle-aware move command: {e}")

    def update_obstacle_avoidance_parameter(self, robot_num):
        """Update the obstacle avoidance parameter based on current state."""
        if robot_num in self.conn:
            # Update the parameter that's used in move commands
            old_value = self.obstacle_avoidance
            self.obstacle_avoidance = self.obstacle_detection_enabled
            self.get_logger().info(f"Obstacle avoidance parameter updated: {old_value} -> {self.obstacle_avoidance}")
            self.get_logger().debug(f"Robot {robot_num}: obstacle_detection_enabled={self.obstacle_detection_enabled}, obstacle_avoidance={self.obstacle_avoidance}")

    def get_obstacle_detection_status_bool(self, robot_num):
        """Get current obstacle detection status as a boolean."""
        return self.obstacle_detection_enabled

    def is_obstacle_detection_enabled(self, robot_num):
        """Check if obstacle detection is currently enabled."""
        return self.obstacle_detection_enabled

    def send_movement_command(self, robot_num, x, y, z, use_obstacle_avoidance=None):
        """
        Send a movement command with optional obstacle avoidance.
        
        Args:
            robot_num: Robot number
            x: Forward/backward velocity
            y: Left/right velocity  
            z: Yaw rotation velocity
            use_obstacle_avoidance: If None, uses current obstacle_avoidance setting
                                   If True/False, overrides current setting
        """
        if robot_num not in self.conn:
            self.get_logger().warn(f"Robot {robot_num} not connected")
            return False
            
        try:
            # Determine whether to use obstacle avoidance
            if use_obstacle_avoidance is None:
                use_obstacle_avoidance = self.obstacle_avoidance
            
            # Generate the appropriate movement command
            if use_obstacle_avoidance:
                # Use obstacle-aware movement (api_id=1003)
                command = {
                    "type": "msg",
                    "topic": RTC_TOPIC["OBSTACLES_AVOID"],
                    "data": {
                        "header": {
                            "identity": {
                                "id": int(asyncio.get_event_loop().time() * 1000) % 2147483648,
                                "api_id": 1003
                            }
                        },
                        "parameter": json.dumps({"x": x, "y": y, "yaw": z, "mode": 0})
                    }
                }
                self.get_logger().debug(f"Obstacle-aware move: x={x:.3f}, y={y:.3f}, yaw={z:.3f}")
            else:
                # Use regular sport movement (api_id=1008)
                command = {
                    "type": "msg",
                    "topic": RTC_TOPIC["SPORT_MOD"],
                    "data": {
                        "header": {
                            "identity": {
                                "id": int(asyncio.get_event_loop().time() * 1000) % 2147483648,
                                "api_id": 1008
                            }
                        },
                        "parameter": json.dumps({"x": x, "y": y, "z": z})
                    }
                }
                self.get_logger().debug(f"Sport move: x={x:.3f}, y={y:.3f}, z={z:.3f}")
            
            # Send the command
            self.conn[robot_num].data_channel.send(json.dumps(command))
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error sending movement command: {e}")
            return False

    def stop_movement(self, robot_num, use_obstacle_avoidance=None):
        """
        Stop robot movement with optional obstacle avoidance.
        
        Args:
            robot_num: Robot number
            use_obstacle_avoidance: If None, uses current obstacle_avoidance setting
                                   If True/False, overrides current setting
        """
        if robot_num not in self.conn:
            self.get_logger().warn(f"Robot {robot_num} not connected")
            return False
            
        try:
            # Determine whether to use obstacle avoidance
            if use_obstacle_avoidance is None:
                use_obstacle_avoidance = self.obstacle_avoidance
            
            if use_obstacle_avoidance:
                # Use obstacle-aware stop (api_id=1003 with zero velocities)
                command = {
                    "type": "msg",
                    "topic": RTC_TOPIC["OBSTACLES_AVOID"],
                    "data": {
                        "header": {
                            "identity": {
                                "id": int(asyncio.get_event_loop().time() * 1000) % 2147483648,
                                "api_id": 1003
                            }
                        },
                        "parameter": json.dumps({"x": 0.0, "y": 0.0, "yaw": 0.0, "mode": 0})
                    }
                }
                self.get_logger().debug("Obstacle-aware stop command sent")
            else:
                # Use regular stop command (api_id=1003)
                command = {
                    "type": "msg",
                    "topic": RTC_TOPIC["SPORT_MOD"],
                    "data": {
                        "header": {
                            "identity": {
                                "id": int(asyncio.get_event_loop().time() * 1000) % 2147483648,
                                "api_id": 1003  # StopMove
                            }
                        },
                        "parameter": json.dumps({})
                    }
                }
                self.get_logger().debug("Sport stop command sent")
            
            # Send the command
            self.conn[robot_num].data_channel.send(json.dumps(command))
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error sending stop command: {e}")
            return False

    def test_movement_modes(self, robot_num):
        """
        Test both movement modes to verify they work correctly.
        This is useful for debugging movement issues.
        """
        if robot_num not in self.conn:
            self.get_logger().warn(f"Robot {robot_num} not connected")
            return False
            
        self.get_logger().info(f"Testing movement modes for robot {robot_num}")
        
        # Test regular sport movement
        self.get_logger().info("Testing regular sport movement...")
        success1 = self.send_movement_command(robot_num, 0.1, 0.0, 0.0, use_obstacle_avoidance=False)
        
        # Wait a moment
        import time
        time.sleep(0.5)
        
        # Test obstacle-aware movement
        self.get_logger().info("Testing obstacle-aware movement...")
        self.get_logger().info(f"Current obstacle_avoidance parameter: {self.obstacle_avoidance}")
        success2 = self.send_movement_command(robot_num, 0.1, 0.0, 0.0, use_obstacle_avoidance=True)
        
        # Wait a moment
        time.sleep(0.5)
        
        # Stop movement
        self.get_logger().info("Stopping movement...")
        success3 = self.stop_movement(robot_num)
        
        if success1 and success2 and success3:
            self.get_logger().info("All movement mode tests passed!")
            return True
        else:
            self.get_logger().warn("Some movement mode tests failed!")
            return False

    def stop_all_movement(self, robot_num):
        """
        Stop all robot movement regardless of current mode.
        This ensures clean stopping before mode switching.
        """
        if robot_num not in self.conn:
            self.get_logger().warn(f"Robot {robot_num} not connected")
            return False
            
        try:
            self.get_logger().info(f"Stopping all movement for robot {robot_num}")
            
            # Stop obstacle avoidance movement first (if it was enabled)
            if self.obstacle_detection_enabled:
                self.get_logger().debug("Stopping obstacle avoidance movement")
                obstacle_stop_command = {
                    "type": "msg",
                    "topic": RTC_TOPIC["OBSTACLES_AVOID"],
                    "data": {
                        "header": {
                            "identity": {
                                "id": int(asyncio.get_event_loop().time() * 1000) % 2147483648,
                                "api_id": 1003
                            }
                        },
                        "parameter": json.dumps({"x": 0.0, "y": 0.0, "yaw": 0.0, "mode": 0})
                    }
                }
                self.conn[robot_num].data_channel.send(json.dumps(obstacle_stop_command))
            
            # Also send regular stop command to ensure complete stopping
            self.get_logger().debug("Sending regular stop command")
            regular_stop_command = {
                "type": "msg",
                "topic": RTC_TOPIC["SPORT_MOD"],
                "data": {
                    "header": {
                        "identity": {
                            "id": int(asyncio.get_event_loop().time() * 1000) % 2147483648,
                            "api_id": 1003  # StopMove
                        }
                    },
                    "parameter": json.dumps({})
                }
            }
            self.conn[robot_num].data_channel.send(json.dumps(regular_stop_command))
            
            self.get_logger().info(f"All movement stopped for robot {robot_num}")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error stopping all movement: {e}")
            return False

    def force_stop_obstacle_avoidance(self, robot_num):
        """
        Force stop obstacle avoidance movement when the robot is stuck in continuous movement.
        This is useful when obstacle avoidance is enabled but there are no obstacles.
        """
        if robot_num not in self.conn:
            self.get_logger().warn(f"Robot {robot_num} not connected")
            return False
            
        try:
            self.get_logger().info(f"Force stopping obstacle avoidance movement for robot {robot_num}")
            
            # Send multiple stop commands to ensure the robot stops
            for i in range(3):  # Send 3 stop commands
                obstacle_stop_command = {
                    "type": "msg",
                    "topic": RTC_TOPIC["OBSTACLES_AVOID"],
                    "data": {
                        "header": {
                            "identity": {
                                "id": int(asyncio.get_event_loop().time() * 1000) % 2147483648,
                                "api_id": 1003
                            }
                        },
                        "parameter": json.dumps({"x": 0.0, "y": 0.0, "yaw": 0.0, "mode": 0})
                    }
                }
                self.conn[robot_num].data_channel.send(json.dumps(obstacle_stop_command))
                
                # Brief wait between commands
                import time
                time.sleep(0.1)
            
            self.get_logger().info(f"Force stop commands sent for robot {robot_num}")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error force stopping obstacle avoidance: {e}")
            return False

    def emergency_stop(self, robot_num):
        """
        Emergency stop all robot movement immediately.
        This sends multiple stop commands to ensure the robot stops.
        """
        if robot_num not in self.conn:
            self.get_logger().warn(f"Robot {robot_num} not connected")
            return False
            
        try:
            self.get_logger().warn(f"EMERGENCY STOP for robot {robot_num}")
            
            # Send multiple stop commands to both topics
            for i in range(5):  # Send 5 stop commands
                # Stop obstacle avoidance
                obstacle_stop = {
                    "type": "msg",
                    "topic": RTC_TOPIC["OBSTACLES_AVOID"],
                    "data": {
                        "header": {
                            "identity": {
                                "id": int(asyncio.get_event_loop().time() * 1000) % 2147483648,
                                "api_id": 1003
                            }
                        },
                        "parameter": json.dumps({"x": 0.0, "y": 0.0, "yaw": 0.0, "mode": 0})
                    }
                }
                self.conn[robot_num].data_channel.send(json.dumps(obstacle_stop))
                
                # Stop regular movement
                regular_stop = {
                    "type": "msg",
                    "topic": RTC_TOPIC["SPORT_MOD"],
                    "data": {
                        "header": {
                            "identity": {
                                "id": int(asyncio.get_event_loop().time() * 1000) % 2147483648,
                                "api_id": 1003  # StopMove
                            }
                        },
                        "parameter": json.dumps({})
                    }
                }
                self.conn[robot_num].data_channel.send(json.dumps(regular_stop))
                
                # Brief wait between commands
                import time
                time.sleep(0.05)  # Faster emergency stop
            
            self.get_logger().warn(f"Emergency stop commands sent for robot {robot_num}")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error during emergency stop: {e}")
            return False

    def set_obstacle_detection(self, robot_num, enable):
        """Set obstacle detection to a specific state (enable/disable)."""
        if robot_num not in self.conn:
            self.get_logger().warn(f"Robot {robot_num} not connected")
            return
            
        try:
            # Only change if the state is different
            if self.obstacle_detection_enabled != enable:
                # First, stop any current movement to prevent double movement
                self.get_logger().info(f"Stopping current movement before mode switch for robot {robot_num}")
                self.stop_all_movement(robot_num)
                
                # Wait a moment for the stop command to take effect
                import time
                time.sleep(0.2)
                
                self.obstacle_detection_enabled = enable
                action = "enabling" if enable else "disabling"
                self.get_logger().info(f"{action.capitalize()} obstacle detection for robot {robot_num}")
                
                # Send the command to the robot
                if enable:
                    self.enable_obstacle_detection(robot_num)
                else:
                    self.disable_obstacle_detection(robot_num)
                
                # Update the obstacle avoidance parameter
                self.update_obstacle_avoidance_parameter(robot_num)
            else:
                status_text = "enabled" if enable else "disabled"
                self.get_logger().info(f"Obstacle detection already {status_text} for robot {robot_num}")
                
        except Exception as e:
            self.get_logger().error(f"Error setting obstacle detection: {e}")


async def run(conn, robot_num, node):
    """
    Standalone run function to handle robot connection.

    Args:
        conn: The robot connection object
        robot_num: The robot number as a string
        node: The RobotBaseNode instance
    """
    node.conn[robot_num] = conn
    if node.conn_type == 'webrtc':
        try:
            await node.conn[robot_num].connect()
            # await
            # node.conn[robot_num].data_channel.disableTrafficSaving(True)
        except Exception as e:
            node.get_logger().error(
                f"Failed to connect to robot {robot_num} - exiting: {e}")
            # Signal that a critical error occurred by raising an exception
            raise RuntimeError(f"Failed to connect to robot {robot_num}") from e

    try:
        while True:
            if node.conn_type == 'webrtc':
                node.joy_cmd(robot_num)
                node.publish_webrtc_commands(robot_num)
            await asyncio.sleep(0.1)
    except Exception as e:
        node.get_logger().error(
            f"Error in run loop for robot {robot_num}: {e}")
        # Raise the exception to signal task failure
        raise


async def spin(node: Node):
    """Spin the node in a separate thread with proper context management."""
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Create a future that will be completed when we want to stop spinning
    event_loop = asyncio.get_event_loop()
    stop_future = event_loop.create_future()

    try:
        # Wait until the future is completed or cancelled
        await stop_future
    except asyncio.CancelledError:
        # Handle cancellation
        pass
    finally:
        # Shutdown the executor and join the thread
        executor.shutdown()
        spin_thread.join(timeout=1.0)  # Add timeout to avoid hanging


async def start_node():
    # Create the node
    base_node = RobotBaseNode()

    # Setup node spinning in a separate thread
    spin_task = asyncio.create_task(spin(base_node))

    # Track all robot connection tasks
    robot_tasks = []

    # Function to handle errors in any task
    def handle_error(e, task_name="unknown"):
        base_node.get_logger().error(f"Error in {task_name}: {e}")
        # Cancel the spin task to initiate shutdown
        if not spin_task.done():
            spin_task.cancel()
        # Cancel all robot tasks
        for task in robot_tasks:
            if not task.done():
                task.cancel()

    # Start connections to robots
    try:
        for i in range(len(base_node.robot_ip_lst)):
            conn = Go2Connection(
                robot_ip=base_node.robot_ip_lst[i],
                robot_num=str(i),
                token=base_node.token,
                on_validated=base_node.on_validated,
                on_message=base_node.on_data_channel_message,
                on_video_frame=base_node.on_video_frame if base_node.enable_video else None,
                decode_lidar=base_node.decode_lidar,
            )

            # Start the robot connection and add to our list
            # Use the standalone run function instead of a method on the node
            run_task = asyncio.create_task(run(conn, str(i), base_node))
            robot_tasks.append(run_task)

            # Define a unique callback for each task that can reference the
            # robot number
            robot_num = str(i)

            def create_callback(robot_id):
                def callback(task):
                    try:
                        task.result()  # Will raise exception if one occurred
                    except asyncio.CancelledError:
                        # Normal during shutdown
                        pass
                    except Exception as e:
                        handle_error(e, f"robot {robot_id}")
                return callback

            # Add the callback to the task
            run_task.add_done_callback(create_callback(robot_num))

        # Wait for any task to complete or fail
        done, pending = await asyncio.wait(
            [spin_task] + robot_tasks,
            return_when=asyncio.FIRST_COMPLETED
        )

        # Check if any task completed with an exception
        for task in done:
            try:
                task.result()
            except asyncio.CancelledError:
                # Normal during shutdown
                pass
            except Exception as e:
                handle_error(e, "completed task")

    except Exception as e:
        handle_error(e, "setup phase")
    finally:
        # Ensure clean shutdown
        if not spin_task.done():
            spin_task.cancel()

        for task in robot_tasks:
            if not task.done():
                task.cancel()

        # Wait for all tasks to finish
        try:
            # Use a timeout to avoid hanging indefinitely
            await asyncio.wait([spin_task] + robot_tasks, timeout=2.0)
        except Exception:
            # Ignore any errors during shutdown
            pass


def main():
    """Main entry point with proper initialization and cleanup."""
    # Initialize ROS
    rclpy.init()

    try:
        # Get the event loop
        loop = asyncio.get_event_loop()

        # Run the main coroutine
        loop.run_until_complete(start_node())
    except KeyboardInterrupt:
        print("Node terminated by keyboard interrupt")
    except Exception as e:
        print(f"Fatal error in node execution: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Clean shutdown
        try:
            # Close remaining tasks
            pending = asyncio.all_tasks(loop)
            if pending:
                print(f"Cancelling {len(pending)} pending tasks...")
                for task in pending:
                    task.cancel()
                # Wait briefly for tasks to acknowledge cancellation
                loop.run_until_complete(asyncio.gather(
                    *pending, return_exceptions=True))
        except Exception as e:
            print(f"Error during cleanup: {e}")

        # Finally, close the loop and shutdown ROS
        loop.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
