"""
Node 1: RGBD Processor Node

역할:
    Gazebo 시뮬레이터에서 발행하는 RGB 이미지와 Depth 이미지를
    시간 동기화(TimeSynchronizer)하여 후속 노드에 전달.
    필요시 전처리(노이즈 제거, 스케일 변환)도 수행.

Subscriptions:
    /camera/color/image_raw      (sensor_msgs/Image)
    /camera/depth/image_raw      (sensor_msgs/Image)
    /camera/color/camera_info    (sensor_msgs/CameraInfo)

Publications:
    /processed/rgb               (sensor_msgs/Image)
    /processed/depth             (sensor_msgs/Image)
    /processed/camera_info       (sensor_msgs/CameraInfo)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import message_filters
from sensor_msgs.msg import Image, CameraInfo

from cv_bridge import CvBridge
import numpy as np


class RGBDProcessorNode(Node):

    def __init__(self):
        super().__init__('rgbd_processor_node')

        # 파라미터 선언
        self.declare_parameter('depth_min_m', 0.1)   # 유효 depth 최솟값 (m)
        self.declare_parameter('depth_max_m', 10.0)  # 유효 depth 최댓값 (m)
        self.declare_parameter('queue_size',  10)

        self.depth_min = self.get_parameter('depth_min_m').value
        self.depth_max = self.get_parameter('depth_max_m').value
        queue_size     = self.get_parameter('queue_size').value

        self.bridge = CvBridge()

        # QoS: 시뮬레이터 이미지 토픽과 호환
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=queue_size,
        )

        # Subscribers (시간 동기화)
        self.sub_rgb   = message_filters.Subscriber(self, Image,      '/camera/color/image_raw',   qos_profile=sensor_qos)
        self.sub_depth = message_filters.Subscriber(self, Image,      '/camera/depth/image_raw',   qos_profile=sensor_qos)
        self.sub_info  = message_filters.Subscriber(self, CameraInfo, '/camera/color/camera_info', qos_profile=sensor_qos)

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.sub_rgb, self.sub_depth, self.sub_info],
            queue_size=queue_size,
            slop=0.1,
        )
        self.sync.registerCallback(self.callback)

        # Publishers
        self.pub_rgb   = self.create_publisher(Image,      '/processed/rgb',         10)
        self.pub_depth = self.create_publisher(Image,      '/processed/depth',       10)
        self.pub_info  = self.create_publisher(CameraInfo, '/processed/camera_info', 10)

        self.get_logger().info('RGBDProcessorNode started.')

    def callback(self, rgb_msg: Image, depth_msg: Image, info_msg: CameraInfo):
        # Depth 이미지 전처리: 유효 범위 클리핑
        depth_cv = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        depth_np = np.array(depth_cv, dtype=np.float32)

        # 유효하지 않은 값(NaN, inf, 범위 초과) → 0으로 마스킹
        depth_np = np.where(
            np.isfinite(depth_np) & (depth_np >= self.depth_min) & (depth_np <= self.depth_max),
            depth_np,
            0.0,
        )

        # float32 Depth 이미지를 ROS 메시지로 재변환
        processed_depth_msg = self.bridge.cv2_to_imgmsg(depth_np, encoding='32FC1')
        processed_depth_msg.header = depth_msg.header

        # 동기화된 타임스탬프로 헤더 통일
        info_msg.header = rgb_msg.header

        self.pub_rgb.publish(rgb_msg)
        self.pub_depth.publish(processed_depth_msg)
        self.pub_info.publish(info_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RGBDProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
