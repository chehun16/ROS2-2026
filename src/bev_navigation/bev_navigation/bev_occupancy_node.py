"""
Node 2: BEV Occupancy Map Node  [프로젝트 핵심 노드]

역할:
    Depth 이미지를 받아 카메라 핀홀 모델로 3D 포인트클라우드를 생성하고,
    로봇 기준 Bird's Eye View(BEV) Occupancy Grid로 변환하여 발행.
    Nav2가 이 맵을 수신하여 경로 계획에 활용.

Subscriptions:
    /processed/depth             (sensor_msgs/Image)       - 전처리된 Depth
    /processed/camera_info       (sensor_msgs/CameraInfo)  - 카메라 내부 파라미터

Publications:
    /bev_map/occupancy           (nav_msgs/OccupancyGrid)  - BEV 점유 격자 지도
    /bev_map/pointcloud          (sensor_msgs/PointCloud2) - 시각화용 포인트클라우드
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import message_filters
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

from cv_bridge import CvBridge
import numpy as np

from bev_navigation.utils.bev_utils import (
    depth_to_pointcloud,
    camera_to_world,
    pointcloud_to_bev_grid,
)


class BEVOccupancyNode(Node):

    def __init__(self):
        super().__init__('bev_occupancy_node')

        # 파라미터 선언
        self.declare_parameter('camera_height_m',    0.3)    # 카메라 지면 높이 (m)
        self.declare_parameter('camera_pitch_deg',  15.0)    # 카메라 피치각 (도)
        self.declare_parameter('grid_resolution',    0.05)   # 셀 크기 (m)
        self.declare_parameter('grid_size_m',       10.0)    # 맵 전체 크기 (m)
        self.declare_parameter('obstacle_min_h',     0.05)   # 장애물 최소 높이
        self.declare_parameter('obstacle_max_h',     2.0)    # 장애물 최대 높이
        self.declare_parameter('map_frame',         'map')
        self.declare_parameter('robot_frame',       'base_link')

        self.camera_height  = self.get_parameter('camera_height_m').value
        pitch_deg           = self.get_parameter('camera_pitch_deg').value
        self.camera_pitch   = np.deg2rad(pitch_deg)
        self.grid_res       = self.get_parameter('grid_resolution').value
        self.grid_size      = self.get_parameter('grid_size_m').value
        self.obs_min_h      = self.get_parameter('obstacle_min_h').value
        self.obs_max_h      = self.get_parameter('obstacle_max_h').value
        self.map_frame      = self.get_parameter('map_frame').value
        self.robot_frame    = self.get_parameter('robot_frame').value

        self.grid_cells = int(self.grid_size / self.grid_res)
        self.bridge = CvBridge()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Subscribers (시간 동기화)
        self.sub_depth = message_filters.Subscriber(self, Image,      '/processed/depth',       qos_profile=sensor_qos)
        self.sub_info  = message_filters.Subscriber(self, CameraInfo, '/processed/camera_info', qos_profile=sensor_qos)

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.sub_depth, self.sub_info],
            queue_size=10,
            slop=0.1,
        )
        self.sync.registerCallback(self.callback)

        # Publishers
        self.pub_occ = self.create_publisher(OccupancyGrid, '/bev_map/occupancy',  10)
        self.pub_pc  = self.create_publisher(PointCloud2,   '/bev_map/pointcloud', 10)

        self.get_logger().info(
            f'BEVOccupancyNode started. '
            f'Grid: {self.grid_cells}x{self.grid_cells} cells @ {self.grid_res}m/cell'
        )

    def callback(self, depth_msg: Image, info_msg: CameraInfo):
        # 1. Depth 이미지 → numpy
        depth_cv = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        depth_np = np.array(depth_cv, dtype=np.float32)

        # 2. 카메라 내부 파라미터 추출
        camera_info = {
            'fx': info_msg.k[0],
            'fy': info_msg.k[4],
            'cx': info_msg.k[2],
            'cy': info_msg.k[5],
        }

        # 3. Depth → 3D 포인트클라우드 (카메라 좌표계)
        points_cam = depth_to_pointcloud(depth_np, camera_info)
        if len(points_cam) == 0:
            return

        # 4. 카메라 좌표 → 월드 좌표 (로봇 기준)
        points_world = camera_to_world(points_cam, self.camera_height, self.camera_pitch)

        # 5. 3D → BEV Occupancy Grid
        grid = pointcloud_to_bev_grid(
            points_world,
            self.grid_res,
            self.grid_size,
            self.obs_min_h,
            self.obs_max_h,
        )

        # 6. OccupancyGrid 메시지 발행
        occ_msg = self._build_occupancy_grid(grid, depth_msg.header)
        self.pub_occ.publish(occ_msg)

        # 7. PointCloud2 메시지 발행 (RViz2 시각화용)
        pc_msg = self._build_pointcloud2(points_world, depth_msg.header)
        self.pub_pc.publish(pc_msg)

    def _build_occupancy_grid(self, grid: np.ndarray, header: Header) -> OccupancyGrid:
        msg = OccupancyGrid()
        msg.header.stamp    = header.stamp
        msg.header.frame_id = self.robot_frame  # 로봇 기준 local map

        msg.info.resolution       = self.grid_res
        msg.info.width            = self.grid_cells
        msg.info.height           = self.grid_cells
        msg.info.origin.position.x = -self.grid_size / 2.0
        msg.info.origin.position.y = -self.grid_size / 2.0
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        # (row, col) → 1D list (row-major, x:col, y:row)
        msg.data = grid.flatten().tolist()
        return msg

    def _build_pointcloud2(self, points: np.ndarray, header: Header) -> PointCloud2:
        msg = PointCloud2()
        msg.header.stamp    = header.stamp
        msg.header.frame_id = self.robot_frame

        msg.height = 1
        msg.width  = len(points)
        msg.fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step   = 12  # 3 * 4 bytes
        msg.row_step     = msg.point_step * msg.width
        msg.is_dense     = True

        # numpy 벡터 연산으로 직렬화 (for-loop 대비 ~100배 빠름)
        msg.data = points.astype(np.float32).tobytes()

        return msg


def main(args=None):
    rclpy.init(args=args)
    node = BEVOccupancyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
