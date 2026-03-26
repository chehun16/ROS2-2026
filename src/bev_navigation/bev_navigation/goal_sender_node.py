"""
Node 3: Goal Sender Node

역할:
    BEV Occupancy Map을 모니터링하다가
    Nav2 Action Server(/navigate_to_pose)에 목표 지점을 전송하고
    이동 상태를 추적.
    키보드 또는 파라미터로 목표 좌표를 설정 가능.

Subscriptions:
    /bev_map/occupancy           (nav_msgs/OccupancyGrid) - 현재 점유 맵 모니터링

Action Clients:
    /navigate_to_pose            (nav2_msgs/action/NavigateToPose)

Publications:
    /navigation_status           (std_msgs/String) - 현재 이동 상태 문자열
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose


class GoalSenderNode(Node):

    def __init__(self):
        super().__init__('goal_sender_node')

        # 목표 좌표 파라미터 (Nav2 map 프레임 기준, 미터)
        self.declare_parameter('goal_x',    3.0)
        self.declare_parameter('goal_y',    0.0)
        self.declare_parameter('goal_yaw',  0.0)   # 도착 방향 (라디안)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('auto_send', True)   # True: 시작 시 자동 목표 전송

        self.goal_x    = self.get_parameter('goal_x').value
        self.goal_y    = self.get_parameter('goal_y').value
        self.goal_yaw  = self.get_parameter('goal_yaw').value
        self.map_frame = self.get_parameter('map_frame').value
        self.auto_send = self.get_parameter('auto_send').value

        self._navigating   = False
        self._goal_handle  = None

        # Nav2 Action Client
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # BEV 맵 구독 (장애물 밀도 모니터링)
        self.sub_occ = self.create_subscription(
            OccupancyGrid,
            '/bev_map/occupancy',
            self._occ_callback,
            10,
        )

        # 상태 발행
        self.pub_status = self.create_publisher(String, '/navigation_status', 10)

        # Nav2 서버 대기 후 자동 목표 전송
        if self.auto_send:
            self.get_logger().info('Waiting for Nav2 action server...')
            self._action_client.wait_for_server()
            self.get_logger().info('Nav2 ready. Sending initial goal.')
            self.send_goal(self.goal_x, self.goal_y, self.goal_yaw)

        self.get_logger().info('GoalSenderNode started.')

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def send_goal(self, x: float, y: float, yaw: float = 0.0):
        """Nav2에 목표 지점 전송."""
        if self._navigating:
            self.get_logger().warn('Already navigating. Cancel first.')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._make_pose(x, y, yaw)

        self.get_logger().info(f'Sending goal → x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}')
        self._publish_status(f'NAVIGATING to ({x:.2f}, {y:.2f})')

        send_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback,
        )
        send_future.add_done_callback(self._goal_response_callback)
        self._navigating = True

    def cancel_goal(self):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._navigating = False
            self._publish_status('CANCELED')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _occ_callback(self, msg: OccupancyGrid):
        """
        BEV 맵 수신 시 장애물 밀도 계산 후 로깅.
        추후 확장: 맵 분석 → 동적 목표 재설정 로직 추가 가능.
        """
        data = list(msg.data)
        total       = len(data)
        occupied    = data.count(100)
        free        = data.count(0)
        unknown     = data.count(-1)
        occ_ratio   = occupied / total * 100 if total > 0 else 0.0
        self.get_logger().debug(
            f'BEV map: {total} cells | occupied={occupied}({occ_ratio:.1f}%) '
            f'free={free} unknown={unknown}'
        )

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by Nav2.')
            self._navigating = False
            self._publish_status('REJECTED')
            return

        self._goal_handle = goal_handle
        self.get_logger().info('Goal accepted by Nav2.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        remaining = feedback.distance_remaining
        self.get_logger().debug(f'Distance remaining: {remaining:.2f} m')

    def _result_callback(self, future):
        result = future.result()
        status = result.status

        # action_msgs/msg/GoalStatus 실제 코드:
        # 4=SUCCEEDED, 5=CANCELED, 6=ABORTED
        STATUS_MAP = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}
        status_str = STATUS_MAP.get(status, f'UNKNOWN({status})')

        self.get_logger().info(f'Navigation result: {status_str}')
        self._publish_status(status_str)
        self._navigating  = False
        self._goal_handle = None

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _make_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        # yaw → quaternion (z축 회전)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.pub_status.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GoalSenderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
