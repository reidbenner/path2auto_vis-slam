"""
Instruction Node — converts a Nav2 path to turn-by-turn operator instructions.

Subscribes: /hmi/goal_pose  (geometry_msgs/PoseStamped)
Publishes:  /hmi/instruction (std_msgs/String, JSON)

JSON format:
  {
    "action":           "forward" | "turn_left" | "turn_right" | "arrived",
    "distance_to_next": 2.3,        # metres to next waypoint/turn
    "distance_to_goal": 8.1,        # metres remaining to goal
    "heading":          "north"     # cardinal / intercardinal
  }
"""
import json
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav2_msgs.action import ComputePathToPose
import tf2_ros
import tf2_geometry_msgs  # noqa: F401 — registers transforms


TURN_THRESHOLD_DEG = 20.0   # heading change that counts as a turn
ARRIVAL_RADIUS_M   = 0.3    # metres — considered arrived when within this distance


def _yaw_from_pose(pose):
    q = pose.orientation
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _dist(a, b):
    return math.hypot(a.position.x - b.position.x, a.position.y - b.position.y)


def _heading_label(yaw_rad):
    deg = math.degrees(yaw_rad) % 360
    labels = ['east', 'north-east', 'north', 'north-west',
              'west', 'south-west', 'south', 'south-east']
    idx = int((deg + 22.5) / 45.0) % 8
    return labels[idx]


def _path_to_waypoints(poses):
    """Reduce full path to a list of (pose, action) waypoints at heading changes."""
    if len(poses) < 2:
        return []

    waypoints = []
    prev_yaw = _yaw_from_pose(poses[0].pose)

    for i in range(1, len(poses)):
        cur_yaw = _yaw_from_pose(poses[i].pose)
        delta = math.degrees(cur_yaw - prev_yaw)
        # Normalise to [-180, 180]
        delta = (delta + 180) % 360 - 180

        if abs(delta) >= TURN_THRESHOLD_DEG:
            action = 'turn_left' if delta > 0 else 'turn_right'
            waypoints.append((poses[i].pose, action))
            prev_yaw = cur_yaw

    waypoints.append((poses[-1].pose, 'arrived'))
    return waypoints


class InstructionNode(Node):
    def __init__(self):
        super().__init__('instruction_node')

        self._action_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._pub = self.create_publisher(String, '/hmi/instruction', 10)
        self._sub = self.create_subscription(
            PoseStamped, '/hmi/goal_pose', self._goal_cb, 10
        )

        self._waypoints = []   # list of (pose, action_string)
        self._goal_pose = None

        # Poll position and update instruction at 2 Hz
        self.create_timer(0.5, self._update)
        self.get_logger().info('Instruction node ready — publish a goal to /hmi/goal_pose')

    # ------------------------------------------------------------------
    def _goal_cb(self, msg: PoseStamped):
        self._goal_pose = msg
        self._waypoints = []
        self.get_logger().info(
            f'New goal received: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})'
        )
        if not self._action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('ComputePathToPose action server not available')
            return

        goal = ComputePathToPose.Goal()
        goal.goal = msg
        goal.planner_id = 'GridBased'
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Path planning goal rejected')
            return
        handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result().result
        path = result.path
        if not path.poses:
            self.get_logger().warn('Planner returned empty path')
            return
        self._waypoints = _path_to_waypoints(path.poses)
        self.get_logger().info(f'Path computed: {len(self._waypoints)} waypoints')

    # ------------------------------------------------------------------
    def _robot_pose(self):
        try:
            tf = self._tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            pose = tf.transform.translation
            return pose
        except Exception:
            return None

    def _update(self):
        if not self._waypoints or not self._goal_pose:
            return

        robot = self._robot_pose()
        if robot is None:
            return

        # Advance past waypoints the robot has already passed
        while len(self._waypoints) > 1:
            wp_pose, _ = self._waypoints[0]
            dist = math.hypot(robot.x - wp_pose.position.x, robot.y - wp_pose.position.y)
            if dist < ARRIVAL_RADIUS_M:
                self._waypoints.pop(0)
            else:
                break

        next_pose, action = self._waypoints[0]
        dist_to_next = math.hypot(
            robot.x - next_pose.position.x,
            robot.y - next_pose.position.y,
        )
        dist_to_goal = math.hypot(
            robot.x - self._goal_pose.pose.position.x,
            robot.y - self._goal_pose.pose.position.y,
        )

        if action == 'arrived' and dist_to_goal < ARRIVAL_RADIUS_M:
            self._publish('arrived', 0.0, 0.0, '')
            self._waypoints = []
            self._goal_pose = None
            return

        # Determine forward/turn based on current action
        if action == 'arrived':
            display_action = 'forward'
        else:
            display_action = action

        yaw = math.atan2(
            next_pose.position.y - robot.y,
            next_pose.position.x - robot.x,
        )
        self._publish(display_action, dist_to_next, dist_to_goal, _heading_label(yaw))

    def _publish(self, action, dist_to_next, dist_to_goal, heading):
        msg = String()
        msg.data = json.dumps({
            'action':           action,
            'distance_to_next': round(dist_to_next, 2),
            'distance_to_goal': round(dist_to_goal, 2),
            'heading':          heading,
        })
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = InstructionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
