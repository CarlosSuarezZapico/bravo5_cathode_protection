#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Empty


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def yaw_from_quat(q):
    """
    Extract yaw from quaternion (x,y,z,w) using standard yaw formula.
    """
    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quat_from_yaw(yaw):
    """
    Build quaternion for yaw-only rotation.
    """
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


def angle_diff(a, b):
    """
    Smallest signed difference a - b, wrapped to [-pi, pi].
    """
    d = a - b
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return d


class JoyToPidPoseNode(Node):
    def __init__(self):
        super().__init__('teleop_rov_pid_pose')

        # -------------------- Tunables --------------------
        self.max_linear_vel = 0.5   # m/s
        self.max_angular_vel = 1.0  # rad/s (yaw rate)

        # Axis mapping (adjust to your controller)
        # Your original mapping:
        # axes[0] -> x, axes[1] -> y, axes[2] -> z, axes[3] -> yaw rate
        self.axis_x = 0
        self.axis_y = 1
        self.axis_z = 2
        self.axis_yaw = 3

        # Deadman / enable button (your original: buttons[0])
        self.enable_button = 3

        # Publish rate for setpoint (and integration)
        self.pub_hz = 50.0

        # Error thresholds for integrator reset
        self.err_thresh_x = 0.7    # meters
        self.err_thresh_y = 0.7    # meters
        self.err_thresh_z = 0.7    # meters
        self.err_thresh_yaw = 0.8  # radians (~46 deg)

        # --------------------------------------------------

        # State
        self.current_pose = None      # geometry_msgs/Pose
        self.desired_pose = None      # geometry_msgs/Pose
        self.have_desired = False

        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_vz = 0.0
        self.cmd_wz = 0.0
        self.enabled = False

        self.last_time = self.get_clock().now()

        # Subscribers
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/auip/vehicle_interface/odometry',
            self.odom_callback,
            10
        )

        # Publishers
        # This is the setpoint your PID position controller consumes:
        self.pid_pub = self.create_publisher(PoseStamped, '/auip/pid/request', 10)

        # Optional: publish the “raw” Twist we’re integrating (for debug)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Integrator reset trigger (topic name may differ in your stack)
        self.reset_pub = self.create_publisher(Empty, '/auip/pid/reset_integrator', 10)

        # Timer loop
        self.timer = self.create_timer(1.0 / self.pub_hz, self.control_loop)

        self.get_logger().info('Teleop → PID Pose node started.')

    # -------------------- Callbacks --------------------

    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose

        # Initialize desired pose to current pose once we have odom
        if not self.have_desired:
            self.desired_pose = msg.pose.pose
            self.have_desired = True

    def joy_callback(self, msg: Joy):
        # Deadman switch
        self.enabled = bool(msg.buttons[self.enable_button]) if len(msg.buttons) > self.enable_button else False

        if self.enabled:
            ax = msg.axes[self.axis_x] if len(msg.axes) > self.axis_x else 0.0
            ay = msg.axes[self.axis_y] if len(msg.axes) > self.axis_y else 0.0
            az = msg.axes[self.axis_z] if len(msg.axes) > self.axis_z else 0.0
            ayaw = msg.axes[self.axis_yaw] if len(msg.axes) > self.axis_yaw else 0.0

            # Joy axes should already be [-1,1], but clamp defensively
            ax = clamp(float(ax), -1.0, 1.0)
            ay = clamp(float(ay), -1.0, 1.0)
            az = clamp(float(az), 0.0, 0.0)
            ayaw = clamp(float(ayaw), 0.0, 0.0)

            self.cmd_vx = ax * self.max_linear_vel
            self.cmd_vy = ay * self.max_linear_vel
            self.cmd_vz = az * self.max_linear_vel
            self.cmd_wz = ayaw * self.max_angular_vel
        else:
            self.cmd_vx = 0.0
            self.cmd_vy = 0.0
            self.cmd_vz = 0.0
            self.cmd_wz = 0.0

    # -------------------- Logic --------------------

    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0:
            return

        # Need odom first
        if self.current_pose is None or not self.have_desired:
            return

        # Optional debug Twist publication (the command we integrate)
        dbg = Twist()
        dbg.linear.x = self.cmd_vx
        dbg.linear.y = self.cmd_vy
        dbg.linear.z = self.cmd_vz
        dbg.angular.z = self.cmd_wz
        self.cmd_pub.publish(dbg)

        # If not enabled, hold position: do NOT integrate
        if not self.enabled:
            # still publish the held desired pose to PID
            self.publish_desired_pose(now)
            return

        # ---- Integrate joystick “twist” into a pose setpoint ----
        self.desired_pose.position.x += self.cmd_vx * dt
        self.desired_pose.position.y += self.cmd_vy * dt
        self.desired_pose.position.z += self.cmd_vz * dt

        cur_yaw = yaw_from_quat(self.desired_pose.orientation)
        new_yaw = cur_yaw + (self.cmd_wz * dt)

        qx, qy, qz, qw = quat_from_yaw(new_yaw)
        self.desired_pose.orientation.x = qx
        self.desired_pose.orientation.y = qy
        self.desired_pose.orientation.z = qz
        self.desired_pose.orientation.w = qw

        # ---- Check pose error; reset integrator if too large ----
        if self.pose_error_exceeds_threshold():
            self.reset_integrator_and_reanchor()

        # Publish setpoint to PID controller
        self.publish_desired_pose(now)

    def pose_error_exceeds_threshold(self) -> bool:
        cp = self.current_pose
        dp = self.desired_pose

        ex = abs(dp.position.x - cp.position.x)
        ey = abs(dp.position.y - cp.position.y)
        ez = abs(dp.position.z - cp.position.z)

        cyaw = yaw_from_quat(cp.orientation)
        dyaw = yaw_from_quat(dp.orientation)
        eyaw = abs(angle_diff(dyaw, cyaw))

        return (ex > self.err_thresh_x or
                ey > self.err_thresh_y or
                ez > self.err_thresh_z or
                eyaw > self.err_thresh_yaw)

    def reset_integrator_and_reanchor(self):
        # 1) Tell PID controller to reset its integral term (topic/interface may vary)
        self.reset_pub.publish(Empty())

        # 2) Re-anchor desired pose to current pose to avoid a giant error step
        self.desired_pose = self.current_pose

        self.get_logger().warn(
            'Pose error exceeded threshold; reset integrator and re-anchored setpoint to current pose.'
        )

    def publish_desired_pose(self, now):
        msg = PoseStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'auip_odom'  # set this to whatever your PID expects
        msg.pose = self.desired_pose
        self.pid_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JoyToPidPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
