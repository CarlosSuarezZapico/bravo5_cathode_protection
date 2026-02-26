#!/usr/bin/env python3
"""
Live Bravo5 URDF + SVD ellipsoid visualizer.

Subscribes to JointState, updates the robot configuration in Meshcat, and draws
the end-effector motion/force ellipsoids from the translational Jacobian SVD.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Dict

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import meshcat.geometry as g
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer

ELLIPSOID_FRAME = "contact_point"


def default_urdf_path() -> Path:
    return Path(__file__).resolve().parents[1] / "urdf" / "bravo_5_dynamics_pinocchio.urdf"


def default_package_dir() -> Path:
    # Workspace src/ (contains bravo_cathode_protection folder for package:// resolution).
    return Path(__file__).resolve().parents[2]


def clamp_positive(values: np.ndarray, min_value: float = 1e-12) -> np.ndarray:
    return np.maximum(values, min_value)


def draw_ellipsoid_from_svd(
    viz: MeshcatVisualizer,
    path: str,
    U: np.ndarray,
    radii: np.ndarray,
    placement: np.ndarray,
    color: int,
    opacity: float,
    scale: float,
) -> None:
    radii = np.asarray(radii, dtype=float)
    radii = clamp_positive(radii)
    transform = placement.copy()
    transform[:3, :3] = transform[:3, :3] @ U
    material = g.MeshLambertMaterial(color=color, transparent=True, opacity=opacity)
    viz.viewer[path].set_object(g.Ellipsoid(scale * radii), material)
    viz.viewer[path].set_transform(transform)


class BravoInertiaEllipsoidViewer(Node):
    def __init__(
        self,
        topic: str,
        urdf_path: Path,
        package_dir: Path,
        frame_name: str,
        motion_scale: float,
        force_scale: float,
        visualization_rate_hz: float,
        open_browser: bool,
        q_log_period_sec: float,
    ) -> None:
        super().__init__("bravo5_inertia_ellipsoid_viewer")

        self.topic = topic
        self.motion_scale = motion_scale
        self.force_scale = force_scale
        self.frame_name = frame_name
        self.q_log_period_sec = max(0.1, q_log_period_sec)
        self.visualization_rate_hz = max(0.2, visualization_rate_hz)
        self.last_q_log_time = self.get_clock().now()

        urdf_str = str(urdf_path.resolve())
        package_dirs = [str(package_dir.resolve())]
        self.model, self.collision_model, self.visual_model = pin.buildModelsFromUrdf(
            urdf_str, package_dirs=package_dirs, meshLoader=pin.MeshLoader()
        )
        self.data = self.model.createData()

        self.frame_id = self.model.getFrameId(frame_name)
        if self.frame_id >= len(self.model.frames):
            raise ValueError(
                f"Frame '{frame_name}' not found in URDF model. "
                f"Available last frame: '{self.model.frames[-1].name}'."
            )

        self.viz = MeshcatVisualizer(self.model, self.collision_model, self.visual_model)
        self.viz.initViewer(open=open_browser)
        self.viz.loadViewerModel(rootNodeName="bravo5")

        self.q = pin.neutral(self.model)
        self.has_new_state = True
        self.viz.display(self.q)
        self._draw_svd_ellipsoids()

        self.joint_name_to_q_index: Dict[str, int] = {}
        for joint_id in range(1, len(self.model.names)):  # skip universe joint
            name = self.model.names[joint_id]
            nq_joint = int(self.model.nqs[joint_id])
            if nq_joint == 1:
                self.joint_name_to_q_index[name] = int(self.model.idx_qs[joint_id])

        self.sub = self.create_subscription(JointState, topic, self._joint_state_cb, 20)
        self.visual_timer = self.create_timer(
            1.0 / self.visualization_rate_hz, self._visualization_timer_cb
        )
        self.get_logger().info(f"Subscribed to '{topic}'")
        self.get_logger().info(f"URDF: {urdf_str}")
        self.get_logger().info(f"Frame for ellipsoid: '{frame_name}'")
        self.get_logger().info(f"Motion scale: {self.motion_scale:.4f} | Force scale: {self.force_scale:.4f}")
        self.get_logger().info(f"Visualization rate: {self.visualization_rate_hz:.2f} Hz")
        self.get_logger().info(f"Meshcat URL: {self.viz.viewer.url()}")

    def _joint_state_cb(self, msg: JointState) -> None:
        updated = False
        for name, value in zip(msg.name, msg.position):
            q_idx = self.joint_name_to_q_index.get(name)
            if q_idx is None:
                continue
            self.q[q_idx] = float(value)
            updated = True

        if not updated:
            return

        self.has_new_state = True

    def _visualization_timer_cb(self) -> None:
        if not self.has_new_state:
            return

        self.viz.display(self.q)
        self._draw_svd_ellipsoids()
        self._maybe_log_current_q()
        self.has_new_state = False

    def _draw_svd_ellipsoids(self) -> None:
        pin.forwardKinematics(self.model, self.data, self.q)
        pin.updateFramePlacements(self.model, self.data)

        jac = pin.computeFrameJacobian(
            self.model, self.data, self.q, self.frame_id, pin.ReferenceFrame.WORLD
        )
        jac_lin = jac[:3, :]

        U, s, _ = np.linalg.svd(jac_lin, full_matrices=False)
        s = clamp_positive(s)
        motion_radii = s
        force_radii = 1.0 / s
        placement = self.data.oMf[self.frame_id].homogeneous.copy()

        draw_ellipsoid_from_svd(
            self.viz,
            "world/motion_ellipsoid",
            U,
            motion_radii,
            placement,
            color=0x2609DE,
            opacity=0.45,
            scale=self.motion_scale,
        )
        draw_ellipsoid_from_svd(
            self.viz,
            "world/force_ellipsoid",
            U,
            force_radii,
            placement,
            color=0xFF0000,
            opacity=0.35,
            scale=self.force_scale,
        )

        ee_marker = g.MeshLambertMaterial(color=0xFF5533, transparent=False, opacity=1.0)
        self.viz.viewer["world/contact_point_marker"].set_object(g.Sphere(0.007), ee_marker)
        marker_tf = np.eye(4)
        marker_tf[:3, 3] = placement[:3, 3]
        self.viz.viewer["world/contact_point_marker"].set_transform(marker_tf)

    def _maybe_log_current_q(self) -> None:
        now = self.get_clock().now()
        elapsed = (now - self.last_q_log_time).nanoseconds * 1e-9
        if elapsed < self.q_log_period_sec:
            return

        q_items = []
        for name, idx in self.joint_name_to_q_index.items():
            q_items.append(f"{name}={self.q[idx]:.3f}")
        self.get_logger().info("Current joints: " + ", ".join(q_items))
        self.last_q_log_time = now


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Subscribe to Bravo joint states and visualize URDF + SVD motion/force ellipsoids in Meshcat."
    )
    parser.add_argument(
        "--topic",
        default="/bravo/fdb/joint_states",
        help="JointState topic name (default: /bravo/fdb/joint_states)",
    )
    parser.add_argument(
        "--urdf",
        default=str(default_urdf_path()),
        help="Absolute path to URDF file",
    )
    parser.add_argument(
        "--package-dir",
        default=str(default_package_dir()),
        help="Workspace directory containing robot packages for package:// mesh resolution",
    )
    parser.add_argument(
        "--motion-scale",
        type=float,
        default=0.2,
        help="Scale multiplier for motion ellipsoid radii",
    )
    parser.add_argument(
        "--force-scale",
        type=float,
        default=0.02,
        help="Scale multiplier for force ellipsoid radii",
    )
    parser.add_argument(
        "--visualization-rate",
        type=float,
        default=5.0,
        help="Meshcat/ellipsoid update rate in Hz (lower keeps computation light)",
    )
    parser.add_argument(
        "--q-log-period",
        type=float,
        default=1.0,
        help="Seconds between current joint-state logs",
    )
    parser.add_argument(
        "--no-open-browser",
        action="store_true",
        help="Do not ask Meshcat to open a browser tab",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    rclpy.init()
    node = BravoInertiaEllipsoidViewer(
        topic=args.topic,
        urdf_path=Path(args.urdf),
        package_dir=Path(args.package_dir),
        frame_name=ELLIPSOID_FRAME,
        motion_scale=args.motion_scale,
        force_scale=args.force_scale,
        visualization_rate_hz=args.visualization_rate,
        open_browser=not args.no_open_browser,
        q_log_period_sec=args.q_log_period,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
