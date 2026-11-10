#!/usr/bin/env python3
"""Real Nav2 controller_server + installed IGO plugin, synthetic kinematic plant.
No real hardware, no sensor input, no simulator, no obstacle-avoidance claim.
Run after sourcing the actual built install/setup.bash; writes to a new result directory.
"""
import argparse
import hashlib
import json
import math
import os
import pathlib
import signal
import subprocess
import sys
import tempfile
import time
import traceback
import uuid

ap = argparse.ArgumentParser(
    description="Synthetic real Nav2 integration only. Choose an UNUSED ROS domain; localhost is not absolute isolation."
)
ap.add_argument(
    "--domain",
    type=int,
    default=198,
    help="UNUSED ROS domain (default:198); never use a physical robot domain",
)
ap.add_argument(
    "--output", help="New or empty evidence directory; default is tempfile.mkdtemp()"
)
args = ap.parse_args()
if not 0 <= args.domain <= 232:
    ap.error("--domain must be in [0,232]")
os.environ["ROS_DOMAIN_ID"] = str(args.domain)
os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
out = (
    pathlib.Path(args.output).expanduser().resolve()
    if args.output
    else pathlib.Path(tempfile.mkdtemp(prefix="igo-nav2-smoke-"))
)
if out.exists() and (not out.is_dir() or any(out.iterdir())):
    ap.error("--output must be a new or empty directory; refusing to mix evidence")
out.mkdir(parents=True, exist_ok=True)
import yaml
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from ament_index_python.packages import get_package_prefix
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav_msgs.msg import Odometry, Path
from nav2_msgs.action import FollowPath
from tf2_ros import TransformBroadcaster

NS = "/igo_smoke_" + str(os.getpid()) + "_" + uuid.uuid4().hex[:8]
common = {
    "plugin": "igo_nav2_controller::IGOController",
    "max_vel_x": 0.5,
    "max_vel_x_backwards": 0.3,
    "max_vel_theta": 0.6,
    "acc_lim_x": 2.0,
    "acc_lim_theta": 0.5,
    "decel_lim_x": 0.4,
    "stopping_delay": 0.2,
    "wheelbase": 0.65,
    "min_turning_radius": 1.35,
    "max_steering_rate": 0.6,
    "steering_change_weight": 0.5,
    "xy_goal_tolerance": 0.15,
    "yaw_goal_tolerance": 0.25,
    "max_global_plan_lookahead_dist": 4.0,
    "allow_init_with_backwards_motion": True,
    "global_plan_overwrite_orientation": True,
    "treat_unknown_as_obstacle": True,
    "goal_yaw_required": True,
    "horizon": 4.0,
    "knots": 10,
    "samples": 48,
    "iterations": 4,
    "transform_tolerance": 0.3,
}
controller = {
    "use_sim_time": False,
    "controller_frequency": 10.0,
    "enable_stamped_cmd_vel": False,
    "min_x_velocity_threshold": 0.001,
    "min_y_velocity_threshold": 0.001,
    "min_theta_velocity_threshold": 0.001,
    "odom_topic": "odom",
    "progress_checker_plugins": ["progress_checker"],
    "goal_checker_plugins": ["goal_checker"],
    "controller_plugins": ["Forward", "Reverse"],
    "progress_checker": {
        "plugin": "nav2_controller::SimpleProgressChecker",
        "required_movement_radius": 0.05,
        "movement_time_allowance": 20.0,
    },
    "goal_checker": {
        "plugin": "nav2_controller::StoppedGoalChecker",
        "xy_goal_tolerance": 0.15,
        "yaw_goal_tolerance": 0.25,
        "trans_stopped_velocity": 0.03,
        "rot_stopped_velocity": 0.03,
        "stateful": False,
    },
    "Forward": dict(common, motion_direction_mode=1),
    "Reverse": dict(common, motion_direction_mode=-1),
    "publish_zero_velocity": True,
    "failure_tolerance": 0.0,
    "costmap_update_timeout": 1.0,
}
costmap = {
    "use_sim_time": False,
    "global_frame": "map",
    "robot_base_frame": "base_link",
    "update_frequency": 20.0,
    "publish_frequency": 2.0,
    "transform_tolerance": 0.3,
    "rolling_window": True,
    "width": 12,
    "height": 12,
    "resolution": 0.1,
    "footprint": "[[1.32, 0.5], [1.32, -0.5], [-0.62, -0.5], [-0.62, 0.5]]",
    "footprint_padding": 0.01,
    "track_unknown_space": False,
    "plugins": ["inflation_layer"],
    "inflation_layer": {
        "plugin": "nav2_costmap_2d::InflationLayer",
        "inflation_radius": 0.6,
        "cost_scaling_factor": 3.0,
    },
    "always_send_full_costmap": True,
}
config = {
    NS + "/controller_server": {"ros__parameters": controller},
    NS + "/local_costmap/local_costmap": {"ros__parameters": costmap},
}
configpath = out / "controller_params.yaml"
configpath.write_text(yaml.safe_dump(config, sort_keys=False))


class Plant(Node):
    def __init__(self):
        super().__init__("igo_synthetic_plant", namespace=NS)
        self.x = self.y = self.yaw = self.v = self.w = 0.0
        self.last = time.monotonic()
        self.command_at = 0.0
        self.commands = []
        self.feedback = []
        self.odom = self.create_publisher(Odometry, "odom", 10)
        self.tf = TransformBroadcaster(self)
        self.sub = self.create_subscription(Twist, "cmd_vel", self.command, 10)
        self.timer = self.create_timer(0.02, self.step)
        self.action = ActionClient(self, FollowPath, "follow_path")

    def command(self, msg):
        vals = [
            msg.linear.x,
            msg.linear.y,
            msg.linear.z,
            msg.angular.x,
            msg.angular.y,
            msg.angular.z,
        ]
        if not all(math.isfinite(v) for v in vals):
            raise AssertionError("Nonfinite velocity command")
        if abs(msg.linear.y) > 1e-9:
            raise AssertionError("Non-Ackermann lateral command")
        self.v = msg.linear.x
        self.w = msg.angular.z
        self.command_at = time.monotonic()
        self.commands.append(
            {
                "t": self.command_at,
                "v": self.v,
                "w": self.w,
                "x": self.x,
                "y": self.y,
                "yaw": self.yaw,
            }
        )

    def step(self):
        now = time.monotonic()
        dt = min(now - self.last, 0.1)
        self.last = now
        if now - self.command_at > 0.5:
            self.v = self.w = 0.0
        self.x += self.v * math.cos(self.yaw + 0.5 * self.w * dt) * dt
        self.y += self.v * math.sin(self.yaw + 0.5 * self.w * dt) * dt
        self.yaw = math.atan2(
            math.sin(self.yaw + self.w * dt), math.cos(self.yaw + self.w * dt)
        )
        stamp = self.get_clock().now().to_msg()
        qz = math.sin(self.yaw / 2)
        qw = math.cos(self.yaw / 2)
        tr = TransformStamped()
        tr.header.stamp = stamp
        tr.header.frame_id = "map"
        tr.child_frame_id = "base_link"
        tr.transform.translation.x = self.x
        tr.transform.translation.y = self.y
        tr.transform.rotation.z = qz
        tr.transform.rotation.w = qw
        self.tf.sendTransform(tr)
        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = "map"
        od.child_frame_id = "base_link"
        od.pose.pose.position.x = self.x
        od.pose.pose.position.y = self.y
        od.pose.pose.orientation.z = qz
        od.pose.pose.orientation.w = qw
        od.twist.twist.linear.x = self.v
        od.twist.twist.angular.z = self.w
        self.odom.publish(od)

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait(self, future, timeout, what):
        end = time.monotonic() + timeout
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
        if not future.done():
            raise TimeoutError(what)
        if future.exception():
            raise future.exception()
        return future.result()

    def transition(self, transition):
        client = self.create_client(ChangeState, "controller_server/change_state")
        if not client.wait_for_service(timeout_sec=20):
            raise TimeoutError("controller lifecycle service")
        req = ChangeState.Request()
        req.transition.id = transition
        res = self.wait(
            client.call_async(req), 25, "lifecycle transition " + str(transition)
        )
        self.destroy_client(client)
        assert res.success, ("Lifecycle transition failed", transition)

    def goal(self, controller, distance):
        goal = FollowPath.Goal()
        goal.controller_id = controller
        goal.goal_checker_id = "goal_checker"
        goal.progress_checker_id = "progress_checker"
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        count = max(2, int(abs(distance) / 0.05) + 1)
        startx, starty = self.x, self.y
        # Both targets are on the vehicle heading axis. Reverse keeps final body yaw.
        for i in range(count + 1):
            point = PoseStamped()
            point.header = path.header
            point.pose.position.x = startx + distance * i / count * math.cos(self.yaw)
            point.pose.position.y = starty + distance * i / count * math.sin(self.yaw)
            point.pose.orientation.z = math.sin(self.yaw / 2)
            point.pose.orientation.w = math.cos(self.yaw / 2)
            path.poses.append(point)
        goal.path = path
        handle = self.wait(
            self.action.send_goal_async(
                goal,
                feedback_callback=lambda f: self.feedback.append(
                    float(f.feedback.distance_to_goal)
                ),
            ),
            10,
            "FollowPath goal acceptance",
        )
        assert handle.accepted, "FollowPath goal was rejected"
        return (
            handle,
            (path.poses[-1].pose.position.x, path.poses[-1].pose.position.y),
            time.monotonic(),
        )


def main():
    get_package_prefix(
        "igo_nav2_controller"
    )  # fail rather than substituting an unbuilt mock
    binary = (
        pathlib.Path(get_package_prefix("nav2_controller"))
        / "lib/nav2_controller/controller_server"
    )
    result = {
        "ros_domain_id": args.domain,
        "namespace": NS,
        "costmap": "12x12m free rolling grid, inflation-only layer, no sensors",
        "plant": "ideal commanded-velocity kinematics; no actuator dynamics/slip/obstacle coverage",
        "evidence_directory": str(out),
        "plugin_prefix": get_package_prefix("igo_nav2_controller"),
        "plugin_sha256": hashlib.sha256(
            (
                pathlib.Path(get_package_prefix("igo_nav2_controller"))
                / "lib/libigo_nav2_controller.so"
            ).read_bytes()
        ).hexdigest(),
        "cases": [],
        "passed": False,
    }
    serverlog = (out / "controller_server.log").open("w")
    server = None
    node = None
    try:
        rclpy.init()
        node = Plant()
        server = subprocess.Popen(
            [
                str(binary),
                "--ros-args",
                "-r",
                "__ns:=" + NS,
                "--params-file",
                str(configpath),
            ],
            stdout=serverlog,
            stderr=subprocess.STDOUT,
        )
        node.spin_for(1.0)
        node.transition(Transition.TRANSITION_CONFIGURE)
        node.spin_for(0.7)
        node.transition(Transition.TRANSITION_ACTIVATE)
        node.spin_for(1.0)
        assert node.action.wait_for_server(
            timeout_sec=10
        ), "FollowPath action unavailable"
        for controller, distance in [("Forward", 1.8), ("Reverse", -1.0)]:
            begin = len(node.commands)
            handle, target, started = node.goal(controller, distance)
            response = node.wait(
                handle.get_result_async(), 45, controller + " goal result"
            )
            node.spin_for(0.7)
            commands = node.commands[begin:]
            case = {
                "case": controller,
                "status": int(response.status),
                "error_code": int(response.result.error_code),
                "error_msg": response.result.error_msg,
                "seconds": time.monotonic() - started,
                "target": target,
                "final_pose": [node.x, node.y, node.yaw],
                "position_error": math.hypot(node.x - target[0], node.y - target[1]),
                "final_speed": node.v,
                "commands": len(commands),
                "min_v": min((c["v"] for c in commands), default=0),
                "max_v": max((c["v"] for c in commands), default=0),
            }
            result["cases"].append(case)
            print(json.dumps(case), flush=True)
            assert response.status == GoalStatus.STATUS_SUCCEEDED, case
            assert case["position_error"] <= 0.18 and abs(node.v) <= 0.03, case
            assert any(
                abs(c["v"]) <= 0.001 and abs(c["w"]) <= 0.001 for c in commands[-3:]
            ), "Server did not publish a final zero command"
            assert case["max_v"] > 0.1 if distance > 0 else case["min_v"] < -0.1, case
        begin = len(node.commands)
        handle, target, started = node.goal("Forward", 4.0)
        node.spin_for(1.3)
        assert node.v > 0.05, ("Cancel must be exercised while moving", node.v)
        before = node.v
        cancel_requested = time.monotonic()
        cancel_response = node.wait(handle.cancel_goal_async(), 10, "cancel acceptance")
        assert cancel_response.goals_canceling, "Cancel request not accepted"
        response = node.wait(
            handle.get_result_async(), 15, "canceled FollowPath result"
        )
        node.spin_for(0.7)
        case = {
            "case": "moving_cancel",
            "status": int(response.status),
            "error_code": int(response.result.error_code),
            "speed_at_request": before,
            "final_speed": node.v,
            "seconds": time.monotonic() - started,
            "final_pose": [node.x, node.y, node.yaw],
            "commands": len(node.commands) - begin,
            "zero_command_after_request_s": next(
                (
                    c["t"] - cancel_requested
                    for c in node.commands[begin:]
                    if c["t"] >= cancel_requested
                    and abs(c["v"]) <= 0.001
                    and abs(c["w"]) <= 0.001
                ),
                None,
            ),
        }
        result["cases"].append(case)
        print(json.dumps(case), flush=True)
        assert (
            response.status == GoalStatus.STATUS_CANCELED and abs(node.v) <= 0.001
        ), case
        assert (
            case["zero_command_after_request_s"] is not None
            and case["zero_command_after_request_s"] <= 2.0
        ), case
        node.transition(Transition.TRANSITION_DEACTIVATE)
        node.transition(Transition.TRANSITION_CLEANUP)
        result["passed"] = True
    except Exception as error:
        result["failure"] = str(error)
        result["traceback"] = traceback.format_exc()
        print(result["traceback"], file=sys.stderr, flush=True)
    finally:
        if node is not None:
            (out / "commands.json").write_text(json.dumps(node.commands, indent=2))
            result["command_count"] = len(node.commands)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if server is not None and server.poll() is None:
            server.send_signal(signal.SIGINT)
            try:
                server.wait(timeout=8)
            except subprocess.TimeoutExpired:
                server.terminate()
                server.wait(timeout=5)
        if server is not None:
            result["server_returncode"] = server.returncode
        serverlog.close()
        (out / "result.json").write_text(json.dumps(result, indent=2))
        print(json.dumps(result, indent=2), flush=True)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
