#!/usr/bin/env python3
"""Closed-loop ROS integration experiment; run only via the isolated test runner."""
import json
import math
import os
from pathlib import Path
import threading
import time
from urllib.parse import urlparse
import actionlib
import dynamic_reconfigure.client
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import String


def main():
    assert os.environ.get('IGO_ISOLATED_SIMULATION') == '1'
    assert urlparse(os.environ['ROS_MASTER_URI']).hostname == '127.0.0.1'
    rospy.init_node('igo_simulated_vehicle')
    output = Path(rospy.get_param('~output'))
    output.mkdir(parents=True, exist_ok=True)
    pose = [0., 0., 0.]
    command = [0., 0.]
    trace = []
    diagnostics = []
    errors = []
    lock = threading.RLock()
    gear = [0]
    def on_command(msg):
        with lock:
            v, w = msg.linear.x, msg.angular.z
            if not (math.isfinite(v) and math.isfinite(w)):
                errors.append('nonfinite command')
            if v > .800001 or v < -.300001 or abs(w) > abs(v)/1.35+1e-5:
                errors.append('speed or turning radius constraint')
            if gear[0] and v*gear[0] < -1e-5:
                errors.append('fixed gear constraint')
            command[:] = [v, w]
    def on_status(msg):
        with lock:
            diagnostics.append(json.loads(msg.data))
    rospy.Subscriber('/cmd_vel', Twist, on_command, queue_size=1)
    rospy.Subscriber('/move_base/IGOLocalPlannerROS/status', String, on_status, queue_size=100)
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
    map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1, latch=True)
    broadcaster = tf2_ros.TransformBroadcaster()
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.resolution = .1
    grid.info.width = grid.info.height = 240
    grid.info.origin.position.x = grid.info.origin.position.y = -12.
    grid.info.origin.orientation.w = 1.
    grid.data = [0]*(240*240)
    map_pub.publish(grid)
    start = time.monotonic()
    last = [start]
    def integrate(_event):
        with lock:
            now = time.monotonic()
            dt = min(.1, now-last[0]); last[0] = now
            v, w = command
            # The test vehicle executes an ideal planar Ackermann twist.
            pose[0] += v*math.cos(pose[2]+.5*w*dt)*dt
            pose[1] += v*math.sin(pose[2]+.5*w*dt)*dt
            pose[2] += w*dt
            stamp = rospy.Time.now()
            tf = TransformStamped(); tf.header.stamp = stamp
            tf.header.frame_id = 'map'; tf.child_frame_id = 'base_link'
            tf.transform.translation.x, tf.transform.translation.y = pose[:2]
            tf.transform.rotation.z = math.sin(pose[2]/2)
            tf.transform.rotation.w = math.cos(pose[2]/2)
            broadcaster.sendTransform(tf)
            odom = Odometry(); odom.header = tf.header; odom.child_frame_id = 'base_link'
            odom.pose.pose.position.x, odom.pose.pose.position.y = pose[:2]
            odom.pose.pose.orientation = tf.transform.rotation
            odom.twist.twist.linear.x, odom.twist.twist.angular.z = command
            odom_pub.publish(odom)
            trace.append([now-start]+list(pose)+list(command))
    timer = rospy.Timer(rospy.Duration(.02), integrate)
    results = []
    try:
        client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        if not client.wait_for_server(rospy.Duration(20)):
            raise RuntimeError('simulation move_base did not start')
        tuning = dynamic_reconfigure.client.Client('/move_base/IGOLocalPlannerROS', timeout=15)
        for name, x, y, yaw, direction in [
                ('forward_goal', 3., 0., 0., 1),
                ('reverse_goal', 1., 0., 0., -1),
                ('automatic_gear', -1., 0., 0., 0)]:
            tuning.update_configuration({'motion_direction_mode': direction,
                                         'allow_init_with_backwards_motion': True})
            with lock:
                gear[0] = direction
            goal = MoveBaseGoal(); goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = x; goal.target_pose.pose.position.y = y
            goal.target_pose.pose.orientation.z = math.sin(yaw/2)
            goal.target_pose.pose.orientation.w = math.cos(yaw/2)
            before = time.monotonic(); client.send_goal(goal)
            done = client.wait_for_result(rospy.Duration(45))
            if not done:
                client.cancel_goal()
            with lock:
                result = {'case': name, 'action_state': client.get_state(),
                          'elapsed_s': time.monotonic()-before, 'pose': list(pose),
                          'position_error_m': math.hypot(pose[0]-x, pose[1]-y)}
                results.append(result)
            rospy.loginfo('IGO SIMULATION: %s', json.dumps(result))
            if not done or client.get_state() != 3:
                raise RuntimeError('simulated goal did not succeed: '+name)
        if errors:
            raise RuntimeError('; '.join(set(errors)))
    except Exception as error:
        errors.append(str(error))
    finally:
        timer.shutdown()
        with lock:
            report = {'hardware_motion': False, 'results': results,
                      'errors': errors, 'diagnostics': diagnostics, 'trace': trace}
        (output/'ros_simulation.json').write_text(json.dumps(report, indent=2)+'\n')
    if errors:
        raise RuntimeError(errors)


if __name__ == '__main__':
    main()
