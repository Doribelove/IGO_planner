#!/usr/bin/env python3
"""Run the installed ROS example on a fresh loopback ROS master."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile


def stop_child(process):
    for sig, timeout in ((signal.SIGINT, 15), (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
        if process.poll() is not None:
            return
        os.killpg(process.pid, sig)
        try:
            process.wait(timeout=timeout)
            return
        except subprocess.TimeoutExpired:
            continue


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, help='New result directory (default: a temporary directory)')
    parser.add_argument('--port', type=int, default=0, help='Unused loopback master port (default: automatic)')
    args = parser.parse_args()
    if args.port and not 1024 <= args.port <= 65535:
        parser.error('--port must be in 1024..65535')
    output = (args.output or Path(tempfile.mkdtemp(prefix='igo_simulation_'))).resolve()
    output.mkdir(parents=True, exist_ok=True)
    report_file = output / 'ros_simulation.json'
    if report_file.exists():
        parser.error('Result already exists; choose a new --output directory')
    # Do not attach to the inherited ROS master, including another local master.
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as port_probe:
        try:
            port_probe.bind(('127.0.0.1', args.port))
        except OSError as error:
            parser.error('Master port is unavailable: ' + str(error))
        port = port_probe.getsockname()[1]
    env = os.environ.copy()
    env.pop('ROS_HOSTNAME', None)
    env.pop('ROS_NAMESPACE', None)
    env.update(ROS_MASTER_URI='http://127.0.0.1:' + str(port), ROS_IP='127.0.0.1',
               ROS_IPV6='off', IGO_ISOLATED_SIMULATION='1',
               ROS_HOME=str(output / 'ros_home'), ROS_LOG_DIR=str(output / 'ros_logs'))
    command = ['roslaunch', '-p', str(port), 'igo_local_planner',
               'simulation.launch', 'output:=' + str(output)]
    print('Simulation master: ' + env['ROS_MASTER_URI'], flush=True)
    print('Simulation results: ' + str(output), flush=True)
    process = subprocess.Popen(command, env=env, start_new_session=True)
    try:
        return_code = process.wait(timeout=180)
    except (subprocess.TimeoutExpired, KeyboardInterrupt):
        stop_child(process)
        return 1
    finally:
        stop_child(process)
    if return_code or not report_file.is_file():
        print('Simulation did not produce a successful result.', flush=True)
        return 1
    report = json.loads(report_file.read_text())
    results = report.get('results', [])
    passed = (not report.get('errors') and len(results) == 3
              and all(result.get('action_state') == 3 for result in results))
    print('Simulation ' + ('passed' if passed else 'failed') + ': ' + str(report_file))
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
