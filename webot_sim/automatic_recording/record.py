#!/usr/bin/env python3
"""
Automatically records one simulation output with Webots.

This wrapper will automatically start all required pieces of software,
record the animation, and exit.
"""

import argparse
import os
import subprocess
import shlex
import signal
import time
import atexit
import tempfile


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--master-firmware", required=True)
    parser.add_argument("--uavcan-controller", required=True)
    parser.add_argument("--webot-supervisor", required=True)
    parser.add_argument(
        "--output",
        "-o",
        help="Folder in which the web page will be stored.",
        required=True,
    )

    return parser.parse_args()


def start_webot(worldfile):
    pro = subprocess.Popen(shlex.split(f"webots {worldfile}"), preexec_fn=os.setsid)
    return pro


def start_master_firmware(exec_path, output_dir):
    cmd = f"{exec_path} --can_iface=vcan0"
    stderr = open(os.path.join(output_dir, "stderr.txt"), "w")
    stdout = open(os.path.join(output_dir, "stdout.txt"), "w")
    pro = subprocess.Popen(
        shlex.split(cmd), preexec_fn=os.setsid, stderr=stderr, stdout=stdout
    )
    return pro


def start_uavcan_webot_bridge(exec_path):
    cmd = f"{exec_path}"
    pro = subprocess.Popen(shlex.split(cmd), preexec_fn=os.setsid)
    return pro


def record(exec_path, output_path):
    output_path = os.path.abspath(output_path)
    cmd = f'{exec_path} --output_dir="{output_path}" --record_duration=20s'
    # Record is actually exiting once the recording is done, so we can use it
    # to wait as needed.
    subprocess.check_call(shlex.split(cmd))


def kill_component(process):
    os.killpg(os.getpgid(process.pid), signal.SIGTERM)


def register_cleanup():
    """
    Ensures all subprocess are killed on exit.
    """
    os.setpgrp()  # create new process group, become its leader

    def cleanup():
        print("Killing all subprocesses!")
        os.killpg(0, signal.SIGKILL)

    atexit.register(cleanup)


def compress(directory, output):
    cmd = f"tar -czf {output} {directory}"
    subprocess.call(shlex.split(cmd))


def main():
    args = parse_args()
    register_cleanup()

    tmpdir = tempfile.TemporaryDirectory()

    wf = os.path.join(
        os.path.dirname(__file__), "../Eurobot_2021/worlds/Eurobot_2021.wbt"
    )
    webot = start_webot(wf)
    master_firmware = start_master_firmware(args.master_firmware, tmpdir.name)
    uavcan_controller = start_uavcan_webot_bridge(args.uavcan_controller)

    record(args.webot_supervisor, tmpdir.name)

    kill_component(webot)
    kill_component(master_firmware)
    kill_component(uavcan_controller)

    compress(tmpdir.name, args.output)


if __name__ == "__main__":
    main()
