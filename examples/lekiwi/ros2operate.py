# !/usr/bin/env python

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import time

import cv2

from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig
from lerobot.utils.robot_utils import precise_sleep

FPS = 30
ARM_KEYS = (
    "arm_shoulder_pan.pos",
    "arm_shoulder_lift.pos",
    "arm_elbow_flex.pos",
    "arm_wrist_flex.pos",
    "arm_wrist_roll.pos",
    "arm_gripper.pos",
)


def main():
    parser = argparse.ArgumentParser(description="Drive LeKiwi from ROS 2 /cmd_vel.")
    parser.add_argument("--remote-ip", required=True, help="IP address of the LeKiwi host.")
    parser.add_argument("--robot-id", default="lekiwi", help="LeKiwi robot id.")
    parser.add_argument("--camera", default="front", help="Observation camera to display.")
    args = parser.parse_args()

    robot = LeKiwiClient(LeKiwiClientConfig(remote_ip=args.remote_ip, id=args.robot_id))
    robot.connect()

    if not robot.is_connected:
        raise ValueError("Robot is not connected!")

    print("Starting ROS 2 teleop loop...")
    while True:
        t0 = time.perf_counter()

        observation = robot.get_observation()
        camera = observation.get(args.camera)
        if camera is not None:
            img = cv2.cvtColor(camera, cv2.COLOR_RGB2BGR)
            cv2.imshow(args.camera, img)
            cv2.waitKey(1)

        arm_action = {key: observation[key] for key in ARM_KEYS}
        base_action = robot._from_twist_to_base_action()
        if base_action:
            action = {**arm_action, **base_action}
            _ = robot.send_action(action)

        precise_sleep(max(1.0 / FPS - (time.perf_counter() - t0), 0.0))


if __name__ == "__main__":
    main()
