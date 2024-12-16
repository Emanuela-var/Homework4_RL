#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
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

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import yaml, os, math
from ament_index_python.packages import get_package_share_directory



waypoints = yaml.safe_load(
    open(os.path.join(get_package_share_directory('rl_fra2mo_description'), "config", "goal.yaml"))
)

#import initial pose
initial_pose = yaml.safe_load(
    open(os.path.join(get_package_share_directory('rl_fra2mo_description'), "config", "initial_pose.yaml"))
)

def main():
    rclpy.init()
    navigator = BasicNavigator()

    def create_pose(transform):
        # Parametri della trasformazione (da mappa a odometria)
        initial_translation = {"x": -3, "y": 3.5}
        initial_rotation = -math.pi / 2  # -90° in radianti

        # Estraggo posizione e orientamento dal transform
        map_position = transform["position"]
        map_orientation = transform["orientation"]

        # Trasformo la posizione dal frame mappa al frame odometria
        translated_x = map_position["x"] - initial_translation["x"]
        translated_y = map_position["y"] - initial_translation["y"]

        # Applico la rotazione inversa
        odom_x = math.cos(-initial_rotation) * translated_x - math.sin(-initial_rotation) * translated_y
        odom_y = math.sin(-initial_rotation) * translated_x + math.cos(-initial_rotation) * translated_y

        # L'orientamento (quaternione) deve essere ruotato per tener conto della rotazione iniziale
        # Funzione per ruotare un quaternione di un angolo attorno all'asse Z
        def rotate_quaternion_z(q, theta):
            # Costruisco il quaternione della rotazione
            rot_q = {
                "x": 0,
                "y": 0,
                "z": math.sin(theta / 2),
                "w": math.cos(theta / 2)
            }

            # Moltiplico i quaternioni: rot_q * q
            new_q = {
                "x": rot_q["w"] * q["x"] + rot_q["x"] * q["w"] + rot_q["y"] * q["z"] - rot_q["z"] * q["y"],
                "y": rot_q["w"] * q["y"] - rot_q["x"] * q["z"] + rot_q["y"] * q["w"] + rot_q["z"] * q["x"],
                "z": rot_q["w"] * q["z"] + rot_q["x"] * q["y"] - rot_q["y"] * q["x"] + rot_q["z"] * q["w"],
                "w": rot_q["w"] * q["w"] - rot_q["x"] * q["x"] - rot_q["y"] * q["y"] - rot_q["z"] * q["z"]
            }
            return new_q

        # Ruoto il quaternione dell'orientamento
        odom_orientation = rotate_quaternion_z(map_orientation, -initial_rotation)

        # Creo il PoseStamped nel frame odometria
        pose = PoseStamped()
        pose.header.frame_id = 'map'  # Ora è nel frame odometria
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = odom_x
        pose.pose.position.y = odom_y
        pose.pose.position.z = map_position["z"]  # La quota rimane invariata
        pose.pose.orientation.x = odom_orientation["x"]
        pose.pose.orientation.y = odom_orientation["y"]
        pose.pose.orientation.z = odom_orientation["z"]
        pose.pose.orientation.w = odom_orientation["w"]

        return pose




    # Create all poses from YAML file
    all_goal_poses = list(map(create_pose, waypoints["waypoints"]))


    # Reorder the goals: Goal 3 → Goal 4 → Goal 2 → Goal 1
    reordered_goal_indices = [2, 3, 1, 0]  # Python indexing starts at 0
    goal_poses = [all_goal_poses[i] for i in reordered_goal_indices]


    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active(localizer="smoother_server")

    # sanity check a valid path exists
    # path = navigator.getPath(initial_pose, goal_pose)

    nav_start = navigator.get_clock().now()
    navigator.followWaypoints(goal_poses)

    i = 0
    while not navigator.isTaskComplete():

        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()

        if feedback and i % 5 == 0:
            print('Executing current waypoint: ' +
                  str(feedback.current_waypoint + 1) + '/' + str(len(goal_poses)))
            now = navigator.get_clock().now()

            # Some navigation timeout to demo cancellation
            if now - nav_start > Duration(seconds=600):
                navigator.cancelTask()

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    # navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()