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

"""
Basic navigation demo to go to poses.
"""


def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Set our demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 1.84
    initial_pose.pose.position.y = -9.05
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 3.14
    navigator.setInitialPose(initial_pose)

    # Activate navigation, if not autostarted. This should be called after setInitialPose()
    # or this will initialize at the origin of the map and update the costmap with bogus readings.
    # If autostart, you should `waitUntilNav2Active()` instead.
    navigator.lifecycleStartup()

    # Wait for navigation to fully activate, since autostarting nav2
    # navigator.waitUntilNav2Active()

    # If desired, you can change or load the map as well
    navigator.changeMap('/home/ubuntu22/eyantra_ws/src/eyrc-24-25-logistic-cobot/ebot_nav2/maps/map_final.yaml')

    # You may use the navigator to clear or obtain costmaps
    # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    # global_costmap = navigator.getGlobalCostmap()
    # local_costmap = navigator.getLocalCostmap()

    # set our demo's goal poses
    goal_poses = []
    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = 'map'
    goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose1.pose.position.x = -0.12
    goal_pose1.pose.position.y = -2.35
    goal_pose1.pose.orientation.w = 3.14
    goal_pose1.pose.orientation.z = 0.0
    goal_poses.append(goal_pose1)

    # additional goals can be appended
    goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id = 'map'
    goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose2.pose.position.x = 1.86
    goal_pose2.pose.position.y = 2.56
    goal_pose2.pose.orientation.w = 0.97
    goal_pose2.pose.orientation.z = 0.0
    goal_poses.append(goal_pose2)
    goal_pose3 = PoseStamped()
    goal_pose3.header.frame_id = 'map'
    goal_pose3.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose3.pose.position.x = -3.84
    goal_pose3.pose.position.y = 2.64
    goal_pose3.pose.orientation.w = 2.78
    goal_pose3.pose.orientation.z = 0.0
    goal_poses.append(goal_pose3)

    for j in range(len(goal_poses)):
        # sanity check a valid path exists
        if j==0:
            path = navigator.getPath(initial_pose, goal_poses[j])
        else:
            path = navigator.getPath(goal_poses[j-1], goal_poses[j])
        
        # navigator.goThroughPoses(goal_poses)
        smoothed_path = navigator.smoothPath(path)        
        navigator.followPath(smoothed_path)

        i = 0
        while not navigator.isTaskComplete():
            ################################################
            #
            # Implement some code here for your application!
            #
            ################################################

            # Do something with the feedback
            i = i + 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print(feedback)
                print(j)

                # Some navigation timeout to demo cancellation
                # if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                #     navigator.cancelTask()


        # Do something depending on the return code
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
            break
        elif result == TaskResult.FAILED:
            print('Goal failed!')
            break
        else:
            print('Goal has an invalid return status!')
            break

    navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()
