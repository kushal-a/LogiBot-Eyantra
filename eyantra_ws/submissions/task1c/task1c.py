#!/usr/bin/env python3
"""
Example of moving to a pose goal.
`ros2 run pymoveit2 ex_pose_goal.py --ros-args -p position:="[0.25, 0.0, 1.0]" -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]" -p cartesian:=False`
"""

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
import numpy as np
from pymoveit2 import MoveIt2   
from pymoveit2.robots import ur5

from linkattacher_msgs.srv import AttachLink, DetachLink



def main():
    rclpy.init()

    # Create node for this example
    node = Node("ex_pose_goal")

    # Declare parameters for position and orientation
    # node.declare_parameter("orientation", [0.0, 0.0, 0.0, 1.0])
    # node.declare_parameter("position", [0.35, 0.10, 0.68])
    # node.declare_parameter("quat_xyzw", [0.5, 0.5, 0.5, 0.5])
    # node.declare_parameter("cartesian", False)

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=ur5.joint_names(),
        base_link_name=ur5.base_link_name(),
        end_effector_name=ur5.end_effector_name(),
        group_name=ur5.MOVE_GROUP_ARM,
        callback_group=callback_group,
        # execute_via_moveit=False
    )

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()


    gripper_attach = node.create_client(AttachLink, '/GripperMagnetON')
    gripper_detach = node.create_client(DetachLink, '/GripperMagnetOFF')



    # Get parameters
    # position = node.get_parameter("position").get_parameter_value().double_array_value
    # quat_xyzw = node.get_parameter("quat_xyzw").get_parameter_value().double_array_value
    # cartesian = node.get_parameter("cartesian").get_parameter_value().bool_value

    P1 = [0.2,-0.47,0.65]
    P2 = [0.75,0.49,-0.05]
    P3 = [0.75,-0.23,-0.05]
    D = [-0.69, 0.1, 0.44]
    P1_quat_xyzw = [0.707,0.0,0.0,0.707]
    P2_quat_xyzw = [1.0,0.0,0.0,0.0]
    P3_quat_xyzw = [1.0,0.0,0.0,0.0]
    D_quat_xyzw = [0.0,-0.707,0.0,0.707]

    home_fwd = [0.0, -2.391, 2.40855, -np.pi, -np.pi/2, np.pi]
    home_rig = [-np.pi/2, -2.391, 2.40855, -np.pi, -np.pi/2, np.pi]
    home_bwd = [-np.pi, -2.391, 2.40855, -np.pi, -np.pi/2, np.pi]
    
    # joint_P1 = moveit2.compute_ik(position=P1, quat_xyzw=P1_quat_xyzw)
    # print(joint_P1)
    # joint_P1 = [-3.9832473089965172, -2.9668291237139126, 1.3859875153596655, -1.559741518856695, 0.8416545422481527, 3.1409221517336494]
    joint_P1 = np.array([-76.0, -86.0, 81.0, -175.0, -174.0+90, 180.0])*np.pi/180
    # joint_P2 = moveit2.compute_ik(position=P2, quat_xyzw=P2_quat_xyzw)
    # print(joint_P2)
    # joint_P2 = [2*np.pi - 5.826609400781735, 5.995409431630927-2*np.pi, -0.4510616587220052, 4.875674761423936, 4.712388981702234, -1.1142204205999233]
    # joint_P2 = [2*np.pi - 5.826609400781735, 5.995409431630927-2*np.pi, 1.029744258676, -2.1293016874330, 1.6406094, 1.6406094]
    # joint_P2 = np.array([22.0,-26.0,59.0,-122.0,-94.0,94.0])*np.pi/180
    # joint_P2 = np.array([20.0,-18.0,44.0,-115.0,-89.0,200.0])*np.pi/180
    joint_P2 =  np.array([19.0, -17.0, 43.0, -113.0,-90.0, 180.0-19.0])*np.pi/180
    joint_P2 = [0.45657589445203234, -0.1449335141273318, 0.45113611444659635, -1.8769989109959222, -1.5707962691510837, 5.16896487095665]
    # joint_P3 = moveit2.compute_ik(position=P3, quat_xyzw=P3_quat_xyzw)
    # print(joint_P3)
    # joint_P3 = [-0.43715219450093423, -0.4690693453328723, 1.1567594740447213, -2.258486455208235, -1.5707963266608471, 4.275236785679264]
    joint_P3 =  np.array([-26.0,-26.0, 63.0, -125.0, -92.0, 180.0+26.0])*np.pi/180
    joint_P3 = [-0.43715237245196925, -0.4690659784884803, 1.1567541676691053, -2.258484515348445, -1.570796325879717, 4.275236607520216]
    # joint_P3 = np.array([-25.0,-30.0,69.0,-128.0,-90.0,47.0])*np.pi/180
    # joint_P3 = np.array([-26.0,-27.0,-63.0,-126.0,-99.0,-155.0])*np.pi/180
    # joint_D = moveit2.compute_ik(position=D, quat_xyzw=D_quat_xyzw)
    # print(joint_D)
    # joint_D = [2.808,-0.227,-0.855, 1.204, 2*np.pi-5.024, np.pi]
    joint_D =  np.array([-203.0, -70.0, 108.0, -219.0, -67.0, 180.0])*np.pi/180
    joint_D = [2.800331611206596 - 2*np.pi, -1.0055300487735468, 1.2706011356905693, -3.407464609342404, 5.053650085927885 - 2*np.pi, -1.5705288975419869]

    # p1 = moveit2.compute_fk(list(joint_P1))
    # p2 = moveit2.compute_fk(list(joint_P2))
    # p3 = moveit2.compute_fk(list(joint_P3))
    # d = moveit2.compute_fk(list(joint_D))
    # print(p1[0].pose.position)
    # print(p1[0].pose.orientation)
    # print(p2[0].pose.position)
    # print(p2[0].pose.orientation)
    # print(p3[0].pose.position)
    # print(p3[0].pose.orientation)
    # print(d[0].pose.position)
    # print(d[0].pose.orientation)
    
    # position = D
    # quat_xyzw = [0.0,-0.707,0,0.707]

    # node.get_logger().info(
    #     f"Moving to {{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}"
    # )
    # moveit2.move_to_pose(position=position, quat_xyzw=quat_xyzw, cartesian=True)
    # moveit2.wait_until_executed()
    # if(True):
        # joint_D = joint_D.position

    going_to_config(node, home_rig, moveit2)
    going_to_config(node, joint_P1, moveit2)
    attach(node,'box1' , gripper_attach)
    going_to_config(node, home_rig, moveit2)
    going_to_config(node, home_bwd, moveit2)
    going_to_config(node, joint_D, moveit2)
   
    detatch(node,'box1' , gripper_detach)
    going_to_config(node, home_bwd, moveit2)
    going_to_config(node, home_fwd, moveit2)
    going_to_config(node, joint_P2, moveit2)
   
    attach(node, 'box3' , gripper_attach)
    going_to_config(node, home_fwd, moveit2)
    going_to_config(node, home_bwd, moveit2)
    going_to_config(node, joint_D, moveit2)
    detatch(node, 'box3' ,gripper_detach)
    going_to_config(node, home_bwd, moveit2)
    going_to_config(node, home_fwd, moveit2)
    going_to_config(node, joint_P3, moveit2)

    attach(node, 'box49' ,gripper_attach)
    going_to_config(node, home_fwd, moveit2)
    going_to_config(node, home_bwd, moveit2)
    going_to_config(node, joint_D, moveit2)
    detatch(node,'box49' , gripper_detach)
    going_to_config(node, home_bwd, moveit2)
    going_to_config(node, home_fwd, moveit2)


    rclpy.shutdown()
    exit(0)

def going_to_config(node, joints, moveit2):
    # node.get_logger().info(f"Moving to {{joint_positions: {joints}}}")
    moveit2.move_to_configuration(joints, cartesian=False)
    moveit2.wait_until_executed()

def attach(node,box,  gripper_attach):
    while not gripper_attach.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('EEF service not available, waiting again...')

    req = AttachLink.Request()
    req.model1_name =  box    
    req.link1_name  = 'link'       
    req.model2_name =  'ur5'       
    req.link2_name  = 'wrist_3_link'  

    gripper_attach.call_async(req)

    # print("Attached*****************************")

def detatch(node, box, gripper_detach):
    while not gripper_detach.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('EEF service not available, waiting again...')

    req = DetachLink.Request()
    req.model1_name =  box   
    req.link1_name  = 'link'       
    req.model2_name =  'ur5'       
    req.link2_name  = 'wrist_3_link'  

    gripper_detach.call_async(req)

    # print("Detached*****************************")

if __name__ == "__main__":
    main()
