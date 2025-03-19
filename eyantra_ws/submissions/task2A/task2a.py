#!/usr/bin/env python3
"""
Example of moving to a pose goal.
`ros2 run pymoveit2 ex_pose_goal.py --ros-args -p position:="[0.25, 0.0, 1.0]" -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]" -p cartesian:=False`
"""

from threading import Thread

import rclpy
import rclpy
import sys
import tf2_ros
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
import numpy as np
from pymoveit2 import MoveIt2   
from pymoveit2.robots import ur5

from linkattacher_msgs.srv import AttachLink, DetachLink
from servo_msgs.srv import ServoLink



class servoing_tf(Node):
    '''
    ___CLASS___

    Description:    Class which servers purpose to define process for servoing.
    '''

    def __init__(self):
        '''
        Description:    Initialization of class aruco_tf
                        All classes have a function called __init__(), which is always executed when the class is being initiated.
                        The __init__() function is called automatically every time the class is being used to create a new object.
                        You can find more on this topic here -> https://www.w3schools.com/python/python_classes.asp
        '''

        super().__init__('servoing_tf_mover')                                          # registering node
                                                               
        self.tf_buffer = tf2_ros.buffer.Buffer()                                        # buffer time used for listening transforms
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)        

        # Create callback group that allows execution of callbacks in parallel without restrictions
        callback_group = ReentrantCallbackGroup()

        # Create MoveIt 2 interface
        moveit2 = MoveIt2(
            node=self,
            joint_names=ur5.joint_names(),
            base_link_name=ur5.base_link_name(),
            end_effector_name=ur5.end_effector_name(),
            group_name=ur5.MOVE_GROUP_ARM,
            callback_group=callback_group,
            # execute_via_moveit=False
        )

        # # Spin the node in background thread(s)
        executor = rclpy.executors.MultiThreadedExecutor(2)
        executor.add_node(self)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()


        self.gripper_attach = self.create_client(AttachLink, '/GripperMagnetON')
        self.gripper_detach = self.create_client(DetachLink, '/GripperMagnetOFF')
        self.servo_control = self.create_client(ServoLink, '/SERVOLINK')

        while not self.servo_control.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servo service not available, waiting again...')

    
        box1, box2 = 1, 2
        box1, box2 = str(box1), str(box2)
        
        t_12_base = self.tf_buffer.lookup_transform( 'base_link', 'obj_12' , rclpy.time.Time()  , rclpy.duration.Duration(seconds=1))
        t_home_base = self.tf_buffer.lookup_transform( 'base_link', 'wrist_3_link' , rclpy.time.Time()  , rclpy.duration.Duration(seconds=1))

        t_4_base = self.tf_buffer.lookup_transform( 'base_link', f'obj_{box1}' , rclpy.time.Time()  , rclpy.duration.Duration(seconds=1))
        t_5_base = self.tf_buffer.lookup_transform( 'base_link', f'obj_{box2}' , rclpy.time.Time()  , rclpy.duration.Duration(seconds=1))

        home_fwd = [0.0, -2.391, 2.40855, -np.pi, -np.pi/2, np.pi]
        home_rig = [-np.pi/2, -2.391, 2.40855, -np.pi, -np.pi/2, np.pi]
        home_lef = [np.pi/2, -2.391, 2.40855, -np.pi, -np.pi/2, np.pi]
        # print(t_12_base)
        # print(t_home_base)
        # print(t_4_base)
        # print(t_5_base)
        filepath = './src/pymoveit2/examples/assets/col_obj.stl'
        mesh_id = 'box'
        position = [0.2, -1.5, 0]
        quat_xyzw = [-0.5, -0.5, 0.5, 0.5]
        moveit2.add_collision_mesh(
            filepath=filepath, id=mesh_id, position=position, quat_xyzw=quat_xyzw, frame_id=ur5.base_link_name()
        )

        moveit2.move_to_configuration(home_rig, cartesian=False)
        moveit2.wait_until_executed()
        self.go_to_tf(t_4_base.transform.translation, [-0.707,0.707,0,0], moveit2, 0.02)
        self.attach(f'box{box1}',moveit2)
        moveit2.move_to_configuration(home_rig, cartesian=False)
        moveit2.wait_until_executed()
        moveit2.move_to_configuration(home_fwd, cartesian=False)
        moveit2.wait_until_executed()
        self.go_to_tf(t_12_base.transform.translation, [0.707,0.707,0,0], moveit2, 0.25)
        self.detatch(f'box{box1}',moveit2)
        self.servolink(f'box{box1}')
        moveit2.move_to_configuration(home_fwd, cartesian=False)
        moveit2.wait_until_executed()

        moveit2.move_to_configuration(home_lef, cartesian=False)
        moveit2.wait_until_executed()
        self.go_to_tf(t_5_base.transform.translation, [0.707,-0.707,0,0], moveit2, 0.02)
        self.attach(f'box{box2}',moveit2)
        moveit2.move_to_configuration(home_lef, cartesian=False)
        moveit2.wait_until_executed()
        moveit2.move_to_configuration(home_fwd, cartesian=False)
        moveit2.wait_until_executed()
        
        # self.go_to_tf(t_home_base.transform.translation, t_home_base.transform.rotation, moveit2)
        self.go_to_tf(t_12_base.transform.translation, [0.707,0.707,0,0], moveit2, 0.25)
        self.detatch(f'box{box2}',moveit2)
        self.servolink(f'box{box2}')
        moveit2.move_to_configuration(home_fwd, cartesian=False)
        moveit2.wait_until_executed()
        


        rclpy.shutdown()
        exit(0)

    def go_to_tf(self, trans,quat, moveit2, offset = 0):
        
        position = np.zeros((3,1))
        position[0] = trans.x
        position[1] = trans.y
        position[2] = trans.z + offset

        # quat = np.zeros((4,1))
        # quat[0] = rot.x
        # quat[1] = rot.y
        # quat[2] = rot.z
        # quat[3] = rot.w   


        print(position)
        print(quat)

        moveit2.move_to_pose(position=position, quat_xyzw=quat, cartesian=False)
        moveit2.wait_until_executed()

    def attach(self,box,moveit2):
        while not self.gripper_attach.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('EEF service not available, waiting again...')

        req = AttachLink.Request()
        req.model1_name =  box    
        req.link1_name  = 'link'       
        req.model2_name =  'ur5'       
        req.link2_name  = 'wrist_3_link'  

        self.gripper_attach.call_async(req)
        # filepath = './src/pymoveit2/examples/assets/Pcoj.stl'
        # moveit2.add_collision_mesh(
        #     filepath=filepath, id='box1', position=[0.1,0.1,0.01], quat_xyzw=[0,0,1,0], frame_id=ur5.end_effector_name()
        # )

        # print("Attached*****************************")

    def detatch(self, box,moveit2):
        while not self.gripper_detach.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('EEF service not available, waiting again...')

        req = DetachLink.Request()
        req.model1_name =  box   
        req.link1_name  = 'link'       
        req.model2_name =  'ur5'       
        req.link2_name  = 'wrist_3_link'  

        self.gripper_detach.call_async(req)
        # moveit2.remove_collision_mesh(id='box1' )

        # print("Detached*****************************")

    def servolink(self,box):
        req = ServoLink.Request()
        req.box_name =  box      
        req.box_link  = 'link'       
        self.servo_control.call_async(req)

def main():
    '''
    Description:    Main function which creates a ROS node and spin around for the aruco_tf class to perform it's task
    '''

    rclpy.init(args=sys.argv)                                       # initialisation

    node = rclpy.create_node('moveit_process')                    # creating ROS node

    node.get_logger().info('Node created: moveit process')        # logging information

    servoing_tf_class = servoing_tf()                                     # creating a new object for class 'servoing_tf'

    rclpy.spin(servoing_tf_class)                                      # spining on the object to make it alive in ROS 2 DDS

    servoing_tf_class.destroy_node()                                   # destroy node after spin ends

    rclpy.shutdown()                                                # shutdown process

if __name__ == "__main__":
    main()
