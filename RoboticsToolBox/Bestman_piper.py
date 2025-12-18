import numpy as np
import time
import ikpy
from ikpy.chain import Chain
from ikpy.inverse_kinematics import inverse_kinematic_optimization
from scipy.spatial.transform import Rotation as R
import sys
import os

from piper_sdk import *


current_dir = os.path.dirname(os.path.abspath(__file__))

#### https://github.com/agilexrobotics/piper_sdk/blob/0.1.13/asserts/V2/INTERFACE_V2.MD
class Bestman_piper:
    # ----------------------------------------------------------------
    # Functions for initalization
    # ----------------------------------------------------------------

    def __init__(self, robot_name="can0", frequency=30):
        # Initialize the robot and gripper with the provided IPs and frequency
        self.robot = C_PiperInterface_V2(robot_name)
        self.robot.ConnectPort()
        local_ip = None
        # self.mode = self.robot.set_mode(0) # 0: default
        # self.robot_states = self.robot.set_state(0)
        self.first_init_flag = True
        self.gripper = True # have gripper by default
        self.frequency = frequency
        self.robot.EnableArm(7)
        self.enable_fun(piper=self.robot)



        # # URDF
        # urdf_file = os.path.join(current_dir, "../Asset/xarm6_robot.urdf")
        # self.robot_chain = Chain.from_urdf_file(urdf_file)
        # self.active_joints = [
        #     joint for joint in self.robot_chain.links 
        #     if isinstance(joint, ikpy.link.URDFLink) and (joint.joint_type == 'revolute' or joint.joint_type == 'prismatic')
        # ]

    # ----------------------------------------------------------------
    # Functions for basic control
    # ----------------------------------------------------------------
    # def clear_fault(self):
    #     '''Clear fault, and updates the current robot states.'''
    #     self.robot.set_state(0)

    # def update_robot_states(self):
    #     '''Updates the current robot states.'''
    #     self.robot.getRobotStates(self.robot_states)

    # def set_mode(self, _mode):
    #     '''
    #     Parameters:
    #     _mode=
    #         0: position controlmode
    #         1: servo motionmode
    #         2: joint teachingmodemode (invalid)3: cartesian teaching
    #         4: joint velocity control mode
    #         5: cartesian velocity control mode
    #         6: joint online trajectory planningmode
    #         7: cartesian online trajectory planning

    #     Notes:
    #         https://github.com/xArm-Developer/xArm-Python-SDK/blob/master/doc/api/xarm_api.md#mode
    #     '''
    #     print("set mode")
    #     self.robot.set_mode(_mode)

    def enable_fun(self,piper:C_PiperInterface_V2):
        '''
        使能机械臂并检测使能状态,尝试5s,如果使能超时则退出程序
        '''
        enable_flag = False
        # 设置超时时间（秒）
        timeout = 5
        # 记录进入循环前的时间
        start_time = time.time()
        elapsed_time_flag = False
        while not (enable_flag):
            elapsed_time = time.time() - start_time
            print("--------------------")
            enable_flag = piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status and \
                piper.GetArmLowSpdInfoMsgs().motor_2.foc_status.driver_enable_status and \
                piper.GetArmLowSpdInfoMsgs().motor_3.foc_status.driver_enable_status and \
                piper.GetArmLowSpdInfoMsgs().motor_4.foc_status.driver_enable_status and \
                piper.GetArmLowSpdInfoMsgs().motor_5.foc_status.driver_enable_status and \
                piper.GetArmLowSpdInfoMsgs().motor_6.foc_status.driver_enable_status
            print("使能状态:",enable_flag)
            piper.EnableArm(7)
            piper.GripperCtrl(0,1000,0x01, 0)
            print("--------------------")
            # 检查是否超过超时时间
            if elapsed_time > timeout:
                print("超时....")
                elapsed_time_flag = True
                enable_flag = True
                break
            time.sleep(1)
            pass
        if(elapsed_time_flag):
            print("程序自动使能超时,退出程序")
            exit(0)



    def reset_to_home(self,dist=0):
        '''Move arm to initial pose.'''
        ####                                speed
        self.robot.MotionCtrl_2(0x01, 0x01, 30, 0x00)
        self.robot.JointCtrl(0, 0, 0, 0, 0, 0)#### angle(degree) *1000
        # time.sleep(3)

    def pose_to_euler(self, pose):
        '''
        Convert robot pose from a list [x, y, z, qw, qx, qy, qz] to [x, y, z] and Euler angles.
        
        Parameters:
        pose: list of 7 floats - [x, y, z, qw, qx, qy, qz]
        
        Returns:
        tuple: (x, y, z, roll, pitch, yaw) where (x, y, z) is the position and (roll, pitch, yaw) are the Euler angles in radians.
        '''
        x, y, z, qw, qx, qy, qz = pose
        r = R.from_quat([qx, qy, qz, qw])  # Reordering to match scipy's [qx, qy, qz, qw]
        roll, pitch, yaw = r.as_euler('xyz', degrees=False)
        return [x, y, z, roll, pitch, yaw]

    def euler_to_pose(self, position_euler):
        '''
        Convert robot pose from [x, y, z, roll, pitch, yaw] to [x, y, z, qw, qx, qy, qz].
        
        Parameters:
        position_euler: list of 6 floats - [x, y, z, roll, pitch, yaw]
        
        Returns:
        list: [x, y, z, qw, qx, qy, qz]
        '''
        x, y, z, roll, pitch, yaw = position_euler
        r = R.from_euler('xyz', [roll, pitch, yaw], degrees=False)
        qx, qy, qz, qw = r.as_quat()  # Getting [qx, qy, qz, qw] from scipy
        return [x, y, z, qw, qx, qy, qz]  # Reordering to match [qw, qx, qy, qz]
    
    # def set_payload(self,payload_weight, payload_cog):

    #     self.robot.set_tcp_load(payload_weight, payload_cog)
    

    # ----------------------------------------------------------------
    # Functions for robot parameters acquisition
    # ----------------------------------------------------------------

    def get_joint_bounds(self):###### here!
        '''
        Retrieves the joint bounds of the robot arm.

        Returns:
            list: A list of tuples representing the joint bounds, where each tuple contains the minimum and maximum values for a joint.
        '''
        maxbounds = self.robot.info().qMax
        minbounds = self.robot.info().qMin
        jointbounds = list(zip(maxbounds,minbounds))
        return jointbounds
    

    def get_dof(self):
        '''
        Retrieves the degree of freedom (DOF) of the robot arm.

        Returns:
            int: The degree of freedom of the robot arm.
        '''
        return 6

    def get_joint_idx(self):
        '''
        Retrieves the indices of the joints in the robot arm.

        Returns:
            list: A list of indices for the joints in the robot arm.
        '''
        return list(range(len(self.active_joints)))
    
    def get_links_info(self):
        '''
        Retrieves the TCP (Tool Center Point) link of the robot arm.

        Returns:
            str: The TCP link of the robot arm.
        '''
        return self.robot_chain.links[6].name

    def get_current_joint_angles(self,is_radian=True):
        '''
        Retrieves the current joint angles of the robot arm.

        Returns:
            list: A list of the current joint angles of the robot arm.
        '''
        if is_radian:
            factor = 1000 *57.2958
        else:
            factor = 1000
        _joint_states = self.robot.GetArmJointMsgs()
        _joint_angles = [0,0,0,0,0,0]
        _joint_angles[0] = _joint_states.joint_state.joint_1/factor #### result as degree
        _joint_angles[1] = _joint_states.joint_state.joint_2/factor #### result as degree
        _joint_angles[2] = _joint_states.joint_state.joint_3/factor #### result as degree
        _joint_angles[3] = _joint_states.joint_state.joint_4/factor #### result as degree
        _joint_angles[4] = _joint_states.joint_state.joint_5/factor #### result as degree
        _joint_angles[5] = _joint_states.joint_state.joint_6/factor #### result as degree
        return _joint_angles
    
    # def get_joint_vel(self):
    #     '''
    #     Retrieves the current joint velocities of the robot arm.

    #     Returns:
    #         list: A list of the current joint velocities of the robot arm.
    #     '''

    #     _joint_states = self.robot.get_joint_states(is_radian=True)
    #     _joint_velocities = _joint_states[1][1][0:6]

    #     return _joint_velocities

    def get_current_eef_pose(self,if_rad=True):
        '''
        Retrieves the current pose of the robot arm's end effector.

        This function obtains the position and orientation of the end effector.

        Returns:
            pose: the [x, y, z, roll, pitch, yaw] value of tcp in mm and radian
        '''
        x = self.robot.GetArmEndPoseMsgs().end_pose.X_axis/1000
        y = self.robot.GetArmEndPoseMsgs().end_pose.X_axis/1000
        z = self.robot.GetArmEndPoseMsgs().end_pose.X_axis/1000
        roll = self.robot.GetArmEndPoseMsgs().end_pose.RX_axis/1000
        pitch = self.robot.GetArmEndPoseMsgs().end_pose.RY_axis/1000
        yaw = self.robot.GetArmEndPoseMsgs().end_pose.RZ_axis/1000

        if if_rad:
            pose = [x, y, z, roll* np.pi / 180, pitch* np.pi / 180, yaw* np.pi / 180]
        else:
            pose = [x, y, z, roll, pitch, yaw]
        return pose   

    def get_eef_vel(self):##  here!
        '''
        Retrieves the current tcp velocities of the robot arm.

        Returns:
            list: A list of the current tcp velocities of the robot arm.
        '''
        speed = self.robot.realtime_tcp_speed
        return speed

    
    # ----------------------------------------------------------------
    # Functions for joint control
    # ----------------------------------------------------------------

    def print_links_info(self, name):
        '''
        Prints the joint and link information of a robot.

        Args:
            name (str): 'base' or 'arm'
        '''
        if name == 'base':
            print("Base joint and link information:")
            for i, link in enumerate(self.robot_chain.links[:1]):  # Assuming the base is the first link
                print(f"Link {i}: {link.name}")
        elif name == 'arm':
            print("Arm joint and link information:")
            for i, link in enumerate(self.robot_chain.links[1:]):  # Assuming the arm starts from the second link
                print(f"Link {i + 1}: {link.name}")

    def move_arm_to_joint_values(self, joint_angles, target_vel=None, target_acc=None, MAX_VEL=20, MAX_ACC=None, wait_for_finish=None):
        '''
        Move arm to a specific set of joint angles, considering physics.

        Args:
            joint_angles: unit in radians
            MAX_VEL :  unit in %
            wait_for_finish=None
        Note:
        '''
        ######################### rad2deg ## piper cmd need degree*1000
        _joint_cmd = [round(angle*57.2958*1000) for angle in joint_angles]
        ####                                speed
        self.robot.MotionCtrl_2(0x01, 0x01, MAX_VEL, 0x00)
        self.robot.JointCtrl(_joint_cmd)#### angle(degree) *1000
        _if_finished = False
        if wait_for_finish:### 待完善，缺少若机械臂出现始终没有到位置怎么办？
            while _if_finished:
                cur_joints = self.get_current_joint_angles()
                for i in range(6):    
                    if (cur_joints[i] > joint_angles[i]+0.1) or (cur_joints[i] < joint_angles[i]-0.1):
                        time.sleep(0.05)
                        break
                else:
                    print("move done")
                    return True


    def move_arm_to_joint_values_traj(self, target_trajectory, target_vel=None, target_acc=None, MAX_VEL=20, MAX_ACC=None):
        '''
        Move arm to a few set of joint angles, considering physics.

        Args:
            target_trajectory: A list of desired joint angles (in radians) for each joint of the arm.
            ##target_vel: Optional. A list of target velocities for each joint.
            ##target_acc: Optional. A list of target accelerations for each joint.
            MAX_VEL: Optional. A list of maximum velocities for each joint.
            ##MAX_ACC: Optional. A list of maximum accelerations for each joint.
        '''
        period = 1.0 / self.frequency
        # self.update_robot_states()
        # DOF = len(self.robot_states.q)

        for target_pos in target_trajectory:
            # Monitor fault on robot server
            # if self.robot.isFault():
            #     raise Exception("Fault occurred on robot server, exiting ...")

            # Send command
            self.set_joint_cmd(target_pos,MAX_VEL= MAX_VEL,wait_for_finish=False )
            print(f"Sent joint positions: {target_pos}")

            # Use sleep to control loop period
            time.sleep(period)

    # ----------------------------------------------------------------
    # Functions for eef control
    # ----------------------------------------------------------------

    # TODO

    # def set_eef_vel(self, _velocity_setpoint, _duration):####
    #     '''
    #     Move arm's end effector to a target tcp velocity.

    #     Args:
    #         _velocity_setpoint: mm/s in transition, rad/s in orientation
    #         _duration: function calling loop time
    #     Notes:
    #         https://github.com/xArm-Developer/xArm-Python-SDK/blob/master/doc/api/xarm_api.md#def-vc_set_cartesian_velocityself-speeds-is_radiannone-is_tool_coordfalse-duration-1-kwargs
    #     '''

    #     self.robot.set_mode(5) # 5 for cartesian vel
    #     self.robot.set_state(0)
    #     self.robot.vc_set_cartesian_velocity(speeds=[_velocity_setpoint[0],
    #                                                 _velocity_setpoint[1],
    #                                                 _velocity_setpoint[2],
    #                                                 _velocity_setpoint[3],
    #                                                 _velocity_setpoint[4],
    #                                                 _velocity_setpoint[5]],
    #                                                 duration=_duration)

    def move_eef_to_goal_pose(self, end_effector_goal_pose, speed=20, mvacc=50000, wait=False):
        '''
        Move arm's end effector to a target position.

        Args:
            end_effector_goal_pose (Pose): position in mm and euler_orientation in radian for orientaiton.
            speed : unit in %
        '''
        factor = 1000
        X = round(end_effector_goal_pose[0]*factor)
        Y = round(end_effector_goal_pose[1]*factor)
        Z = round(end_effector_goal_pose[2]*factor)## rad 2 deg
        RX = round(end_effector_goal_pose[3]*factor*57.2958)
        RY = round(end_effector_goal_pose[4]*factor*57.2958)
        RZ = round(end_effector_goal_pose[5]*factor*57.2958)
        print(X,Y,Z,RX,RY,RZ)
        # piper.MotionCtrl_1()
        self.robot.MotionCtrl_2(0x01, 0x00, speed, 0x00)
        self.robot.EndPoseCtrl(X,Y,Z,RX,RY,RZ)


        _if_finished = False
        if wait:### 待完善，缺少若机械臂出现始终没有到位置怎么办？
            while _if_finished:
                cur_eef_pose = self.get_current_eef_pose()
                for i in range(6):
                    if i < 3:##### position in mm
                        if (cur_eef_pose[i] > end_effector_goal_pose[i]+10) or (cur_eef_pose[i] < end_effector_goal_pose[i]-10):
                            time.sleep(0.05)
                            break
                    else:  ##### orientaiton in rad
                        if (cur_eef_pose[i] > end_effector_goal_pose[i]+0.1) or (cur_eef_pose[i] < end_effector_goal_pose[i]-0.1):
                            time.sleep(0.05)
                            break
                else:
                    print("move done")
                    return True


    def rotate_eef_joint(self, angle,MAX_VEL = 20,wait_for_finish=True):### 
        '''
        Rotate the end effector of the robot arm by a specified angle by joint.

        Args:
            angle (float): The desired rotation angle in radians.
        '''
        current_joint_angles = self.get_current_joint_angles()
        
        target_joint_angles = current_joint_angles.copy()
        target_joint_angles[6] += angle 

        self.move_arm_to_joint_values(target_joint_angles,wait_for_finish=wait_for_finish)

    # ----------------------------------------------------------------
    # Functions for IK/FK
    # ----------------------------------------------------------------

    def forward_kinematics(self, joint_angles):
        '''
        Transforms the robot arm's joint angles to its Cartesian coordinates.

        Args:
            joint_angles (list): A list of joint angles for the robot arm.

        Returns:
            tuple: A tuple containing the Cartesian coordinates (position and orientation) of the robot arm.
        '''
        # Validate the number of joint values matches the number of active joints
        if len(joint_angles) != len(self.active_joints):
            raise ValueError("The number of joint values does not match the number of active joints")
        
        # Map joint values to the full joint chain
        full_joint_angles = np.zeros(len(self.robot_chain.links))
        active_joint_indices = [self.robot_chain.links.index(joint) for joint in self.active_joints]

        for i, joint_value in enumerate(joint_angles):
            full_joint_angles[active_joint_indices[i]] = joint_value

        # Calculate the end effector position and orientation
        cartesian_matrix = self.robot_chain.forward_kinematics(full_joint_angles)

        # Extract position and orientation
        position = cartesian_matrix[:3, 3]
        orientation_matrix = cartesian_matrix[:3, :3]

        # Convert rotation matrix to quaternion
        r = R.from_matrix(orientation_matrix)
        quaternion = r.as_quat()
        orientation = [quaternion[3], quaternion[0], quaternion[1], quaternion[2]]

        return position, orientation
    
    def inverse_kinematics(self, position, orientation):
        '''
        Transforms the robot arm's Cartesian coordinates to its joint angles.

        Args:
            position (list): The Cartesian position of the robot arm.
            orientation (list): The Cartesian orientation of the robot arm.

        Returns:
            list: A list of joint angles corresponding to the given Cartesian coordinates.
        '''
        rotation_matrix = R.from_euler('xyz', orientation).as_matrix()

        # Combine rotation matrix and position into a list
        target_pose = np.eye(4)
        target_pose[:3, :3] = rotation_matrix
        target_pose[:3, 3] = position

        initial_joint_angles = [0] * len(self.robot_chain)

        # inverse kinematics calculations and return joint angles
        joint_angles = ikpy.inverse_kinematics.inverse_kinematic_optimization(
        chain=self.robot_chain,
        target_frame=target_pose,
        starting_nodes_angles=initial_joint_angles,
        orientation_mode='all',         
        )

        return joint_angles[1:8]


    def calc_ik_error(self, goal_position, goal_orientation):
        '''
        Calculate the inverse kinematics (IK) error for performing pick-and-place manipulation of an object using a robot arm.

        Args:
            goal_position: The desired goal position for the target object.
            goal_orientation: The desired goal orientation for the target object.
        '''
        pass

    # ----------------------------------------------------------------
    # Functions for gripper
    # ----------------------------------------------------------------
    
    ### xArm 
    def find_gripper(self):
        '''
        Searches for the gripper on available serial ports and returns the port if found.

        Returns:
            str: The serial port where the gripper is connected, or None if not found.
        '''
        gripper_status = self.robot.GetArmGripperMsgs()
        if gripper_status is not None:
            return True
        else:
            return False

    def get_gripper_pos(self):
        '''
        Get the position of the XArm gripper.
        '''
        gripper_status = self.robot.GetArmGripperMsgs()
        return gripper_status.gripper_state.grippers_angle[0]

    def gripper_goto(self, value, speed=5000, force=None):
        '''
        Moves the gripper to a specified position with given speed.

        Args:
            value (int): unit mm  limit to 66mm
            speed (int): Speed of the gripper movement, between 0 and 8000.
            force (int): Not applicable for xarm gripper
        
        Note:
            - 0 means fully closed.
            - 800 means fully open.
        '''
        self.robot.GripperCtrl(round(value),1000,0x01, 0)

    def open_gripper(self):
        ''' Opens the gripper to its maximum position with maximum speed and force. '''
        ####################### 66mm  1Nm
        self.robot.GripperCtrl(66000,1000,0x01, 0)

    def close_gripper(self):
        '''Closes the gripper to its minimum position with maximum speed and force.'''
        self.robot.GripperCtrl(000,1000,0x01, 0)
    
# ### Robotiq
    
#     def find_gripper_robotiq(self):
#         """
#         Config the parameter via Python SDK
#         """
#         # Baud rate
#         # Modify the baud rate to 115200, the default is 2000000.
#         self.robot.set_tgpio_modbus_baudrate(115200)  

#         # TCP Payload and offset
#         # Robotiq 2F/85 Gripper
#         self.robot.set_tcp_load(0.925, [0, 0, 58])
#         self.robot.set_tcp_offset([0, 0, 174, 0, 0, 0])
#         self.robot.save_conf()

#         # Self-Collision Prevention Model
#         # Robotiq 2F/85 Gripper
#         self.robot.set_collision_tool_model(4)

#         self.robot.robotiq_reset()
#         self.robot.robotiq_set_activate()    #enable the robotiq gripper
        


    

    def get_gripper_pos_robotiq(self, number_of_registers=3):
        """
        Reading the status of robotiq gripper
        
        :param number_of_registers: number of registers, 1/2/3, default is 3
            number_of_registers=1: reading the content of register 0x07D0
            number_of_registers=2: reading the content of register 0x07D0/0x07D1
            number_of_registers=3: reading the content of register 0x07D0/0x07D1/0x07D2
            
            Note: 
                register 0x07D0: Register GRIPPER STATUS
                register 0x07D1: Register FAULT STATUS and register POSITION REQUEST ECHO
                register 0x07D2: Register POSITION and register CURRENT
        :return: tuple((code, robotiq_response))
            code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.
            robotiq_response: See the robotiq documentation
        """
        status = self.robot.robotiq_get_status(number_of_registers=number_of_registers)
        gripper_width = status[1][-2]
        return gripper_width

    def set_gripper_pos_robotiq(self, pos, speed=0xFF, force=0xFF, wait=False, timeout=5, **kwargs):
        """
        Go to the position with determined speed and force.
        
        :param pos: position of the gripper. Integer between 0 and 255. 0 being the open position and 255 being the close position.
        :param speed: gripper speed between 0 and 255
        :param force: gripper force between 0 and 255
        :param wait: whether to wait for the robotion motion complete, default is True
        :param timeout: maximum waiting time(unit: second), default is 5, only available if wait=True
        
        :return: tuple((code, robotiq_response))
            code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.
            robotiq_response: See the robotiq documentation 
        """
        return self.robot.robotiq_set_position(pos, speed=speed, force=force, wait=wait, timeout=timeout, **kwargs)
    
    def open_gripper_robotiq(self, speed=0xFF, force=0xFF, wait=False, timeout=5, **kwargs):
        """
        Open the robotiq gripper
        
        :param speed: gripper speed between 0 and 255
        :param force: gripper force between 0 and 255
        :param wait: whether to wait for the robotiq motion to complete, default is True
        :param timeout: maximum waiting time(unit: second), default is 5, only available if wait=True
        
        :return: tuple((code, robotiq_response))
            code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.
            robotiq_response: See the robotiq documentation 
        """
        return self.robot.robotiq_open(speed=speed, force=force, wait=wait, timeout=timeout, **kwargs)

    def close_gripper_robotiq(self, speed=0xFF, force=0xFF, wait=False, timeout=5, **kwargs):
        """
        Close the robotiq gripper
        
        :param speed: gripper speed between 0 and 255
        :param force: gripper force between 0 and 255
        :param wait: whether to wait for the robotiq motion to complete, default is True
        :param timeout: maximum waiting time(unit: second), default is 3, only available if wait=True
        
        :return: tuple((code, robotiq_response))
            code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.
            robotiq_response: See the robotiq documentation
        """
        return self.robot.robotiq_close(speed=speed, force=force, wait=wait, timeout=timeout, **kwargs)
