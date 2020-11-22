#!/usr/bin/env python
import rospy
import numpy as np
import time
from copy import deepcopy
from sensor_msgs.msg import Joy, JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from rdda_interface.srv import SetStiffness
from std_msgs.msg import Float64

def set_stiffness(val):
    rospy.wait_for_service('/rdda_interface/set_stiff')
    try:
        set_stiff = rospy.ServiceProxy('/rdda_interface/set_stiff', SetStiffness)
        result = set_stiff([val, val])
        print('Setting Stiffness : {}, with error code: {}'.format(val,result))
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)



class xbox_control():
    joint_positions = np.array([0.0,0.0])
    is_closed = True

    def __init__(self):
        self.closed_position = None
        self.stiffness_list = [0.0, 0.3, 1.0, 10]
        self.stiffness_index = 3
        self.stiffness = self.stiffness_list[self.stiffness_index]
        self.last_updated_stiffness = time.time()


        rospy.init_node('joy_listener', anonymous=True)
        rospy.Subscriber("joy", Joy, self.joy_callback)
        rospy.Subscriber("/rdda_interface/joint_states", JointState, self.joints_callback)
        self.commander = rospy.Publisher("/rdda_interface/joint_cmds", JointTrajectoryPoint)
        self.traj_message = JointTrajectoryPoint()
        print('Waiting for to recieve initial joint states')
        while True:
            # print(self.closed_position)
            if not self.closed_position is None:
                time.sleep(0.2)
                break
        self.open_position = deepcopy(self.closed_position) - 0.6
        # self.traj_message.positions = self.open_position
        # self.commander.publish(self.traj_message)
        # print('Opening?')
        time.sleep(0.5)
        print('Initialized')
        rospy.on_shutdown(self.close_gripper)

        set_stiffness(self.stiffness)

    def close_gripper(self):
        print('Close Gripper')
        self.traj_message.positions = self.closed_position
        self.commander.publish(self.traj_message)

    def joy_callback(self, data):
        # print(data.axes)
        if data.axes[2]<0:
            print('Close Gripper')
            self.traj_message.positions = self.closed_position
            self.commander.publish(self.traj_message)
            # time.sleep(0.1)
        else:
            print('Open Gripper')
            self.traj_message.positions = self.open_position
            self.commander.publish(self.traj_message)
            # time.sleep(0.1)

        if time.time()-self.last_updated_stiffness > 0.5:
            self.last_updated_stiffness = time.time()
            if data.axes[7] > 0.5:
                #increase stiffness
                if self.stiffness_index < len(self.stiffness_list)-1:
                    print('Increasing stiffness')
                    self.stiffness_index += 1;
                    set_stiffness(self.stiffness_list[self.stiffness_index])
                else:
                    print('Stiffness Max = {}'.format(self.stiffness_list[self.stiffness_index]))

            elif data.axes[7]< -0.5:
                if self.stiffness_index > 0:
                    print('Decreasing Stiffness')
                    self.stiffness_index -= 1;
                    set_stiffness(self.stiffness_list[self.stiffness_index])
                else:
                    print('Stiffness Min = {}'.format(self.stiffness_list[self.stiffness_index]))


    def joints_callback(self, data):
        self.joint_positions = np.array(data.position)
        if self.closed_position is None:
            # print('?')
            self.closed_position = np.array(data.position)
            print(self.closed_position)
        # print('Got Joint States : {}'.format(self.joint_positions))


if __name__=='__main__':
    controller = xbox_control()
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
