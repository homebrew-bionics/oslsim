#!/usr/bin/env python
import rospy
import rospkg
from std_msgs.msg import Float64
import numpy as np
import pickle

class JointCmds:
    def __init__(self, joints, path):
        self.jnt_cmd_dict = {}
        self.joints_list = joints
        self.t = 0.0
        self.path = path + '/data/'

    def update(self, dt):
        sign = 1.0

        if self.t%100 > 50:
            sign = -1.0

        with open(self.path + 'angles.pkl', 'rb') as f:
            angles = pickle.load(f)

        # -------------------------------------- #

        self.jnt_cmd_dict['osl_ankle'] = 0.0174533 * angles['angle_ankle'][self.t%100]
        self.jnt_cmd_dict['osl_hip'] = 0.0174533 * angles['angle_thigh'][self.t%100]
        self.jnt_cmd_dict['osl_knee'] = -0.0174533 * angles['angle_knee'][self.t%100]

        self.jnt_cmd_dict['ankle'] = -0.0174533 * (angles['angle_ankle'][abs(50 + sign * (self.t%100))])
        self.jnt_cmd_dict['hip'] = 0.0174533 * (angles['angle_thigh'][abs(50 + sign * (self.t%100))])
        self.jnt_cmd_dict['knee'] = -0.0174533 * (angles['angle_knee'][abs(50 + sign * (self.t%100))])

        # -------------------------------------- #

        self.t += dt
        return self.jnt_cmd_dict


def publish_commands(joints, hz):
    rospack = rospkg.RosPack()
    cwd = rospack.get_path('oslsim')
    pub={}
    ns_str = '/oslsim/'
    cont_str = '_position_controller'
    for j in joints:
        pub[j] = rospy.Publisher(ns_str + j + cont_str + '/command', Float64, queue_size=10)

    rospy.init_node('oslsim_walker', anonymous=True)
    rate = rospy.Rate(hz)
    jntcmds = JointCmds(joints=joints, path=cwd)
    while not rospy.is_shutdown():
        jnt_cmd_dict = jntcmds.update(1)
        for jnt in jnt_cmd_dict.keys() :
            pub[jnt].publish(jnt_cmd_dict[jnt])
        rate.sleep()

if __name__ == "__main__":
    try:
        joints = ['hip', 'knee', 'ankle', 'osl_hip', 'osl_knee', 'osl_ankle']
        hz = 10
        publish_commands(joints, hz)
    except rospy.ROSInterruptException:
        pass
