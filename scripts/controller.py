#!/usr/bin/env python
import rospy
import rospkg
from std_msgs.msg import Float32
import numpy as np
import pickle

class JointCmds:
    def __init__(self, joints, path):
        self.jnt_cmd_dict = {}
        self.joints_list = joints
        self.t = 0.0
        self.path = path + '/data/'

    def update(self, dt):
        with open(self.path + 'angles.pkl', 'rb') as f:
            angles = pickle.load(f)

        # -------------------------------------- #

        self.jnt_cmd_dict['osl_ankle'] = np.sin(self.t)
        self.jnt_cmd_dict['osl_knee'] = np.sin(self.t)

        # -------------------------------------- #

        self.t += dt
        return self.jnt_cmd_dict


def publish_commands(joints, hz):
    rospack = rospkg.RosPack()
    cwd = rospack.get_path('oslsim')
    pub={}
    for j in joints:
        pub[j] = rospy.Publisher('/' + j + '/command', Float32, queue_size=10)

    rospy.init_node('oslsim_controller', anonymous=True)
    rate = rospy.Rate(hz)
    jntcmds = JointCmds(joints=joints, path=cwd)
    while not rospy.is_shutdown():
        jnt_cmd_dict = jntcmds.update(1)
        for jnt in jnt_cmd_dict.keys() :
            pub[jnt].publish(jnt_cmd_dict[jnt])
        rate.sleep()

if __name__ == "__main__":
    try:
        joints = ['osl_knee', 'osl_ankle']
        hz = 10
        publish_commands(joints, hz)
    except rospy.ROSInterruptException:
        pass
