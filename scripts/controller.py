#!/usr/bin/env python
import rospy
import rospkg
from std_msgs.msg import Float32, Int32
import numpy as np
import pickle

class JointCmds:
    def __init__(self, joints, path):
        self.jnt_cmd_dict = {}
        self.joints_list = joints
        self.t = 0.0
        self.path = path + '/data/'

        self.osl_knee_enc = 0.0
        self.osl_ankle_enc = 0.0

        rospy.Subscriber('/oslsim/osl_knee/encoder', Int32, self.osl_knee_enc_cb)
        rospy.Subscriber('/oslsim/osl_ankle/encoder', Int32, self.osl_ankle_enc_cb)

    def osl_knee_enc_cb(self, data):
        self.osl_knee_enc = data

    def osl_ankle_enc_cb(self, data):
        self.osl_ankle_enc = data

    def update(self, dt):
        with open(self.path + 'angles.pkl', 'rb') as f:
            angles = pickle.load(f)

        # -------------------------------------- #

        self.jnt_cmd_dict['osl_ankle'] = 0.1 * np.sin(self.t)
        self.jnt_cmd_dict['osl_knee'] = 0.1 * np.sin(self.t)

        # -------------------------------------- #

        self.t += dt
        return self.jnt_cmd_dict

class Controller:
    def __init__(self, joints, hz):
        self.joints = joints
        self.hz = hz

    def run(self):
        rospack = rospkg.RosPack()
        cwd = rospack.get_path('oslsim')
        pub={}
        ns_str = '/oslsim/'
        for j in self.joints:
            pub[j] = rospy.Publisher(ns_str + j + '/command', Float32, queue_size=10)

        rospy.init_node('oslsim_controller', anonymous=True)

        rate = rospy.Rate(self.hz)
        jntcmds = JointCmds(joints=self.joints, path=cwd)
        
        while not rospy.is_shutdown():
            jnt_cmd_dict = jntcmds.update(0.1)
            for jnt in jnt_cmd_dict.keys() :
                pub[jnt].publish(jnt_cmd_dict[jnt])
            rate.sleep()

if __name__ == "__main__":
    try:
        joints = ['osl_knee', 'osl_ankle']
        pid = Controller(joints=joints, hz=50)
        pid.run()

    except rospy.ROSInterruptException:
        pass
