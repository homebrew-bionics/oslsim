#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
import numpy as np

class JointCmds:
    def __init__(self, joints):
        self.jnt_cmd_dict = {}
        self.joints_list = joints
        self.t = 0.0

    def update(self, dt):
        self.t += dt

        # A sinusoidal waveform as a placeholder #
        # -------------------------------------- #

        a = 0.3*np.pi
        b = 2*np.pi
        c = 0*np.pi

        num_segments = 3
        gamma=-c/num_segments
        beta=b/num_segments
        alpha=a*np.abs(np.sin(beta/2))

        for i, jnt in enumerate(self.joints_list):
            self.jnt_cmd_dict[jnt] = alpha*np.sin(2*self.t*np.pi+(i%3)*beta)+gamma

        # -------------------------------------- #

        return self.jnt_cmd_dict


def publish_commands(joints, hz):
    pub={}
    ns_str = '/oslsim/'
    cont_str = '_position_controller'
    for j in joints:
        pub[j] = rospy.Publisher(ns_str + j + cont_str + '/command', Float64, queue_size=10 )

    rospy.init_node('oslsim_walker', anonymous=True)
    rate = rospy.Rate(hz)
    jntcmds = JointCmds(joints=joints)
    while not rospy.is_shutdown():
        jnt_cmd_dict = jntcmds.update(1./hz)
        for jnt in jnt_cmd_dict.keys() :
            pub[jnt].publish(jnt_cmd_dict[jnt])
        rate.sleep()

if __name__ == "__main__":
    try:
        joints = ['hipL', 'kneeL', 'ankleL', 'hipR', 'kneeR', 'ankleR']
        hz = 50
        publish_commands(joints, hz)
    except rospy.ROSInterruptException:
        pass
