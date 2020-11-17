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
        ampT=20*np.pi*(1./180)
        ampS=35*np.pi*(1./180)
        offsetS=-25*np.pi*(1./180)
        phaseLS=-np.pi/2
        phaseRS=-np.pi/2-1.12
        k=4
        self.jnt_cmd_dict['ankleL']=0.0
        self.jnt_cmd_dict['ankleR']=0.0
        self.jnt_cmd_dict['hipR']=ampT*np.sin(self.t*k)
        self.jnt_cmd_dict['hipL']=-ampT*np.sin(self.t*k)
        self.jnt_cmd_dict['kneeL']=ampS*np.sin((self.t*k)+phaseLS)+offsetS
        self.jnt_cmd_dict['kneeR']=ampS*np.sin((self.t*k)+phaseRS)+offsetS

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
