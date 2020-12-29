#!/usr/bin/env python
import rospy
import rospkg
from std_msgs.msg import Float32, Int32
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import pickle

class JointCmds:
    def __init__(self, joints, path):
        self.jnt_cmd_dict = {}
        self.joints_list = joints
        self.t = 0.0
        self.path = path + '/data/'

        self.osl_knee_pose = 0
        self.osl_ankle_pose = 0

        self.setpoint_knee = 0.0
        self.error_knee = 0.0
        self.errord_knee = 0.0
        self.errori_knee = 0.0
        self.preverror_knee = 0.0

        self.setpoint_ankle = 0.0
        self.error_ankle = 0.0
        self.errord_ankle = 0.0
        self.errori_ankle = 0.0
        self.preverror_ankle = 0.0

        rospy.Subscriber('/oslsim/imu/osl_shank', Imu, self.osl_knee_pose_cb)
        rospy.Subscriber('/oslsim/imu/osl_foot', Imu, self.osl_ankle_pose_cb)

    def osl_knee_pose_cb(self, data):
        temp = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(temp)
        self.osl_knee_pose = -1.0 * pitch

    def osl_ankle_pose_cb(self, data):
        temp = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(temp)
        self.osl_ankle_pose = -1.0 * pitch

    def update(self, dt):
        with open(self.path + 'angles.pkl', 'rb') as f:
            angles = pickle.load(f)

        # -------------------------------------- #
       
        self.setpoint_knee = -0.0174533 * angles['angle_knee'][self.t%100]
        self.setpoint_ankle = -0.0174533 * angles['angle_ankle'][self.t%100]

        kp_knee = 0.8
        ki_knee = 0.1
        kd_knee = 0.02

        kp_ankle = 1.0
        ki_ankle = 0.0
        kd_ankle = 0.0

        self.error_knee = self.setpoint_knee - self.osl_knee_pose
        self.errord_knee = self.error_knee - self.preverror_knee
        self.errori_knee += self.error_knee

        self.error_ankle = self.setpoint_ankle - self.osl_ankle_pose
        self.errord_ankle = self.error_ankle - self.preverror_ankle
        self.errori_ankle += self.error_ankle

        # -------------------------------------- #

        self.jnt_cmd_dict['osl_ankle'] = (kp_ankle*self.error_ankle) + (kd_ankle*self.errord_ankle) + (ki_ankle*self.errori_ankle)
        self.jnt_cmd_dict['osl_knee'] = (kp_knee*self.error_knee) + (kd_knee*self.errord_knee) + (ki_knee*self.errori_knee)

        # -------------------------------------- #

        self.preverror_knee = self.error_knee
        self.preverror_ankle = self.error_ankle
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
            jnt_cmd_dict = jntcmds.update(1)
            for jnt in jnt_cmd_dict.keys() :
                pub[jnt].publish(jnt_cmd_dict[jnt])
            rate.sleep()

if __name__ == "__main__":
    try:
        joints = ['osl_knee', 'osl_ankle']
        pid = Controller(joints=joints, hz=10)
        pid.run()

    except rospy.ROSInterruptException:
        pass
