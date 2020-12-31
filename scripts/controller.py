#!/usr/bin/env python
import rospy
import rospkg
from std_msgs.msg import Float32, Int32
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from pid_tune.msg import PidTune
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

        self.kp_knee = 250 * 0.01
        self.ki_knee = 30 * 0.01
        self.kd_knee = 1 * 0.01

        self.kp_ankle = 50 * 0.01
        self.ki_ankle = 0 * 0.01
        self.kd_ankle = 0 * 0.01

        rospy.Subscriber('/oslsim/imu/osl_shank', Imu, self.osl_knee_pose_cb)
        rospy.Subscriber('/oslsim/imu/osl_foot', Imu, self.osl_ankle_pose_cb)
        rospy.Subscriber('/oslsim/osl_knee/pid', PidTune, self.osl_knee_pid_cb)
        rospy.Subscriber('/oslsim/osl_ankle/pid', PidTune, self.osl_ankle_pid_cb)
    
    def osl_knee_pid_cb(self,data):
        self.kp_knee=float(data.Kp)*0.01
        self.kd_knee=float(data.Kd)*0.01
        self.ki_knee=float(data.Ki)*0.01
        
    def osl_ankle_pid_cb(self,data):
        self.kp_ankle=float(data.Kp)*0.01
        self.kd_ankle=float(data.Kd)*0.01
        self.ki_ankle=float(data.Ki)*0.01

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
       
        raw_knee = list(angles['angle_knee'].values())
        raw_ankle = list(angles['angle_ankle'].values())

        n = -15
        angle_knee = raw_knee[n:]
        angle_knee.extend(raw_knee[:n])

        angle_ankle = raw_ankle[n:]
        angle_ankle.extend(raw_ankle[:n])

        self.setpoint_knee = -0.0174533 * angle_knee[int(self.t%100)]
        self.setpoint_ankle = 0.0174533 * angle_ankle[int(self.t%100)]

        self.error_knee = self.setpoint_knee - self.osl_knee_pose
        self.errord_knee = self.error_knee - self.preverror_knee
        self.errori_knee += self.error_knee

        self.error_ankle = self.setpoint_ankle - self.osl_ankle_pose
        self.errord_ankle = self.error_ankle - self.preverror_ankle
        self.errori_ankle += self.error_ankle

        # -------------------------------------- #

        self.jnt_cmd_dict['osl_ankle'] = (self.kp_ankle*self.error_ankle) + (self.kd_ankle*self.errord_ankle) + (self.ki_ankle*self.errori_ankle)
        self.jnt_cmd_dict['osl_knee'] = (self.kp_knee*self.error_knee) + (self.kd_knee*self.errord_knee) + (self.ki_knee*self.errori_knee)

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
