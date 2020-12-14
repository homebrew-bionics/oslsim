#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from gazebo_msgs.msg import ContactsState

class Loadcell(object):
    def __init__(self):
        self.ll_pub = rospy.Publisher('/oslsim/footL/loadcell/fz', Float32, queue_size=10)
        self.lr_pub = rospy.Publisher('/oslsim/footR/loadcell/fz', Float32, queue_size=10)

    def publish_ll_fz(self, val=0.0):
        self.ll_pub.publish(val)

    def publish_lr_fz(self, val=0.0):
        self.lr_pub.publish(val)

class LoadcellSub(object):
    def __init__(self):
        rospy.Subscriber('/oslsim/footL/loadcell', ContactsState, self.loadcell_l_callback)
        rospy.Subscriber('/oslsim/footR/loadcell', ContactsState, self.loadcell_r_callback)
        self.lc = Loadcell()

    def loadcell_l_callback(self, data):
        val = 0.0
        if len(data.states)>0:
            val = data.states[-1].wrenches[0].force.z
            # rospy.loginfo(rospy.get_caller_id() + "%s", data.states[-1].wrenches[0].force.z)

        self.lc.publish_ll_fz(val)

    def loadcell_r_callback(self, data):
        val = 0.0
        if len(data.states)>0:
            val = data.states[-1].wrenches[0].force.z

        self.lc.publish_lr_fz(val)

    def go(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('loadcell', anonymous=True)
    lc_sub = LoadcellSub()
    lc_sub.go()
    