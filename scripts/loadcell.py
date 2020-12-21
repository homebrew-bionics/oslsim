#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from gazebo_msgs.msg import ContactsState

class Loadcell(object):
    def __init__(self):
        self.lr_pub = rospy.Publisher('/oslsim/loadcell/fz', Float32, queue_size=10)

    def publish_lr_fz(self, val=0.0):
        self.lr_pub.publish(val)

class LoadcellSub(object):
    def __init__(self):
        rospy.Subscriber('/oslsim/loadcell', ContactsState, self.loadcell_r_callback)
        self.lc = Loadcell()

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
    