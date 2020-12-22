#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from gazebo_msgs.msg import ContactsState

class Loadcell(object):
    def __init__(self):
        self.pub_fx = rospy.Publisher('/oslsim/loadcell/fx', Float32, queue_size=10)
        self.pub_fy = rospy.Publisher('/oslsim/loadcell/fy', Float32, queue_size=10)
        self.pub_fz = rospy.Publisher('/oslsim/loadcell/fz', Float32, queue_size=10)

    def publish_fx(self, val=0.0):
        self.pub_fx.publish(val)

    def publish_fy(self, val=0.0):
        self.pub_fy.publish(val)

    def publish_fz(self, val=0.0):
        self.pub_fz.publish(val)

class LoadcellSub(object):
    def __init__(self):
        rospy.Subscriber('/oslsim/loadcell', ContactsState, self.loadcell_callback)
        self.lc = Loadcell()

    def loadcell_callback(self, data):
        x = 0.0
        y = 0.0
        z = 0.0

        if len(data.states)>0:
            x = data.states[-1].wrenches[0].force.x
            y = data.states[-1].wrenches[0].force.y
            z = data.states[-1].wrenches[0].force.z

        self.lc.publish_fx(x)
        self.lc.publish_fy(y)
        self.lc.publish_fz(z)

    def go(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('loadcell', anonymous=True)
    lc_sub = LoadcellSub()
    lc_sub.go()
    