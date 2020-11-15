#!/usr/bin/env python 
import rospy
from std_msgs.msg import Float64
import thread 
import math
pub={}
def ini(str,x=-1):
    if x==-1:
        while not rospy.is_shutdown():
         pub[str].publish(-ang)
    else:
        k=math.pi/1
        if str=='ankleL' or str=='ankleR':
           amp=-ang
        else:
           amp=2*ang
        if str[-1]=='R':
           phi=-phase
        else:
           phi=0
        while not rospy.is_shutdown():
           xo=rospy.get_time()
           theta=amp*math.fabs(math.sin(k*(xo-x)+phi))
           pub[str].publish(theta)
           
if __name__=='__main__':
    try:
        rospy.init_node('walking_gait',anonymous=True)
        joint=['hipL','kneeL','ankleL','hipR','kneeR','ankleR']
        for j in joint:
         pub[j]=rospy.Publisher('/oslsim/'+j+'_position_controller/command',Float64,queue_size=10)
        phase=math.pi/2
        ang=0.3
        t=rospy.get_time()
        try: 
         thread.start_new_thread(ini,('kneeL',))
         thread.start_new_thread(ini,('kneeR',))
         thread.start_new_thread(ini,('hipR',t,))
         thread.start_new_thread(ini,('hipL',t,))
         thread.start_new_thread(ini,('ankleL',t,))
         thread.start_new_thread(ini,('ankleR',t,))
        except:
         print('ERROR: cannot start the threads')
    except rospy.ROSInterruptException:
        print('ROSINTERRUPTEXCEPTION ERROR')
        pass