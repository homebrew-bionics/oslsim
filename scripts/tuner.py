#!/usr/bin/env python
from Tkinter import *
from pid_tune.msg import PidTune
import rospy

class PID():
	def __init__(self, title, topic, kp=100, ki=0, kd=0, queue_size=1000):
		self.pub_pid = rospy.Publisher(topic, PidTune, queue_size=queue_size)
		self.pid_params = PidTune()

		self.root = Tk()
		self.root.configure()
		self.root.title(title)
		self.root.attributes("-topmost", True)
		self.root.geometry('320x240') 

		self.kpscale = Scale(self.root, orient='horizontal', highlightthickness=0, bd=0, fg="black", troughcolor="#00274C", from_=0, to=1200, label= 'Proportional gain (Kp):', width = "15", length = "300", sliderlength="15")
		self.kiscale = Scale(self.root, orient='horizontal', highlightthickness=0, bd=0, fg="black", troughcolor="#00274C", from_=0, to=1200, label= 'Integral gain (Ki):', width = "15", length = "300", sliderlength="15")
		self.kdscale = Scale(self.root, orient='horizontal', highlightthickness=0, bd=0, fg="black", troughcolor="#00274C", from_=0, to=1200, label= 'Derivative gain (Kd):', width = "15", length = "300", sliderlength="15")

		self.kpscale.set(kp)
		self.kiscale.set(ki)
		self.kdscale.set(kd)

		self.kpscale.pack(pady=2)
		self.kiscale.pack(pady=2)
		self.kdscale.pack(pady=2)

		self.set_value()

		Button(self.root, highlightthickness=0, bd=0, bg="#FFCB05", activeforeground="white", activebackground="#00274C", text='Update', command=self.set_value, height=2, width=8).pack(pady=10)

	def set_value(self):

		self.pid_params.Kp = self.kpscale.get()
		self.pid_params.Ki = self.kiscale.get()
		self.pid_params.Kd = self.kdscale.get()
		self.pub_pid.publish(self.pid_params)

rospy.init_node('oslsim_pid_pub',anonymous=True)
osl_knee_pid = PID(title='OSL Knee PID', topic='/oslsim/osl_knee/pid', kp=250, ki=4, kd=1)
osl_ankle_pid = PID(title='OSL Ankle PID', topic='/oslsim/osl_ankle/pid', kp=50, ki=0, kd=0)

osl_knee_pid.root.mainloop()
osl_ankle_pid.root.mainloop()