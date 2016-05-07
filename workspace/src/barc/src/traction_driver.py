#!/usr/bin/env python
# license removed for brevity
import rospy
from data_service.msg import TimeData
from barc.msg import SPEED
from math import pi,sin
import time
import serial
from numpy import zeros, hstack, cos, array, dot, arctan
from manuevers import StraightBrake

def driver():
    pub = rospy.Publisher('speed', SPEED, queue_size=10)
    rospy.init_node('driver', anonymous=True)

    rateHz  = 50
    dt      = 1.0 / rateHz
    rate    = rospy.Rate(rateHz)
    t_i     = 0

    speed = rospy.get_param("driver/speed")
    t_0 = rospy.get_param("driver/t_0")
    t_exp = rospy.get_param("driver/t_exp")

    opt         = TestSettings(SPD = speed, turn = 0, dt=t_exp)
    opt.t_0    = t_0

    while not rospy.is_shutdown():
        (motorCMD, _) = StraightBrake(opt, rateHz, t_i)
        pub.publish(SPEED(speed))
        rate.sleep()
        t_i +=1

if __name__ == '__main__':
    try:
        driver()
    except rospy.ROSInterruptException:
        pass
