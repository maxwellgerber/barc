#!/usr/bin/env python

# ---------------------------------------------------------------------------
# Licensing Information: You are free to use or extend these projects for 
# education or reserach purposes provided that (1) you retain this notice
# and (2) you provide clear attribution to UC Berkeley, including a link 
# to http://barc-project.com
#
# Attibution Information: The barc project ROS code-base was developed
# at UC Berkeley in the Model Predictive Control (MPC) lab by Jon Gonzales
# (jon.gonzales@berkeley.edu). The cloud services integation with ROS was developed
# by Kiet Lam  (kiet.lam@berkeley.edu). The web-server app Dator was 
# based on an open source project by Bruce Wootton
# ---------------------------------------------------------------------------

import rospy
from data_service.msg import TimeData
from barc.msg import Encoder, MOT, STATE, SPEED
from math import pi,sin
import time
import serial
from numpy import zeros, hstack, cos, array, dot, arctan
from input_map import angle_2_servo, servo_2_angle
from pid import PID
from filtering import filteredSignal

###########################################################
# Set up measure callbacks
# imu measurement update
# TODO

prev_time       = 0
delta_time      = 0
v_x             = 0
speed           = 0
a_x             = 0
sleep_time      = 0.01

N                   = rospy.get_param("longitudinal/N")
Reff                = rospy.get_param("longitudinal/Reff")
counts              = zeros(N)
times               = zeros(N)
Ww_Reff             = 0
desired_slip_ratio  = 0.04

def imu_callback(data):
	global a_x, v_x, delta_time, prev_time
	(_, _, _, a_x, _, _, _, _, _) = data.value
    current_time    = data.timestamp
    delta_time      = current_time - prev_time      # Get the time difference inbetween call backs
    acc_filtered    = filter_acc_x(a_x)             # Uses low pass filter to filter raw accel data
    v_x += acc_filtered *delta_time                 # Integrates accel to get velocity

# encoder measurement update
def encoder_callback(data):
	global counts
    counts = hstack(([data.FL],counts[1:]))
    times = hstack(([rospy.get_rostime()], times[1:]))
    Ww = (counts[0] - counts[-1])/(times[0] - times[-1])
    Ww_Reff = Ww * pi/2 * Reff

def speed_callback(data):
    global speed
    speed = data.speed

def filter_acc_x(acc_raw):
    y0 = 0; a = 0.5; n = 200; method = 'lp'
    filter_obj = filteredSignal(y0=y0, a=a, n=n, method='lp')
    filter_obj.update(a_x)
    return filter_obj.getFilteredSignal()
#############################################################
# main code
def main_auto():
    # initialize ROS node
    rospy.init_node('auto_mode', anonymous=True)
    rospy.Subscriber('imu', TimeData, imu_callback)
    rospy.Subscriber('encoder', Encoder, encoder_callback)
    rospy.Subscriber('speed', SPEED, speed_callback)
    nh = rospy.Publisher('mot', MOT, queue_size = 10)
    log = rospy.Publisher('state', STATE, queue_size = 10)

	# set node rate
    rateHz  = 50
    rate 	= rospy.Rate(rateHz)
    dt      = 1.0 / rateHz 
    p       = rospy.get_param("longitudinal/p")
    i       = rospy.get_param("longitudinal/i")
    d       = rospy.get_param("longitudinal/d")
    pid     = PID(P=1, I=1, D=0)

    # main loop
    while not rospy.is_shutdown():

        # publish command signal 
        # TODO
        global v_x, Ww_Reff, desired_slip_ratio
        slip_ratio      = (Ww_Reff -v_x) /Ww_Reff
        err_slip_ratio  = slip_ratio -desired_slip_ratio
        motor_PWM       = pid.update(err_slip_ratio)
        # motor_PWM       = 90

        nh.publish(MOT(motor_PWM))
        log.publish(STATE(Vx, Ww_Reff, err))
	
        # wait
        rate.sleep(sleep_time)

#############################################################
if __name__ == '__main__':
	try:
		main_auto()
	except rospy.ROSInterruptException:
		pass
