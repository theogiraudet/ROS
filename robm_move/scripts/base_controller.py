#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from robm_nxt.msg import MotorCommand
from geometry_msgs.msg import Twist
from math import pi

pub = rospy.Publisher("nxt/command", MotorCommand, queue_size=3)

def cmd_vel_callback(cmd_vel_msg):
    # Extract speed command from msg
    v = cmd_vel_msg.linear.x
    w = cmd_vel_msg.angular.z
    
    k = 16.8      # motor speed factor (rad/s)
    r = 0.0166    # wheel radius
    l = 0.15/2.0  # half track

    # Linear wheel speeds
    v1 = v + l * w
    v2 = v - l * w

    # Motor rotation speed commands
    u1 = v1 / (r * k)
    u2 = v2 / (r * k)

    # Limit motor speed to +/-1.0
    u_max = max( abs(u1), abs(u2) )
    if u_max > 1.0:
        u1 = u1 / u_max
        u2 = u2 / u_max
    
    # Publish odometry message
    msg = MotorCommand(u1, u2)
    pub.publish(msg)

def base_controller_node():
    # Initialize ROS node
    rospy.init_node('base_controller')
    # Subscribe to motor rotation status
    motor_sub = rospy.Subscriber("cmd_vel", Twist, cmd_vel_callback)
    # Process incoming ROS messages until termination
    rospy.spin()
    
if __name__ == '__main__':
    try:
        base_controller_node()
    except rospy.ROSInterruptException:
        pass
