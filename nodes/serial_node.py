#!/usr/bin/env python

import rospy
from orientalmotor_ros import OrientalMotor

def main():
    rospy.init_node("motor_control")
    port = rospy.get_param('~port','/dev/ttyUSB0')
    sub = OrientalMotor(port)
    rospy.spin()

if __name__ == '__main__':
    main()
