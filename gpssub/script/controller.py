#!/usr/bin/env python
#coding=utf-8

import rospy
from gpssub.msg import gps
from gpssub.msg import command
import PID_control

def callback(data):
    heading = data.Header
    lattitude = data.GPSWeek
    longitude = data.GPSTime
    Ve = data.Ve
    Vn = data.Vn

    CommandSpeed , CommandSteering = PID_control.main(lattitude , longitude , heading , Ve , Vn)

    pub = rospy.Publisher('command', command, queue_size = 10)
    rate = rospy.Rate(1)
    msg1 = command()
    msg1.speed , msg1.steer = CommandSpeed , CommandSteering
    # rospy.loginfo(msg1)
    pub.publish(msg1)
    rate.sleep()

def listener():
    rospy.init_node('controller' , anonymous=True)
    
    rospy.Subscriber('gps_info' , gps , callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
#    talker()
