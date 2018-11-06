#!/usr/bin/env python

import rospy
import serial as ser
import pynmea2
from mapping.msg import gps

def Gps():
    try:
        GPS = ser.Serial("/dev/ttyAMA0",baudrate=57600)
    except ser.SerialException:
        print('could not connect to %s' % "/dev/ttyAMA0")
        return
    pub = rospy.Publisher('gps',gps,queue_size = 10)
    rospy.init_node('rtk_receiver',anonymous = True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():  
	nmea_msg = pynmea2.parse(GPS.readline())
        try:
            #print(nmea_msg.latitude,nmea_msg.longitude,nmea_msg.altitude)
            msg = gps()
	    msg.lat = nmea_msg.latitude
	    msg.lng = nmea_msg.longitude
	    msg.elv = nmea_msg.altitude
            #msg.latitude = nmea_msg.latitude
            #msg.longitude = nmea_msg.longitude
            #msg.elevation = nmea_msg.altitude
            pub.publish(msg)
            rate.sleep()
	    #print(nmea_msg.latitude,nmea_msg.longitude,nmea_msg.altitude)
        except AttributeError:
            continue

if __name__ == '__main__':
    try:
        Gps()
    except rospy.ROSInterruptException:
        pass
