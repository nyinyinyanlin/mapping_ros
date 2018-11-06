#!/usr/bin/env python

import time
import numpy
import rospy
import navio.ms5611
import navio.util
from collections import deque
from std_msgs.msg import Float64

class barometer():
    WINDOW_SIZE = 40
    def __init__(self):
        navio.util.check_apm()
        self.pub = rospy.Publisher('altitude', Float64, queue_size=10)
	self.tempPub = rospy.Publisher('temp', Float64, queue_size=10)
    	rospy.init_node('barometer', anonymous=True)
    	self.rate = rospy.Rate(25)
        self.baro = navio.ms5611.MS5611()
        self.baro.initialize()
	self.arr = []
        while not rospy.is_shutdown():
            msg = Float64()
	    msg.data = self.get_average(self.WINDOW_SIZE)[0]
	    tempMsg = Float64()
	    tempMsg.data = self.baro.TEMP
	    self.pub.publish(msg)
	    self.tempPub.publish(tempMsg)
            self.rate.sleep()
    
    def get_average(self,window_size):
        self.baro.refreshPressure()
	time.sleep(0.01) # Waiting for pressure data ready 10ms
	self.baro.readPressure()
	self.baro.refreshTemperature()
	time.sleep(0.01) # Waiting for temperature data ready 10ms
	self.baro.readTemperature()
	self.baro.calculatePressureAndTemperature()
	self.prepare_array(self.baro.PRES,window_size)
        pressure = self.running_mean(self.arr,len(self.arr))
	return self.calculate_altitude(pressure)
    
    def prepare_array(self,x, N):
	if len(self.arr) == N:
		self.arr.pop(0)
	self.arr.append(x)

    def running_mean(self,x, N):
    	cumsum = numpy.cumsum(numpy.insert(x, 0, 0)) 
	return (cumsum[N:] - cumsum[:-N]) / float(N)

    def calculate_altitude(self,pressure):
        #  computeHeight() - the conversion uses the formula:
        #
        #  h = (T0 / L0) * ((p / P0)**(-(R* * L0) / (g0 * M)) - 1)
        #
        #  where:
        #  h  = height above sea level
        #  T0 = standard temperature at sea level = 288.15
        #  L0 = standard temperatur elapse rate = -0.0065
        #  p  = measured pressure
        #  P0 = static pressure = 1013.25
        #  g0 = gravitational acceleration = 9.80665
        #  M  = mloecular mass of earth's air = 0.0289644
        #  R* = universal gas constant = 8.31432
        #
        #  Given the constants, this works out to:
        #
        #  h = 44330.8 * (1 - (p / P0)**0.190263)
        return 44330.8 * (1 - pow(pressure / 1013.25, 0.190263));

if __name__ == '__main__':
	try:
		baro = barometer()
	except rospy.ROSInterruptException:
		pass

