#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64, Bool
from mapping.msg import gps, Single_Measure
from mapping.srv import GElevation

class fuser():
	MODE_POLE = 1
	MODE_BOAT = 2
	def __init__(self):
		self.mode = self.MODE_POLE
		self.coord = (None,None)
		self.last_coord = (None,None)
		self.elev = None
		self.altitude = None
		self.alt_offset = None
		self.depth = None
		self.surface_offset = None
		self.pole_offset = 1
		rospy.init_node("fuser")
		self.pub = rospy.Publisher("single_measure",Single_Measure,queue_size=10)
		rospy.Subscriber("gps",gps,self.gps_callback)
		rospy.Subscriber("altitude",Float64,self.altitude_callback)
		rospy.Subscriber("depth",Float64,self.depth_callback)
		rospy.Subscriber("trigger",Bool,self.trigger_callback)
		rospy.Subscriber("start",Bool,self.start_callback)
		rospy.Subscriber("mode",Bool,self.mode_change_callback)
		rospy.Subscriber("stop",Bool,self.stop_callback)
		rospy.spin()

	def mode_change_callback(self,mode):	
		if(mode.data):
			self.mode = self.MODE_POLE
		else:
			if(self.elev == None):
				return
			if(self.altitude == None):
				return
			if(self.alt_offset == None):
				return
			if(self.pole_offset == None):
				return
			if(self.depth == None):
				return
			self.mode = self.MODE_BOAT
			self.surface_offset = self.altitude - self.pole_offset
 
	def send_msg(self,coord):
		print("Send Message")
		print(coord[2])
		msg = Single_Measure()
		msg.lat = coord[0]
		msg.lng = coord[1]
		msg.elv = coord[2]
		self.pub.publish(msg)

	def start_callback(self,data):
		print("Start Callback")
		print(self.coord)
		if (not data.data):
			return
		if (self.coord == (None,None)):
			print("Start failed")
			return
		#elev = self.get_google_elevation(self.coord[0],self.coord[1]).elev
		#if(elev==None):
		#	return
		self.elev = 10
		if(self.altitude==None):
			return
		self.alt_offset = self.altitude
		if(self.pole_offset==None):
			return
		self.send_msg((self.coord[0],self.coord[1],self.altitude - self.pole_offset))
	
	def stop_callback(self,data):
		print("Stop Callback")
		if (not data.data):
			return
		self.coord = (None,None)
		self.elev = None
		self.altitude = None
		self.alt_offset = None
		self.surface_offset = None
		self.depth = None
		self.mode == self.MODE_POLE

	def trigger_callback(self,data):
		print("Trigger Callback")
		print(data.data)
		if (not data.data): 
			return
		if(self.coord != self.last_coord):
			print("Triggered")
			self.last_coord = self.coord
			if(self.mode==self.MODE_POLE):
				if(self.altitude==None):
					return
				if(self.alt_offset==None):
					return
				if(self.pole_offset==None):
					return
				if(self.coord == (None,None)):
					return
				if(self.elev==None):
					return
				print(self.altitude-self.pole_offset)
				self.send_msg((self.coord[0],self.coord[1],self.altitude-self.pole_offset))
			elif(self.mode==self.MODE_BOAT):
				if(self.coord == (None,None)):
					return
				if(self.surface_offset == None):
					return
				if(self.depth == None):
					return
				self.send_msg((self.coord[0],self.coord[1],self.surface_offset-self.depth))		
		else:
			print("not triggered")
					
	def gps_callback(self,data):
		self.coord = (data.lat,data.lng)
		print(self.coord)
		
	def altitude_callback(self,altitude):
		self.altitude = altitude.data

	def depth_callback(self,depth):
		self.depth = depth.data

	def get_google_elevation(self,lat,lng):
		rospy.wait_for_service('get_elev')
		try:
			get_elev = rospy.ServiceProxy('get_elev',GElevation)
			return get_elev(lat,lng)
		except rospy.ServiceException, e:
			pass

if __name__ == "__main__":
	fuser()
