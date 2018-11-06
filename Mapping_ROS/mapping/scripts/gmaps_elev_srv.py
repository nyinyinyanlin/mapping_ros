#!/usr/bin/env python

import rospy
import urllib2
import json
from mapping.srv import GElevation

def getElev(req):
	content = urllib2.urlopen("https://maps.googleapis.com/maps/api/elevation/json?locations="+str(req.lat)+","+str(req.lng)+"&key=AIzaSyDt-IRfqD_RTeLBXpr9ARzzQpefHLqzvj0").read();
	jsonObj = json.loads(content)
	if(jsonObj['status']=='OK'):
		return jsonObj['results'][0]['elevation']
	else:
		return None

def gmaps_elev_server():
	rospy.init_node('gmaps_elev_server')
	serv = rospy.Service('get_elev',GElevation,getElev)
	rospy.spin()

if __name__ == "__main__":
	gmaps_elev_server()
