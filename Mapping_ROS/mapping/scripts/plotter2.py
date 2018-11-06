#!/usr/bin/env python

import rospy
import operator
import numpy as np
import random
from geographiclib.geodesic import Geodesic
from std_msgs.msg import String
from mapping.msg import Single_Measure
import time
import math
import json

class plotter():
        def __init__(self):
            self.coord_data = []
            self.fitted_data = []
            rospy.init_node('plotter')
            self.cmd_sub = rospy.Subscriber('plotter_cmd',String,self.plotter_cmd_callback)
            self.subscriber = rospy.Subscriber('single_measure',Single_Measure,self.ros_callback)
            self.pub = rospy.Publisher('json_plot',String,queue_size=10)
	    self.pubcoord = rospy.Publisher('json_coord',String,queue_size=10)
	    self.maninput = rospy.Subscriber('manual_input',String,self.manual_input_callback)
            rospy.spin()

	def manual_input_callback(self,data):
		if(not data.data):
			return
		if(not self.fitted_data):
			return
		inv = Geodesic.WGS84.Inverse(self.fitted_data[0][0], self.fitted_data[0][1], self.fitted_data[len(self.fitted_data)-1][0], self.fitted_data[len(self.fitted_data)-1][1])
		obj = json.loads(data.data)
		if(not obj):
			return
		azi = None
		if(not obj['dist']):
			return
		point = None
		ptelev = None
		if(obj['dir']=='rmp'):
			azi = inv['azi1']
			point = [inv['lat2'],inv['lon2']]
			ptelev = self.fitted_data[len(self.fitted_data)-1][3]
		elif(obj['dir']=='lmp'):
			azi = inv['azi2']+180
			point = [inv['lat1'],inv['lon1']]
			ptelev = self.fitted_data[0][3]
		else:
			return
		if azi is None:
			return
		if point is None:
			return
		direct = Geodesic.WGS84.Direct(point[0], point[1], azi, obj['dist'])
		self.data_callback((direct['lat2'],direct['lon2'],ptelev + obj['elev']))

        def plotter_cmd_callback(self,data):
                if (data.data=="clear"):
                        self.clear_graph()

        def ros_callback(self,data):
		print("Plotter")
		print(data.elv)
                self.data_callback((data.lat,data.lng,data.elv))

        def data_callback(self,data):
                self.coord_data.append(data)
                line = np.polyfit([i[0] for i in self.coord_data],[j[1] for j in self.coord_data],1)
                perp_slope = np.negative(np.reciprocal(line[0]))
                self.fitted_data = []
                for point in self.coord_data:
                        c = point[1] - (point[0]*perp_slope)
                        x = (c-line[1])/(line[0]-perp_slope)
                        y = (line[0]*x) + line[1]
                        self.fitted_data.append((x,y,0,point[2]))
		self.update_data()

	def update_data(self):
                self.fitted_data = sorted(self.fitted_data, key = lambda x: (x[0],x[1]))
                lmp = self.fitted_data[0]
                self.fitted_data[:] = [(x[0],x[1],self.get_distance((lmp[0],lmp[1]),(x[0],x[1])),x[3]) for x in self.fitted_data]
		self.send_json()

        def get_distance(self,pt1,pt2):
                return  Geodesic.WGS84.Inverse(pt1[0],pt1[1],pt2[0],pt2[1])['s12'];

        def send_json(self):
           	js = json.dumps(self.fitted_data)
		jscoord = json.dumps(self.coord_data)
		msgcoord = String()
		msgcoord.data = jscoord
		msg = String()
		msg.data = js
		self.pubcoord.publish(msgcoord)
		self.pub.publish(msg)

        def clear_graph(self):
                self.fitted_data = []
                self.coord_data = []
                self.send_json()

if  __name__ == "__main__":
	try:
		plot = plotter()
	except rospy.ROSInterruptException:
		pass
