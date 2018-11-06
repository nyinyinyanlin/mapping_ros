#!/usr/bin/env python

import rospy
import operator
import numpy as np
import matplotlib.pyplot as plt
import random
from geopy.distance import vincenty
from mapping.msg import Single_Measure
import time
import math
import matplotlib
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2TkAgg
from matplotlib.figure import Figure
from matplotlib.ticker import AutoMinorLocator
import matplotlib.style
import Tkinter as tk
import tkFileDialog
import cPickle as pickle
from os.path import expanduser

matplotlib.use("TkAgg")
matplotlib.style.use("bmh")

class plotter(tk.Frame):
	def __init__(self,master=None):
		tk.Frame.__init__(self,master)

		self.btngp = tk.Frame(master)
		self.btngp.pack(side=tk.TOP)

		self.clrbtn = tk.Button(self.btngp, text="Clear", command=self.clear_graph)
		self.clrbtn.pack(side=tk.RIGHT)
		self.unsubbtn = tk.Button(self.btngp, text="Unsubscribe", command=self.unsubscribe)
		self.unsubbtn.pack(side=tk.RIGHT)
		self.subbtn = tk.Button(self.btngp, text="Subscribe", command=self.subscribe)
		self.subbtn.pack(side=tk.RIGHT)
		self.exportbtn = tk.Button(self.btngp, text="Export", command=self.export_data)
		self.exportbtn.pack(side=tk.RIGHT)
		self.importbtn = tk.Button(self.btngp, text="Import", command=self.import_data)
		self.importbtn.pack(side=tk.RIGHT)

		self.f = Figure(figsize=(5,5), dpi=100)
		self.ax = self.f.gca()
		self.ax.set_xlabel("DISTANCE FROM LEFT-MOST POINT (M)")
		self.ax.set_ylabel("ELEVATION ABOVE SEA LEVEL (M)")
		self.ax.grid(which='both')

		self.ax.xaxis.set_minor_locator(AutoMinorLocator())
		self.ax.yaxis.set_minor_locator(AutoMinorLocator())
		self.ax.set_autoscale_on(True)
		self.line,= self.ax.plot([],[])

		self.canvas = FigureCanvasTkAgg(self.f,master=root)
		self.canvas.get_tk_widget().pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True)
		self.canvas.draw()

		self.coord_data = []
		self.aligned_data = []
		self.index_data = []
		self.dist_data = []
		self.subscriber = None

		rospy.init_node('plotter',anonymous=True)

	def ros_callback(self,data):
		self.data_callback((data.lat,data.lng,data.elv))

	def subscribe(self):
		self.subscriber = rospy.Subscriber('single_measure',Single_Measure,self.ros_callback)

	def unsubscribe(self):
		self.subscriber.unregister()
		self.subscriber = None

	def data_callback(self,data):
		self.coord_data.append(data)
		line = np.polyfit([i[0] for i in self.coord_data],[j[1] for j in self.coord_data],1)
		perp_slope = np.negative(np.reciprocal(line[0]))
		self.fitted_data = []
		for point in self.coord_data:
			c = point[1] - (point[0]*perp_slope)
			x = np.negative((line[1]+c)/(line[0]+perp_slope))
			y = (line[0]*x) + line[1]
			dist = 0
			self.fitted_data.append((x,y,dist,point[2]))
		self.fitted_data = sorted(self.fitted_data, key = lambda x: (x[0],x[1]))		
		lmp = self.fitted_data[0]
		self.fitted_data[:] = [(x[0],x[1],self.get_distance((lmp[0],lmp[1]),(x[0],x[1])),x[3]) for x in self.fitted_data]
		self.update_plot()

	def pt_on_line(self,pt,pt1,pt2):
		x, y = pt
		x1, y1 = pt1
		x2, y2 = pt2
		if x2 == x1 and y2 == y1:
			x3 = x
			y3 = y
		else:
			k = ((y2-y1) * (x-x1) - (x2-x1) * (y-y1)) / (math.pow((y2-y1),2) + math.pow((x2-x1),2))
			x3 = x - k * (y2-y1)
			y3 = y + k * (x2-x1)
		return (x3,y3)

	def get_distance(self,pt1,pt2):
		return  vincenty(pt1, pt2).meters

	def update_plot(self):
		self.line.set_ydata([i[3] for i in self.fitted_data])
		self.line.set_xdata([i[2] for i in self.fitted_data])
		self.ax.relim()
		self.ax.autoscale_view(True,True,True)
		self.canvas.draw()

	def clear_graph(self):
		self.fitted_data = []
		self.coord_data = []
		self.update_plot()

	def prepare_data(self):
		prepared_data = []
		for index, point in enumerate(self.fitted_data):
			prepared_data.append((index,point[0],point[1],point[2],point[3]))
		return prepared_data

	def import_data(self):
		if(self.subscriber):
			self.unsubscribe()
		self.clear_graph()
		self.master.filename = tkFileDialog.askopenfilename(initialdir = expanduser('~/Documents'),title = "Select file",filetypes = (("Cross-Section Data","*.csd"),("all files","*.*")))
		with open(self.master.filename,'rb') as f:
			loaded_data = pickle.load(f)
			for data in loaded_data:
				self.fitted_data.append([data[2],data[3]])
			self.update_plot()

	def export_data(self):
		self.master.filename = tkFileDialog.asksaveasfilename(initialdir = expanduser('~/Documents'),title = "Save as", filetypes = (("Cross-Section Data","*.csd"),("Comma-Separated Value","*.csv"),("Formatted Text File","*.txt"),("Graph Image","*.png"),("All","*.*")))
		ext = self.master.filename[-3:]
		data = self.prepare_data()
		subscribed = False

		if(self.subscriber):
			subscribed = True
			self.unsubscribe()

		if (ext == 'csd'):
			self.write_csd(self.master.filename,data)
		elif (ext == 'csv'):
			self.write_csv(self.master.filename,data)
		elif (ext == 'txt'):
			self.write_txt(self.master.filename,data)
		elif (ext == 'png'):
			self.write_png(self.master.filename)
		else:
			self.write_csd(self.master.filename+".csd",data)
			self.write_csv(self.master.filename+".csv",data)
			self.write_txt(self.master.filename+".txt",data)
			self.write_png(self.master.filename+".png")

		if(subscribed):
			self.subscribe()

	def write_csv(self,filename,data):
		with open(filename,'wb') as f:
			for (index, lat,lng,dflmp,elv) in data:
				f.write('{},{},{},{},{}\n'.format(index,lat,lng,dflmp,elv))

	def write_txt(self,filename,data):
		with open(filename,'wb') as f:
			for (index, lat,lng,dflmp,elv) in data:
				f.write('{}\t{}\t{}\t{}\t{}\n'.format(index,lat,lng,dflmp,elv))

	def write_csd(self,filename,data):
		with open(filename,'wb') as f:
			pickle.dump(data,f,pickle.HIGHEST_PROTOCOL)

	def write_png(self,filename):
		self.f.savefig(filename,dpi=300)

if __name__ == "__main__":
	root = tk.Tk()
	root.title("Cross-Section Graph - Live")
	pl = plotter(master=root)
	pl.mainloop()
