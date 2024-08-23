#!/usr/bin/env python
import os
import sys
import copy
import rospy
import copy
import subprocess
from math import *
from gcode_pkg.msg import Position_EE
from Tkinter import *
import Tkinter, Tkconstants, tkFileDialog

#constants
GCODE_UNIT_MM=1
DO_CALIB=0
DO_ALIGNMENT=1
VEL = 15 #mm/s

#colours
G  = '\033[92m' # green
W  = '\033[0m'  #white
B  = '\033[94m'  #blue

HOME_X = 0
HOME_Y = 160
HOME_Z = 290


class gcode_interpreter(object):


  #Constructor function: initializes variables and defines publisher 
  def __init__(self):
    
    super(gcode_interpreter, self).__init__()
    self.pub = rospy.Publisher('coordinates', Position_EE, queue_size=50)
    rospy.init_node('gcode_interpreter')
    #self.rate = rospy.Rate(10)	# Hz
    self.read_file = None
    self.parsed_lines = []


  # Generates a window in order to select the gcode file from the hdd
  def read_gcode_file(self):
	
  	print "============ Seleziona il file .gcode "
  	root = Tk()
  	root.withdraw()
  	root.filename = tkFileDialog.askopenfilename(initialdir = os.path.dirname(__file__),title = "Selezione il file .gcode",filetypes = (("file gcode","*.gcode"),("file gcode","*.ngc"),("all files","*.*")))
  	self.read_file=open(root.filename,"r").readlines()
  	print "============ Percorso assoluto: "+ root.filename;	
  	root.destroy()
  	return root.filename


  # Generates a dictionary structure with all the parsed lines from the gcode file
  def parse_gcode(self):
  	
  	x_pos = 0
  	y_pos = 0
  	z_pos = 0
	conv = ""
	first_line = 1
  	for line in self.read_file:
  	 	line = line.replace("\n","")
  	 	line = line.replace("\r","")
		if line:	#check if not empty
	 		sub_codes=line.split()
	 		if sub_codes[0]=="G1" or sub_codes[0]=="G01" or sub_codes[0]=="G00" :
				if first_line:
					conv = "clik"
				else:
					conv = "path"
				for sub_code in sub_codes:
					if sub_code[0]=="X":
						x_pos=sub_code[1:]
					elif sub_code[0]=="Y":
						y_pos=sub_code[1:]
					elif sub_code[0]=="Z":
						z_pos=sub_code[1:]
				self.parsed_lines.append(dict(zip(['x_pos', 'y_pos', 'z_pos', 'convergence'],[x_pos,y_pos,z_pos,conv])))
				first_line = 0
	self.parsed_lines.append(dict(zip(['x_pos', 'y_pos', 'z_pos', 'convergence'],[HOME_X,HOME_Y,HOME_Z,"clik"])))


  #Publish all the coordinates in the dictionary as a Position_EE variable using ROS publisher
  def execute_gcode(self):

  	print "============ Premi 'invio' per portare la penna sul foglio"
  	raw_input()
  
  	for parsed_line in self.parsed_lines:
		
		new_point = Position_EE()
		new_point.p.x = float(parsed_line["x_pos"])
		new_point.p.y = float(parsed_line["y_pos"])
		new_point.p.z = float(parsed_line["z_pos"])
		new_point.convergence = parsed_line["convergence"]
		print B + "x: ", new_point.p.x , "\ty: ", new_point.p.y , "\tz: ", new_point.p.z
		print B + "convergence: ", new_point.convergence
		
		self.pub.publish(new_point)
		#self.rate.sleep()

		if new_point.convergence == "path":
			rospy.sleep(sleep_time(old_point.p,new_point.p))
		else:
			print W + "============ Premi 'invio' per continuare o terminare l'esecuzione"
			raw_input()

		old_point = new_point



#Estimates time necessary to go from old to new point
def sleep_time(point1,point2):
  #Compute euclidean distance between points
  dist = sqrt(pow((point2.x-point1.x),2) +  pow((point2.y-point1.y),2) + pow((point2.z-point1.z),2))
  return dist/VEL				
		


def main():

  try:

    print G + "======================"
    print G + "GCODE-INTERPRETER v1.0"   
    print G + "======================"
    print(B+'Unita di misura di default:'+G+' mm' if GCODE_UNIT_MM else B + 'Unita di misura di default: '+ G+'non mm')
    print(B+'Calibrazione:' + G + ' ON' if DO_CALIB else B+'Calibrazione:'+G+' OFF')
    print(B+'Allineamento:'+G+' ON' if DO_ALIGNMENT else 'Allineamento:'+G+' OFF')
    print B+  "======================"
    print W+" "

    gcode_interpreter_in = gcode_interpreter()
    file_path=gcode_interpreter_in.read_gcode_file()
   
    if DO_CALIB:
      gcode_interpreter_in.robot_calibration()
    
    gcode_interpreter_in.parse_gcode()
    gcode_interpreter_in.execute_gcode()

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return


if __name__ == '__main__':
  main()

