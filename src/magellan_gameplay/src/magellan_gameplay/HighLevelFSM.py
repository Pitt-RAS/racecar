#!/usr/bin/env python

# HighLevelFSM.py
# Author: Thomas Detlefsen
# Last Update: 9/21/2020

# Topic Name: FSM
# Node Name: HighLevelFSM
# Message Type: std_msgs/String

# Imports
from enum import Enum
import rospy
from std_msgs.msg import String

# Class to describe racecar FSM
class racecarFSM:

	# Initialize car attributes
	def __init__(self):
		
		# Initialize State as off
		self.state = state.OFF
		print(self.state.name)
		
		# Define quit Variable
		self.run = True
		
		# Define Variables
		self.Phy_Kill = True
		self.Soft_Kill = True
		self.Exit_Pit = False
		self.start_warmup = False
		self.Green = False
		self.Yellow = False
		self.Blue = False
		self.Quali_Green = False
		self.coord_stop = False
		self.Comm_Stop = False	
		self.end_race = False
		self.disq = False
		self.is_vel_zero = False
		self.Returned_to_pit = False

	# Press response
	def callback(self, msg):

		# Phy_Kill
		if msg.data == "Phy_Kill":
			if self.state == state.OFF or self.state == state.SLEEP \
			or self.state == state.ON:				
				self.Phy_Kill = not self.Phy_Kill
				self.update()
		# Soft_Kill
		elif msg.data == "Soft_Kill":
			if self.state == state.OFF or self.state == state.SLEEP \
			or self.state == state.ON:			
				self.Soft_Kill = not self.Soft_Kill
				self.update()
		# Exit_Pit
		elif msg.data == "Exit_Pit":
			if self.state == state.ON:
				self.Exit_Pit = True
				self.update()
		# start_warmup
		elif msg.data == "start_warmup":
			if self.state == state.EXITING_PIT or self.state == state.EXHIBITING_CAUTION:
				self.start_warmup = True
				self.Yellow = False
				self.Green = False
				self.update()
		# Green
		elif msg.data == "Green":
			if self.state == state.WARMUP:
				self.Green = True
				self.update()
		# Yellow
		elif msg.data == "Yellow":
			if self.state == state.NOMINAL_RACING or self.state == state.WARMUP \
			or (self.state == state.EXHIBITING_CAUTION and self.Green == True):
				self.Yellow = not self.Yellow
				self.update()
		# Blue
		elif msg.data == "Blue":
			if self.state == state.NOMINAL_RACING or self.state == state.ALLOWING_OVERTAKE:
				self.Blue = not self.Blue
				self.update()
		# Quali_Green
		elif msg.data == "Quali_Green":
			if self.state == state.WARMUP:
				self.Quali_Green = True
				self.update()
		# coord_stop
		elif msg.data == "coord_stop":
			if self.state == state.EXITING_PIT or self.state == state.WARMUP \
 			or self.state == state.NOMINAL_QUALIFYING or self.state == state.NOMINAL_RACING \
			or self.state == state.EXHIBITING_CAUTION or self.state == state.ALLOWING_OVERTAKE:
				self.coord_stop = True
				self.update()
		# Comm_Stop
		elif msg.data == "Comm_Stop":
			if self.state == state.EXITING_PIT or self.state == state.WARMUP \
 			or self.state == state.NOMINAL_QUALIFYING or self.state == state.NOMINAL_RACING \
			or self.state == state.EXHIBITING_CAUTION or self.state == state.ALLOWING_OVERTAKE:
				self.Comm_Stop = True
				self.update()
		# end_race
		elif msg.data == "end_race":
			if self.state == state.EXITING_PIT or self.state == state.WARMUP \
 			or self.state == state.NOMINAL_QUALIFYING or self.state == state.NOMINAL_RACING \
			or self.state == state.EXHIBITING_CAUTION or self.state == state.ALLOWING_OVERTAKE:
				self.end_race = True
				self.update()
		# disq
		elif msg.data == "disq":
			if self.state == state.EXITING_PIT or self.state == state.WARMUP \
 			or self.state == state.NOMINAL_QUALIFYING or self.state == state.NOMINAL_RACING \
			or self.state == state.EXHIBITING_CAUTION or self.state == state.ALLOWING_OVERTAKE:
				self.disq = True
				self.update()
		# is_vel_zero
		elif msg.data == "is_vel_zero":
			if self.state == state.ENTERING_PIT or self.state == state.EXECUTING_COORD_STOP:
				self.is_vel_zero = True
				self.coord_stop = False
				self.Comm_Stop = False
				if self.Returned_to_pit:
					self.disq = False
					self.end_race = False
				self.update()
		# Returned_to_pit
		elif msg.data == "Returned_to_pit":
			if self.state == state.ENTERING_PIT or self.state == state.EXECUTING_COORD_STOP:
				self.is_vel_zero = True
				self.coord_stop = False
				self.Comm_Stop = False
				if self.is_vel_zero:
					self.disq = False
					self.end_race = False
				self.update()
		# quit
		elif msg.data == 'quit':
			print("\nExiting FSM")
			self.run = False

	# Update state and variables after keypress
	def update(self):

		# -----EXIT TRACK-----
		# EXECUTING_COORD_STOP
		if self.coord_stop == True or self.Comm_Stop == True:
			print("EXECUTING_COORD_STOP")
			self.state = state.EXECUTING_COORD_STOP
			self.Exit_Pit = False
			self.start_warmup = False
			self.Green = False
			self.Yellow = False
			self.Blue = False
			self.Quali_Green = False
			
		# ENTERING_PIT		
		elif self.disq == True or self.end_race == True:
			print("ENTERING_PIT")
			self.state = state.ENTERING_PIT
			self.Exit_Pit = False
			self.start_warmup = False
			self.Green = False
			self.Yellow = False
			self.Blue = False
			self.Quali_Green = False

		# -----MOVING-----
		# NOMINAL_RACING
		elif self.Green == True and self.Yellow == False and self.Blue == False:
			print("\nNOMINAL_RACING")
			self.state = state.NOMINAL_RACING
			self.start_warmup = False
		# EXHIBITING_CAUTION
		elif self.Yellow == True:
			print("\nEXHIBITING_CAUTION")
			self.state = state.EXHIBITING_CAUTION
			self.start_warmup = False
		# ALLOWING_OVERTAKE
		elif self.Green == True and self.Blue == True:
			print("ALLOWING_OVERTAKE")
			self.state = state.ALLOWING_OVERTAKE
		# NOMINAL_QUALLIFYING
		elif self.Quali_Green == True:
			print("NOMINAL_QUALIFYTING")
			self.state = state.NOMINAL_QUALIFYING
			self.start_warmup = False
		# WARMUP
		elif self.start_warmup == True:
			print("\nWARMUP")
			self.state = state.WARMUP
			self.Exit_Pit = False
			self.Yellow = False
			self.Green = False
		
		# -----STATIONARY-----
		# OFF
		elif self.Phy_Kill == True:
			print("\nOFF")
			self.state = state.OFF
			self.Soft_Kill = True
		# SLEEP
		elif self.Phy_Kill == False and self.Soft_Kill == True:
			print("\nSLEEP")
			self.state = state.SLEEP
		# ON
		elif self.Soft_Kill == False and self.Exit_Pit == False:
			print("\nON")			
			self.state = state.ON
		# EXITING_PIT
		elif self.Exit_Pit == True:
			print("\nEXITING PIT")
			self.state = state.EXITING_PIT

# Possible states of racecar
class state(Enum):
	OFF = 1
	SLEEP = 2	
	ON = 3
	WARMUP = 4
	NOMINAL_RACING = 5
	NOMINAL_QUALIFYING = 6
	ALLOWING_OVERTAKE = 7
	EXHIBITING_CAUTION = 8
	EXECUTING_COORD_STOP = 9
	ENTERING_PIT = 10
	EXITING_PIT = 11

if __name__ == '__main__':	

	# Initialize Racecar
	racecar = racecarFSM()

	# Initialize Node and start listening
	rospy.init_node('HighLevelFSM')
	rospy.Subscriber('FSM', String, racecar.callback)

	# Dead loop
	rospy.spin()
