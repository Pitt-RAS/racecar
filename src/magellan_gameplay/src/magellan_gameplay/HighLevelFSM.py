# HighLevelFSM.py
# Author: Thomas Detlefsen
# Date: 9/13/2020

# Controls
# [1] - Phy_Kill
# [2] - Soft_Kill
# [3] - Exit_Pit
# [4] - Start_Warmup
# [5] - Green
# [6] - Yellow
# [7] - Blue
# [8] - Quali_Green
# [9] - Coord_Stop
# [0] - Comm_Stop
# [q] - End_Race
# [w] - Disq
# [e] - is_vel_zero
# [r] - Returned_to_Pit
# [esc/ctrl-c] - Quit Script

# Imports
from enum import Enum
from pynput import keyboard
import time

# Class to describe racecar FSM
class racecarFSM:

	# Initialize car attributes
	def __init__(self):

		# Start listening for keypress
		self.key_listener = keyboard.Listener(on_press=self.on_press)
		self.key_listener.start()
		
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
	def on_press(self, key):

		# Format keyname
		keyname = str(key).strip('u\'')	

		# [1] Phy_Kill
		if keyname == '1':
			if self.state == state.OFF or self.state == state.SLEEP \
			or self.state == state.ON:				
				self.Phy_Kill = not self.Phy_Kill
				self.update()
		# [2] Soft_Kill
		elif keyname == '2':
			if self.state == state.OFF or self.state == state.SLEEP \
			or self.state == state.ON:			
				self.Soft_Kill = not self.Soft_Kill
				self.update()
		# [3] Exit_Pit
		elif keyname == '3':
			if self.state == state.ON:
				self.Exit_Pit = True
				self.update()
		# [4] start_warmup
		elif keyname == '4':
			if self.state == state.EXITING_PIT or self.state == state.EXHIBITING_CAUTION:
				self.start_warmup = True
				self.Yellow = False
				self.Green = False
				self.update()
		# [5] Green
		elif keyname == '5':
			if self.state == state.WARMUP:
				self.Green = True
				self.update()
		# [6] Yellow
		elif keyname == '6':
			if self.state == state.NOMINAL_RACING or self.state == state.WARMUP \
			or (self.state == state.EXHIBITING_CAUTION and self.Green == True):
				self.Yellow = not self.Yellow
				self.update()
		# [7] Blue
		elif keyname == '7':
			if self.state == state.NOMINAL_RACING or self.state == state.ALLOWING_OVERTAKE:
				self.Blue = not self.Blue
				self.update()
		# [8] Quali_Green
		elif keyname == '8':
			if self.state == state.WARMUP:
				self.Quali_Green = True
				self.update()
		# [9] coord_stop
		elif keyname == '9':
			if self.state == state.EXITING_PIT or self.state == state.WARMUP \
 			or self.state == state.NOMINAL_QUALIFYING or self.state == state.NOMINAL_RACING \
			or self.state == state.EXHIBITING_CAUTION or self.state == state.ALLOWING_OVERTAKE:
				self.coord_stop = True
				self.update()
		# [0] Comm_Stop
		elif keyname == '0':
			if self.state == state.EXITING_PIT or self.state == state.WARMUP \
 			or self.state == state.NOMINAL_QUALIFYING or self.state == state.NOMINAL_RACING \
			or self.state == state.EXHIBITING_CAUTION or self.state == state.ALLOWING_OVERTAKE:
				self.Comm_Stop = True
				self.update()
		# [q] end_race
		elif keyname == 'q':
			if self.state == state.EXITING_PIT or self.state == state.WARMUP \
 			or self.state == state.NOMINAL_QUALIFYING or self.state == state.NOMINAL_RACING \
			or self.state == state.EXHIBITING_CAUTION or self.state == state.ALLOWING_OVERTAKE:
				self.end_race = True
				self.update()
		# [w] disq
		elif keyname == 'w':
			if self.state == state.EXITING_PIT or self.state == state.WARMUP \
 			or self.state == state.NOMINAL_QUALIFYING or self.state == state.NOMINAL_RACING \
			or self.state == state.EXHIBITING_CAUTION or self.state == state.ALLOWING_OVERTAKE:
				self.disq = True
				self.update()
		# [e] is_vel_zero
		elif keyname == 'e':
			if self.state == state.ENTERING_PIT or self.state == state.EXECUTING_COORD_STOP:
				self.is_vel_zero = True
				self.coord_stop = False
				self.Comm_Stop = False
				if self.Returned_to_pit:
					self.disq = False
					self.end_race = False
				self.update()
		# [r] Returned_to_pit
		elif keyname == 'r':
			if self.state == state.ENTERING_PIT or self.state == state.EXECUTING_COORD_STOP:
				self.is_vel_zero = True
				self.coord_stop = False
				self.Comm_Stop = False
				if self.is_vel_zero:
					self.disq = False
					self.end_race = False
				self.update()
		# [esc] Quit
		elif keyname == 'esc':
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

	try:
	# Dead Loop
		while racecar.run:
			time.sleep(1)

	except KeyboardInterrupt:
		print("\nExiting FSM")
		exit(0)
