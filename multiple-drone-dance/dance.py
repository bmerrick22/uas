 #!/usr/bin/env python
# -*- coding: utf-8 -*-
# Developed and Designed By
#     - Ben Merrick -
#     -   9/23/20   -
#


#All imports for the function
import time
from dronekit_sitl import SITL
from dronekit import Vehicle, VehicleMode, connect, LocationGlobalRelative
import math
import matplotlib.pyplot as plt 
from matplotlib.animation import FuncAnimation
import numpy as np 

#COPTER CLASS
class copter:
	#Initialize the drone with variables
	def __init__(self, badge, home, alt, vehicle, sitl, airspeed, groundspeed):
		self.badge       = badge
		self.home        = home
		self.alt         = alt
		self.vehicle     = vehicle
		self.sitl        = sitl
		self.airspeed    = airspeed
		self.groundspeed = groundspeed
		self.line_data_x = [0]
		self.line_data_y = [0]
		self.target      = LocationGlobalRelative(home[0], home[1], alt)
		self.initial_dist= 0

	def connect(self):
		#Initiaize drone
		self.sitl = SITL()
    		self.sitl.download('copter', '3.3', verbose=True)
    		badge_arg = '-I%s' %(str(self.badge))
    		print("Drone instance is: %s" % badge_arg)
    		home_arg = '--home=%s, %s,%s,180' % (str(self.home[0]), str(self.home[1]), str(self.home[2]))
		speedup_arg = '--speedup=4'
    		sitl_args = [badge_arg, '--model', 'quad', home_arg, speedup_arg]
    		self.sitl.launch(sitl_args, await_ready=True)
    		tcp, ip, port = self.sitl.connection_string().split(':')
    		port = str(int(port) + self.badge * 10)
    		conn_string = ':'.join([tcp, ip, port])
    		print('Connecting to vehicle on: %s' % conn_string)

		#Connect Vehicle
    		self.vehicle = connect(conn_string)
    		self.vehicle.wait_ready(timeout=120)

	#Arm the drone
	def arm(self):
		#Arm the copters and fly to target altitude
		print("Waiting for drone #%i to initialize..." % self.badge)
		print("Arming drone #%i motors" % self.badge)
		self.vehicle.mode  = VehicleMode("GUIDED")
		self.vehicle.armed = True
		print("Drone #%i vehicle armed" % self.badge)

	#Land the drone
	def land(self):
		self.vehicle.mode = VehicleMode("LAND")
		print("Drone #%i landing..." % self.badge)
		
	#Close the drone
	def close(self):
		print("Closing drone #%i" % self.badge)
		self.vehicle.close()
		self.sitl.stop()

	#Print information about the drone
	def info(self):
		print("Drone #%i - %s, %s - %s" % (self.badge, self.vehicle.location.global_relative_frame.lat, self.vehicle.location.global_relative_frame.lon, self.vehicle.location.global_relative_frame.alt))

	#Fly to target of drone
	def flyto_target(self):
		self.vehicle.simple_goto(self.target)
 		
	#Get the location from the current location to desired deltax and deltay
	def get_location(self, dNorth, dEast):
		#Current location
		current = self.vehicle.location.global_relative_frame

		#Calculate distances
		earth_radius = 6378137.0
		dLat = dNorth/earth_radius
		dLon = dEast/(earth_radius*math.cos(math.pi*current.lat/180))

		#New Position
		new_lat = current.lat + (dLat * 180/math.pi)
		new_lon = current.lon + (dLon * 180/math.pi)
		target  = LocationGlobalRelative(round(new_lat, 6), round(new_lon, 6), current.alt)

		#Return target position
		return target

	#Get distance from drone current location to target
	def get_distance(self):
		drone = self.vehicle.location.global_relative_frame
		dLat = drone.lat - self.target.lat
		dLon = drone.lon - self.target.lon
		#return distance from position to target
		return math.sqrt((dLat*dLat) + (dLon*dLon)) * 1.113195e5


#GROUND CONTROL Class
class GroundControl:
	#Create the ground control class and intialize with variables
	def __init__ (self, num_drones, start, groundspeed, airspeed, alt):
		self.num_drones  = num_drones
		self.start       = start
		self.groundspeed = groundspeed
		self.airspeed    = airspeed
		self.alt         = alt
		self.DRONES	 = []

	
	#Determine if the copters have reached desired altitude
	def copters_at_altitude(self):
		#Loop until all drones are at altitiude
    		while True:
        		at_altitude = True
        		ctr=0
       			for drone in self.DRONES:
            			print ('Copter ID: {} at altitude {} '.format(ctr,str(drone.vehicle.location.global_relative_frame.alt))) 
            			ctr = ctr + 1
           			if (not drone.vehicle.location.global_relative_frame.alt >= self.alt * 0.95):
                			at_altitude = False
        		time.sleep(3)

        		if at_altitude == True:
            			print("All drones have reached their target altitudes")
            			break     

	#Arm the drones
	def copters_arm(self):
		#Arm all drones
		for drone in self.DRONES:
			drone.arm()

		#Wait until all drones armed
		for drone in self.DRONES:
			while not (drone.vehicle.armed):
				time.sleep(1)


	#Wait until all drones are armed to begin
	def copters_armable(self):
    		while True:
        		unarmable = False
        		for drone in self.DRONES:
            			if (not drone.vehicle.is_armable):
                			unarmable = True
        		time.sleep(3)

        		if unarmable == False:
            			break     

	#Arm and takeoff drones simultaneously
	def arm_and_takeoff(self):
		#Connect all drones
		for drone in self.DRONES:
			drone.connect()
    
    		print("Basic pre-arm checks")
    		# Don't try to arm until autopilot is ready
    		self.copters_armable()
 
    		print("Arming motors")
    		self.copters_arm()
  
    		print("Vehicles armed!")

    		print("All drones are now Taking off!")
    		for drone in self.DRONES:
        		drone.vehicle.simple_takeoff(self.alt)  # Take off to target altitude

    		print("Waiting for copters to ascend")
    		self.copters_at_altitude()


	#Build all the copters we need
	def build_copters(self):
		#build drones based upon ground control input
		for badge in range(self.num_drones):
			drone = copter(badge, self.start, self.alt, None, None, self.airspeed, self.groundspeed)

			self.DRONES.append(drone)


	#Move copters simultaneously and wait until all reached target
	def move_copters_wait(self):
		print("Flying all copters to target!")
		#Fly to specified target
		for drone in self.DRONES:
			drone.initial_dist = drone.get_distance()
			drone.flyto_target()


		while True:
			at_pos = False
			ctr    = 0
			for drone in self.DRONES:
				#Graph Information
				drone.line_data_x.append((drone.vehicle.location.global_relative_frame.lat - drone.home[0])*1000000)
				drone.line_data_y.append((drone.vehicle.location.global_relative_frame.lon - drone.home[1])*1000000)

				#Print information about position
				print('Copter ID: {} at {},{}'.format(ctr, str(drone.vehicle.location.global_relative_frame.lat), str(drone.vehicle.location.global_relative_frame.lon)))
				ctr+=1
				#Calculate remaining distance
				remaining = drone.get_distance()
				if remaining <= drone.initial_dist*0.01:
					at_pos = True
			time.sleep(0.5)
			#Loop until all reached distance
			if at_pos == True:
				print("All drones have reached their target positions and are waiting")
				break
		
	
	#Get specific drone information	
	def specific_info_copter(self, badge):
		#Get info on specific copter
		ref_drone = self.DRONES[badge]
		ref_drone.info()


	#Get all drone information
	def info_copters(self):
		#Get info on all copters
		for drone in self.DRONES:
			drone.info()

	#Land all drones and then close them 
	def land_close(self):
		#Land all drones
		for drone in self.DRONES:
			drone.land()
		time.sleep(30)
		print("All drones LANDED")

		#Close all drones
		for drone in self.DRONES:
			drone.close()
		print("All drones CLOSED")

		
#DANCE ALGORITHM
def dance(gc):
	#Variables for reference
	start = gc.start
	num_drones = gc.num_drones
		
	#Set the square position of the drones
	dist = 50
	#Drone 1 bottom left - don't need to change
	#Drone 2 top left
	gc.DRONES[1].target = gc.DRONES[1].get_location(dist, 0)
	#Drone 3 top right
	gc.DRONES[2].target = gc.DRONES[2].get_location(dist, dist)	
	#Drone 4 bottom right
	gc.DRONES[3].target = gc.DRONES[3].get_location(0, dist)

	#Position Drones in square to begin sequence
	gc.move_copters_wait()

	
	#Begin sequence
	print("Beginning drone sequence...")
	x = 1
	y = -1
	turn = 0
	move_round = 0
	iterations = 0
	#Move for one N iteratons
	while iterations < 2:
		#Check the round we are on and set alternators
		if move_round == 0:
			turn = 0
			x, y = 1, -1
		elif move_round == 1:
			turn = 1
			x, y = 1, 1
		elif move_round == 2:
			turn  = 0
			x, y, = -1, 1
		elif move_round == 3:
			turn = 1
			x, y = -1, -1
	
		#Loop through drones and set appropraite target
		for drone in gc.DRONES:
			#Set appropriate x and y change
			if turn == 0:
				turn = 1
				drone.target = drone.get_location(0, x*dist)
				print(x*dist, 0)
				x*=-1
			else:
				turn = 0
				drone.target = drone.get_location(y*dist, 0)
				print(0, y*dist)
				y*=-1
		#Move drones
		gc.move_copters_wait()
		#Completed move and increment counter
		move_round += 1
		#Reset if at end
		if move_round > 3:
			move_round = 0
			iterations += 1
	
		
#Update the lines for animation
def update(i, graph_data):
	#Graph the current points for all drones we have
	for drone in graph_data:
		line = drone["line"]
		x    = drone["x"]
		y    = drone["y"]
		line.set_data(x[:i], y[:i])

#Animate the drone path
def animate(ground_control):
	#Create the plots
	fig, ax = plt.subplots()
	ax.set_xlim(-200, 750)
	ax.set_ylim(-200, 750)
	ax.set_xlabel('Latitude Displacement from Start')
	ax.set_ylabel('Longitude Displacement from Start')
	ax.set_title('Drone Flight Path')


	#Create and store all graph data
	graph_data = []
	line_color = ['red', 'green', 'blue', 'yellow']
	color = 0
	for drone in ground_control.DRONES:
		line, = ax.plot([], [])
		line.set_color(line_color[color])
		graph_data.append(
			{'line':line, 
			 'x':drone.line_data_x, 
			 'y':drone.line_data_y
			}
		)
		color+=1
	#Graph the animation of the drones flying
	animation = FuncAnimation(fig, func=update, fargs=[graph_data], frames=len(graph_data[0]['x']), interval=200)
	plt.show()
	
				 	
#MAIN DRIVER
def main():
	#Variables
	num_drones  = 4
	alt         = 10
	airspeed    = 3
	groundspeed = 10
	start 	    = [41.7148412, -86.2419385,0]

	#Ground Control Creation
	# -Provide initial variables for starting program
	ground_control = GroundControl(num_drones, start, groundspeed, airspeed, alt)

	#Use Ground Control to tkaeoff, fly, and land drones
	ground_control.build_copters()
	ground_control.arm_and_takeoff()
	dance(ground_control)
	ground_control.land_close()

	#Animate the flight path
	animate(ground_control)

	#Goodbye!
	print("Completed flying and graphing. Goodbye!")

 

#RUN IT ALL
if __name__ == "__main__":
	main()
