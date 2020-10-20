#!/usr/bin/env python
#-*- coding: utf-8 -*-
#
# Name: collision.py
# Developer: Ben Merrick
# Date: 10/07/2020
# Description: This program uses NED velocity vectors to prevent two drones
#   from a head-on collision


#All imports for the program
import time
import os
import math
from dronekit import Vehicle, connect, VehicleMode, LocationGlobalRelative
from dronekit_sitl import SITL
from flight_plotter import Location, CoordinateLogger, GraphPlotter
from ned_utilities import ned_controller
from shapely.geometry import LineString



######################################################################
#Class Name: Copter
#Description: A class to represent a drone with basic actions
######################################################################
class copter:
	######################################################################
	#Name: Init
	#Parameters: ID number, Global Relative Locations, Number for alt
	#Return: None
	#Description: Assign the initial variables of the class
	######################################################################
	def __init__(self, badge, home, target, alt):
		#Assigned variables
		self.home 	= home
		self.badge 	= badge
		self.target 	= target
		self.alt 	= alt

		#Unassigned variables
		self.sitl 	= None
		self.vehicle 	= None
		self.init_dist 	= 0
		self.logger 	= CoordinateLogger()



	######################################################################
	#Name: Connect Virtual vehicle
	#Parameters: None
	#Return: None
	#Description: Connect the drone to system and ready for flight
	######################################################################
	def connect_virtual_vehicle(self):
    		self.sitl = SITL()
    		self.sitl.download('copter', '3.3', verbose=True)
    		instance_arg = '-I%s' %(str(self.badge))
    		print("Drone instance is: %s" % instance_arg)
		speedup_arg = '--speedup=4'
    		home_arg = '--home=%s, %s,%s,180' % (str(self.home.lat), str(self.home.lon), str(self.home.alt))
    		sitl_args = [instance_arg, '--model', 'quad', home_arg, speedup_arg]
    		self.sitl.launch(sitl_args, await_ready=True)
    		tcp, ip, port = self.sitl.connection_string().split(':')
    		port = str(int(port) + self.badge * 10)
    		conn_string = ':'.join([tcp, ip, port])
    		print('Connecting to vehicle on: %s' % conn_string)

    		self.vehicle = connect(conn_string)
    		self.vehicle.wait_ready(timeout=120)



	######################################################################
	#Name: Arm 
	#Parameters: None
	#Return: None
	#Description: Arm the drone and change the mode to Guided
	######################################################################
	def arm(self):
		#Arm the copters and fly to target altitude
		print("Waiting for drone #%i to initialize..." % self.badge)
		print("Arming drone #%i motors" % self.badge)
		self.vehicle.mode = VehicleMode('GUIDED')
		self.vehicle.armed = True
		print("Drone #%i vehicle armed" % self.badge)

	
	######################################################################
	#Name: Land 
	#Parameters: None 
	#Return: None
	#Description: Change vehicle mode of drone to land 
	######################################################################
	def land(self):
		self.vehicle.mode = VehicleMode("LAND")
		print("Drone #%i landing..." % self.badge)

	
	######################################################################
	#Name: Close
	#Parameters: None
	#Return: None
	#Description: Close the drone
	######################################################################
	def close(self):
		print("Closed drone #%i" % self.badge)
		self.vehicle.close()
		self.sitl.stop()



######################################################################
#Class Name: Ground Control
#Description: The main communicato for the drones to perform actions
######################################################################
class GroundControl:

	######################################################################
	#Name: Init
	#Parameters: Number of drones, Array to store all drones, Nums for tols
	#Return: None
	#Description: Set the initial variables of the class
	######################################################################
	def __init__(self, num_drones, DRONES, time_tol, range_tol, dist_target, stop_tol, scale, unit_test):
		self.num_drones    = num_drones
		self.DRONES 	   = DRONES
		self.nedcontroller = ned_controller()
		self.range_tol     = range_tol
		self.time_tol	   = time_tol
		self.dist_target   = dist_target
		self.stop_tol	   = stop_tol
		self.unit_test	   = unit_test

		self.collision_detected = False
		self.scale 		= scale


	######################################################################
	#Name: Plot Flight
	#Parameters: None
	#Return: None
	#Description: Plot the flight plan of the two drones
	######################################################################
	def plot_flight(self):
		drone0 = self.DRONES[0]
		drone1 = self.DRONES[1]
		plotter = GraphPlotter(drone0.logger.lat_array, drone0.logger.lon_array, drone1.logger.lat_array, drone1.logger.lon_array, "Longitude", "Latitude", "NED Flight Path")

		plotter.scatter_plot()



	######################################################################
	#Name: Copters at Altitude
	#Parameters: None
	#Return: None
	#Description: Fly all drones to alt in sync
	######################################################################
	def copters_at_altitude(self):
		#Loop until all drones at altitude
		while True:
			at_altitude = True
			ctr = 0
			for drone in self.DRONES:
				print('Copter ID: {} at altitude {}'.format(ctr, str(drone.vehicle.location.global_relative_frame.alt)))
				ctr = ctr + 1
				if (not drone.vehicle.location.global_relative_frame.alt >= drone.alt*0.95):
					at_altitude = False
			time.sleep(3)

			if at_altitude == True:
				print("All drones have reached their target altitude")
				break


	######################################################################
	#Name: Copters Arm
	#Parameters: None
	#Return: None
	#Description: Arm all of the copters and sleep if not armed
	######################################################################
	def copters_arm(self):
		for drone in self.DRONES:
			drone.arm()

		for drone in self.DRONES:
			while not(drone.vehicle.armed):
				time.sleep(1)



	######################################################################
	#Name: Copters Armable
	#Parameters: None
	#Return: None
	#Description: A check function to test if all drones have been armed 
	######################################################################
	def copters_armable(self):
		while True:
			unarmable = False
			for drone in self.DRONES:
				if (not drone.vehicle.is_armable):
					unarmable = True
			time.sleep(3)

			if unarmable == False:
				break


	######################################################################
	#Name: Arm and Takeoff
	#Parameters: None
	#Return: None
	#Description: Driver to arm and takeoff all drones in sync
	######################################################################
	def arm_and_takeoff(self):
		for drone in self.DRONES:
			#Connect the drone
			drone.connect_virtual_vehicle()
			
		#Don't arm until autopilot is ready
		print("Basic pre-arm checks")
		self.copters_armable()

		print("Arming motors")
		self.copters_arm()
	
		print("Vehicles armed!")
	
		print("All drones are now taking off!")
		for drone in self.DRONES:
			drone.vehicle.simple_takeoff(drone.alt)

		print("Waiting for copters to ascend")
		self.copters_at_altitude()


	######################################################################
	#Name: Land Close
	#Parameters: None
	#Return: None
	#Description: Land all drones and then close them
	######################################################################
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


	######################################################################
	#Name: Fly Drones
	#Parameters: None
	#Return: None
	#Description: Tell all drones to fly to targets using NED vectors
	######################################################################
	def fly_drones(self):
		print("Flying all drones to target!")

		#Loop through all drones and set NED vectors
		for drone in self.DRONES:
			starting = Location(drone.vehicle.location.global_relative_frame.lat, drone.vehicle.location.global_relative_frame.lon)
			
			#Set initial distance to know how long we have until target
			drone.init_dist = get_distance(starting, drone.target)
			
			#Set NED vector
			ned = self.nedcontroller.setNed(starting, drone.target)
			#self.nedcontroller.send_ned_velocity(ned.north, ned.east, ned.down, 1, drone.vehicle)

		while True:
			at_pos 	= False
			ctr 	= 0

			for drone in self.DRONES:
				#Print current drone flight information
				currLat = drone.vehicle.location.global_relative_frame.lat
				currLon = drone.vehicle.location.global_relative_frame.lon
				speed   = drone.vehicle.groundspeed
				#print('Copter ID: {} at {},{} with speed {}'.format(ctr, str(currLat), str(currLon), speed))
				ctr += 1
				
				#Log the data points
				drone.logger.add_data(currLat, currLon)

				#Calculate remaining distance
				currentLocation = Location(drone.vehicle.location.global_relative_frame.lat, drone.vehicle.location.global_relative_frame.lon)

				distance_to_target = get_distance(currentLocation, drone.target)
				print(distance_to_target)

				#Exit if reached otherwise recalculate ned
				if distance_to_target < self.dist_target:
					at_pos = True
				else:
					at_pos = False
					#Check if current drone will collide with other drones
					if self.collision_detected == False:
						self.check_collision(drone)
					
					#Send next ned vector
					current = Location(drone.vehicle.location.global_relative_frame.lat, drone.vehicle.location.global_relative_frame.lon)
					ned = self.nedcontroller.setNed(current, drone.target)
					self.nedcontroller.send_ned_velocity(ned.north*self.scale, ned.east*self.scale, ned.down*self.scale, 1, drone.vehicle)

					
			time.sleep(0.5)

			#Break if all drones are done flying
			if at_pos == True:
				print("All drones have SAFELY reached their targets")
				break
	

	######################################################################
	#Name: Check Collision
	#Parameters: Drone object
	#Return: None
	#Description: See if current drone is within vicinty of other drones
	######################################################################
	def check_collision(self, curr_drone):
		#Check relative distance of current drone to the rest of drones
		print("Checking for possible distance warning...")
		for drone in self.DRONES:
			#Ignore the current drone in the list
			if drone.badge == curr_drone.badge:
				continue
			#Get distance between
			distance_between = get_distance(curr_drone.vehicle.location.global_relative_frame, drone.vehicle.location.global_relative_frame)


			#Within relative distance for collision
			if(distance_between < self.range_tol):
				self.collision_detected = True
				print("POSSIBLE COLLISION DETECTED")
				self.collision_path(curr_drone, drone)
				self.collision_dtected = False
				return
		self.collision_detected = False
		print("Collision detection check complete")


	######################################################################
	#Name: Collision Path
	#Parameters: Two drone objects
	#Return: None
	#Description: Determine the point at which two drones will collide
	######################################################################
	def collision_path(self, drone1, drone2):
		print("Calculating possible collision path...")
		#Determine the line equations of both drone
		x1 = float(drone1.home.lon)
		y1 = float(drone1.home.lat)
		x2 = float(drone1.target.lon)
		y2 = float(drone1.target.lat)

		x3 = float(drone2.home.lon)
		y3 = float(drone2.home.lat)
		x4 = float(drone2.target.lon)
		y4 = float(drone2.target.lat)

		#Use Shapely to find the intersection of the lines
		line1 = LineString([(x1, y1), (x2, y2)])
		line2 = LineString([(x3, y3), (x4, y4)])
		intersect = line1.intersection(line2)
	
		#Check to see if we have an intersection
		if (type(intersect).__name__) == 'Point':
			lon,lat = intersect.x, intersect.y
			point = LocationGlobalRelative(lat, lon, 0)
			print("Point of collision: %s, %s" % (point.lat, point.lon))
			self.velocity_path(drone1, drone2, point)
			return
		else:
			print("No collision on drone flight path")



	######################################################################
	#Name: Velocity Path
	#Parameters: Two drone objects and a Global Relative Location
	#Return: None
	#Description: Determine time until collision using V = M/S
	######################################################################
	def velocity_path(self, drone1, drone2, point):
		print("Calculating time until drones collide...")
		#Determine time to impact for first drone
		dist1 = get_distance(drone1.vehicle.location.global_relative_frame, point)
		time1 = dist1 / drone1.vehicle.groundspeed
		print("Drone #%i reaching collision point in %f seconds" % (drone1.badge, time1))

		#Determine time to impact for second drone
		dist2 = get_distance(drone2.vehicle.location.global_relative_frame, point)
		time2 = dist2 / drone2.vehicle.groundspeed
		print("Drone #%i reaching collision point in %f seconds" % (drone2.badge, time1))

		#check to see if both are within range
		if(abs(time1-time2) < self.time_tol):
			print("Folks, we have a collision!!")
			self.stop_collision(drone1, drone2, point)
			return
	

	
	######################################################################
	#Name: Stop Collision
	#Parameters: Two drone objects and a Global Relative Location
	#Return: None
	#Description: One drone to stops until other has passed collision point
	######################################################################
	def stop_collision(self, drone1, drone2, point):
		print("Temporarily stopping collision...")
		#Tell slowest  drone to stop
		if drone1.vehicle.groundspeed < drone2.vehicle.groundspeed:
			stop_drone = drone1
			move_drone = drone2
		else:
			stop_drone = drone2
			move_drone = drone1

		#Stop drone until it has passed
		print("Stopping Drone #%i until Drone #%i has passed." % (stop_drone.badge, move_drone.badge))
		if(self.unit_test == True):
			print("Not performing flying as this is a unit test")
			return
		current = stop_drone.vehicle.location.global_relative_frame
		while get_distance(move_drone.vehicle.location.global_relative_frame, point) > self.stop_tol:
			#Set stop drone to current target of its current location
			ned = self.nedcontroller.setNed(stop_drone.vehicle.location.global_relative_frame, current)
			self.nedcontroller.send_ned_velocity(ned.north*self.scale, ned.east*self.scale, ned.down*self.scale, 1, stop_drone.vehicle)
			print("Move Drone Distance to Target %f" % (get_distance(move_drone.vehicle.location.global_relative_frame, move_drone.target)))
			print("Move Drone Distance to Collision %f" % (get_distance(move_drone.vehicle.location.global_relative_frame, point)))
		        print("Stop Drone Distance to Target %f" %(get_distance(stop_drone.vehicle.location.global_relative_frame, stop_drone.target)))
			print("\n")

			#Set move drone to its original target location
			ned = self.nedcontroller.setNed(move_drone.vehicle.location.global_relative_frame, move_drone.target)
			self.nedcontroller.send_ned_velocity(ned.north*self.scale, ned.east*self.scale, ned.down*self.scale, 1, move_drone.vehicle)

		self.collision_detected = False


		

	
########################################################################
#Name: Get Location
#Parameters: Global Relative Location, Distance desired north and east
#Return: None
#Description: Helper to get new loc from original & desired dist change
########################################################################		
def get_location(original, dNorth, dEast):
	earth_radius = 6378137.0

	dLat = dNorth/earth_radius
	dLon = dEast/(earth_radius*math.cos(math.pi*original.lat/180))

	newlat = original.lat + (dLat * 180/math.pi)
	newlon = original.lon + (dLon * 180/math.pi)

	return LocationGlobalRelative(newlat, newlon, original.alt)



######################################################################
#Name: Get Distance
#Parameters: Two Global Locations
#Return: None
#Description: Helper to find distance between two functions
######################################################################
def get_distance(locationA, locationB):
	R = 6373.0
	
	lat1 = math.radians(locationA.lat)
	lon1 = math.radians(locationA.lon)
	lat2 = math.radians(locationB.lat)
	lon2 = math.radians(locationB.lon)

	dlon = lon2 - lon1
	dlat = lat2 - lat1

	a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
	c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

	distance = (R * c) * 1000

	return distance



######################################################################
#Name: Main
#Parameters: Four Global Relative Locations and other vars for Unit Tests
#Return: None
#Description: Main driver to run the collision avoidance program
######################################################################
def main(coord0, coord1, coord2, coord3, alt, time_tol, range_tol, dist_target, stop_tol, scale, unit_test):
	#Initial variables
	DRONES = []

	print("Building copters...")
	d1 = copter(0, coord0, coord1, alt)
	d2 = copter(1, coord2, coord3, alt)
	DRONES.append(d1)
	DRONES.append(d2)
	
	print("Building ground control...")
	ground_control = GroundControl(len(DRONES), DRONES, time_tol, range_tol, dist_target, stop_tol, scale, unit_test)

	print("Arming and taking off...")
	ground_control.arm_and_takeoff()

	print("Flying drones...")
	ground_control.fly_drones()

	#print("Landing copters...")
	ground_control.land_close()

	print("Printing total flight path...")
	ground_control.plot_flight()	
	
	return

######################################################################
#Name: Unit Tests
#Parameters: None
#Return: None
#Description: Main driver to run the collision avoidance program 
#    unit tests to check validity of specific functions
######################################################################
def unit_tests():
	#Initial variables
	alt = 10
	time_t = 10
	range_t = 60
	dist_targ = 5
	stop_t = 10
	scale = 0.25
	unit_test = True
	DRONES = []


	#UNIT TEST 1 - No collision imminent
	#Given different starting points
	position0 = LocationGlobalRelative(41.715436, -86.24473, 0)
	position1 = LocationGlobalRelative(41.714436, -86.24173, 0)
	position2 = LocationGlobalRelative(41.714236, -86.24473, 0)
	position3 = LocationGlobalRelative(41.714936, -86.24173, 0)
	
	print("Building copters...")
	d1 = copter(0, position0, position1, alt)
	d2 = copter(1, position2, position3, alt)
	DRONES.append(d1)
	DRONES.append(d2)
	
	print("Building ground control...")
	ground_control = GroundControl(len(DRONES), DRONES, time_t, range_t, dist_targ, stop_t, scale, unit_test)

	print("Arming and taking off...")
	ground_control.arm_and_takeoff()

	print("UNIT TEST OF NO COLLISION PROGRAM")
	print("No collision imminent for this test")
	ground_control.check_collision(d1)

	ground_control.land_close()

	#UNIT TEST 2 - Collision Imminent
	DRONES = []

	#Given same starting points
	position0 = LocationGlobalRelative(41.715436, -86.24473, 0)
	position1 = LocationGlobalRelative(41.714436, -86.24173, 0)
	position2 = LocationGlobalRelative(41.715436, -86.24473, 0)
	position3 = LocationGlobalRelative(41.714936, -86.24173, 0)


	print("Building copters...")
	d12 = copter(0, position0, position1, alt)
	d22 = copter(1, position2, position3, alt)
	DRONES.append(d12)
	DRONES.append(d22)
	
	print("Building ground control...")
	ground_control2 = GroundControl(len(DRONES), DRONES, time_t, range_t, dist_targ, stop_t, scale, unit_test)

	print("Arming and taking off...")
	ground_control2.arm_and_takeoff()

	print("UNIT TEST OF COLLISION PROGRAM")
	print("Collision imminent for this test")
	ground_control2.check_collision(d12)

	ground_control2.land_close()


	

######################################################################
#Name: Acceptance Tests
#Parameters: None
#Return: None
#Description: Main driver to run the collision avoidance program
#    acceptance tests to check validiy of overall program
######################################################################
def acceptance_tests():
	#Initial variables
	alt = 10
	time_t = 10
	range_t = 60
	dist_targ = 5
	stop_t = 10
	scale = 0.25
	unit_test = False

	
	print("\nPerforming Test #1...ANGLED")
	position0 = LocationGlobalRelative(41.715436, -86.24473, 0)
	position1 = LocationGlobalRelative(41.714436, -86.24173, 0)
	position2 = LocationGlobalRelative(41.714236, -86.24473, 0)
	position3 = LocationGlobalRelative(41.714936, -86.24173, 0)
	main(position0, position1, position2, position3, alt, time_t, range_t, dist_targ, stop_t, scale, unit_test)
	print("ACCEPTANCE TEST #1 SUCCESS")


	#Change speed
	scale = 0.5
	print("\nPerforming Test #2...CROSS")
	position0 = LocationGlobalRelative(41.714446, -86.24573, 0)
	position1 = LocationGlobalRelative(41.714436, -86.24073, 0)
	position2 = LocationGlobalRelative(41.716836, -86.24323, 0)
	position3 = LocationGlobalRelative(41.712036, -86.24333, 0)
	main(position0, position1, position2, position3, alt, time_t, range_t, dist_targ, stop_t, scale, unit_test)
	print("ACCEPTANCE TEST #2 SUCCESS")

	#Change speed
	scale = 0.3
	print("\nPerforming Test #3...HEAD ON")
	position0 = LocationGlobalRelative(41.715436, -86.24473, 0)
	position1 = LocationGlobalRelative(41.714436, -86.24173, 0)
	position2 = LocationGlobalRelative(41.715446, -86.24473, 0)
	position3 = LocationGlobalRelative(41.714436, -86.24173, 0)
	main(position0, position1, position2, position3, alt, time_t, range_t, dist_targ, stop_t, scale, unit_test)
	print("ACCEPTANCE TEST #3 SUCCESS")
	

	#Change speed
	scale = 0.7
	print("\nPerforming Test #3...PARALLEL LINES")
	position0 = LocationGlobalRelative(41.715436, -86.24473, 0)
	position1 = LocationGlobalRelative(41.714436, -86.24173, 0)
	position2 = LocationGlobalRelative(41.714986, -86.24473, 0)
	position3 = LocationGlobalRelative(41.713936, -86.24173, 0)
	main(position0, position1, position2, position3, alt, time_t, range_t, dist_targ, stop_t, scale, unit_test)
	print("ACCEPTANCE TEST #4 SUCCESS")




#RUN IT ALL
if __name__ == "__main__":
	print("Performing Unit Tests...")
	unit_tests()

	print("Performing Acceptance Tests...")
	acceptance_tests()
	
