import numpy as np
from enum import Enum

STEPS = 3 # the number of steps to take before recomputing map
ANGLE_RANGE = (-60, 60) # the minimum and maximum angle that the servo can turn. This is relative to the position of ultrasonic sensor in the car
ANGLE_STEPS = 5 # granularity of changing the angle of the servo for distance readings

class direction(Enum):
	FORWARD = 1
	BACKWARDS = -1
	RIGHT = 2
	LEFT = -2


"""
Maps a distance measurement to a cell in a grid

Parameters:
------------
car_pos : (int, int) 
    The current position of the car in the grid
car_direction : int
	The direction in which the car is facing. 1 means front (parallel to positive y axis), -1 means backwards (parallel to negative y axis), 2 means right (parallel to positive x-axis), -2 means left (parallel to negative y axis)
measurement : (int, int)
	A distance measurement from the ultrosonic in the format (angle, distance), were distance is in cm and angle in degrees
"""
def getPos(car_pos, car_direction, measurement):
	

"""
Maps a given grid with obstactles (where 1 represents an obstacle)

Parameters:
------------
car_pos : (int, int) 
    The current position of the car in the grid
grid_size : (int, int)
    The grid size in cm
car_direction : int
	The direction in which the car is facing. 1 means front (parallel to positive y axis), -1 means backwards (parallel to negative y axis), 2 means right (parallel to positive x-axis), -2 means left (parallel to negative y axis)
angle_range : (int, int)
	The range of angles that from which the distance readings will be obtained
angle_steps: 
	Granularity of changing the angle of the servo for distance readings
"""
def mapGrid(car_pos=(5,0), grid_size=(10,10), car_direction=direction.FORWARD, angle_range=ANGLE_RANGE, angle_steps=ANGLE_STEPS):
	grid = np.zeros(grid_size)
	distance_measurements = getDistanceMeasurements(angle_range, angle_steps)
	for i in range(1, len(distance_measurements)):
		measurement = distance_measurements[i]
		last_measurement = distance_measurements[i-1]
		obstacle_pos = getPos(car_pos, car_direction, measurement)
		last_obstacle_pos = getPos(car_pos, car_direction, last_measurement)
		if validPos(obstacle_pos, grid_size) && validPos(last_obstacle_pos, grid_size):
			addPoints(grid, obstacle_pos, last_obstacle_pos) # Make given positions and cells on a slope between them 1
		elif validPos(obstacle_pos, grid_size):
			grid[obstacle_pos[0]][obstacle_pos[1]] = 1
		elif validPos(last_obstacle_pos, grid_size):
			grid[last_obstacle_pos[0]][last_obstacle_pos[1]] = 1

		

"""
Navigates the car to a goal

Parameters:
------------
car_pos : (int, int) 
    The current position of the car in the grid
goal : (int, int)
	The position of the goal where we want the car to navigate to in the grid
grid_size : (int, int)
    The grid size in cm
car_direction : int
	The direction in which the car is facing. 1 means front (parallel to positive y axis), -1 means backwards (parallel to negative y axis), 2 means right (parallel to positive x-axis), -2 means left (parallel to negative y axis)
"""
def navigate(car_pos=(5,0), goal=(9,9), grid_size=(10,10), car_direction=direction.FORWARD):
	while (car_pos != goal):
		grid = mapGrid(car_pos, grid_size, car_direction, ANGLE_RANGE, ANGLE_STEPS)
		car_pos = goal
		# path = findPath(grid, car_pos, goal) # path is a series of cells to visit to get to the goal
		# (car_pos, car_direction) = moveCar(path, STEPS, car_pos, car_direction)

navigate()
	