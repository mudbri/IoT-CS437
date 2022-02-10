import numpy as np
from enum import Enum
import math
import heapq


STEPS = 3 # the number of steps to take before recomputing map
ANGLE_RANGE = (-60, 60) # the minimum and maximum angle that the servo can turn. This is relative to the position of ultrasonic sensor in the car
ANGLE_STEPS = 5 # granularity of changing the angle of the servo for distance readings
SCALE = 0.1 # descrete steps taken while calculating intersecting cells along a slope

# class direction(Enum):
# 	FORWARD = 1
# 	BACKWARDS = -1
# 	RIGHT = 2
# 	LEFT = -2

# given an actual position in decimals, return the cell that the position belongs to
def getCell(curr_pos):
	return (math.floor(curr_pos[0]),math.floor(curr_pos[1]))

"""
Maps a distance measurement from ultrasonic to a cell in a grid

Parameters:
------------
car_pos : (int, int) 
    The current position of the car in the grid
car_direction : double
	The direction (angle) at which the car is facing
measurement : (int, int)
	A distance measurement from the ultrosonic in the format (angle, distance), were distance is in cm and angle in degrees
"""
def getPos(car_pos, car_direction, measurement):
	angle = measurement[0]
	distance = measurement[1]
	angle = angle+car_direction # get angle relative to the direction of the positive y axis
	x = car_pos[0]+distance*math.sin(math.radians(angle))
	y = car_pos[1]+distance*math.cos(math.radians(angle))
	return (getCell((x,y)))

# """
# Add points along the slope between two positions (to represent an object between two positions) 
# Methodology: Draw a box around the two positions and for each cell in that box see if it should be 1 or not (by counting its edges)

# Parameters:
# ------------
# grid : np.array 
#     A 2D array 1cm^2 squares of the map with 1 representing an obstacle
# pos1 : (int, int)
# 	Position 1 of a detected obstacle in the grid (x,y)
# pos2 : (int, int)
# 	Position 2 of a detected obstacle in the grid (x,y)
# """
# def addPoints(grid, pos1, pos2):
# 	# TODO: Check if pos1 and pos2 are in the grid
# 	box_bottom_left = (min(pos1[0],pos2[0]),min(pos1[1],pos2[1])) 
# 	box_bottom_right = (max(pos1[0],pos2[0]),max(pos1[1],pos2[1])) 
# 	for x in range(box_bottom_left[0], box_bottom_right[0]):
# 		for y in range(box_bottom_left[1], box_bottom_right[1]):
# 			cell = (x,y)
# 			if cellIncludesLine(cell, pos1, pos2):
# 				grid[cell[0]][cell[1]] = 1

"""
Add points along the slope between two positions (to represent an object between two positions) 
Methodology: Start at pos1 and then move towards pos2 in little increments. With each increment, mark the cell at that position as 1

Parameters:
------------
grid : np.array 
    A 2D array 1cm^2 squares of the map with 1 representing an obstacle
pos1 : (int, int)
	Position 1 of a detected obstacle in the grid (x,y)
pos2 : (int, int)
	Position 2 of a detected obstacle in the grid (x,y)
"""
def addPoints(grid, pos1, pos2):
	# TODO: Check if pos1 and pos2 are in the grid
	curr_pos = pos1
	dx = pos2[0]-pos1[0]
	dy = pos2[1]-pos1[1]
	dx_movement = SCALE*dx
	dy_movement = SCALE*dy
	while getCell(curr_pos) != pos2:
		# print(curr_pos[0],curr_pos[1])
		curr_cell = getCell(curr_pos)
		grid[curr_cell[0]][curr_cell[1]] = 1	
		curr_pos = (dx_movement + curr_pos[0], dy_movement + curr_pos[1])
	grid[pos2[0]][pos2[1]] = 1	

# check if given position exists inside the grid
def isValid(pos, grid_size):
	return (pos[0] >= 0 and pos[1] >= 0 and pos[0] < grid_size[0] and pos[1] < grid_size[1])

"""
Maps a given grid with obstactles (where 1 represents an obstacle)

Parameters:
------------
car_pos : (int, int) 
    The current position of the car in the grid
grid_size : (int, int)
    The grid size in cm
car_direction : double
	The direction (angle) at which the car is facing
angle_range : (int, int)
	The range of angles that from which the distance readings will be obtained
angle_steps: 
	Granularity of changing the angle of the servo for distance readings
"""
def mapGrid(car_pos=(5,0), grid_size=(10,10), car_direction=0, angle_range=ANGLE_RANGE, angle_steps=ANGLE_STEPS):
	grid = np.zeros(grid_size)
	# distance_measurements = getDistanceMeasurements(angle_range, angle_steps)
	distance_measurements = [(60,5),(30,6)]
	for i in range(1, len(distance_measurements)):
		measurement = distance_measurements[i]
		last_measurement = distance_measurements[i-1]
		obstacle_pos = getPos(car_pos, car_direction, measurement)
		last_obstacle_pos = getPos(car_pos, car_direction, last_measurement)
		if isValid(obstacle_pos, grid_size) and isValid(last_obstacle_pos, grid_size):
			addPoints(grid, obstacle_pos, last_obstacle_pos) # Make given positions and cells on a slope between them 1
		# elif isValid(obstacle_pos, grid_size) and not isValid(last_obstacle_pos, grid_size): #TODO: Add clearance
		# 	grid[obstacle_pos[0]][obstacle_pos[1]] = 1
		elif isValid(obstacle_pos, grid_size):
			grid[obstacle_pos[0]][obstacle_pos[1]] = 1
		elif isValid(last_obstacle_pos, grid_size):
			grid[last_obstacle_pos[0]][last_obstacle_pos[1]] = 1
	return grid


# prints grid in normal cartesian axes form
def printGrid(grid, grid_size):
	for y in range(grid_size[1]-1, -1, -1):
		for x in range(0, grid_size[0]):
			print(str(int(grid[x][y])),end=' ')
		print()


# calculates distance between two nodes
def distance(pos1, pos2):
	x_dist = abs(pos1[0]-pos2[0])
	y_dist = abs(pos1[1]-pos2[1])
	return math.sqrt(x_dist*x_dist + y_dist*y_dist)


"""
Finds an optimal path between two points given obstacles
Pseudo-code taken from: https://medium.com/@nicholas.w.swift/easy-a-star-pathfinding-7e6689c7f7b2

Parameters:
------------
grid : np.array 
    A 2D array 1cm^2 squares of the map with 1 representing an obstacle
car_pos : (int, int) 
    The current position of the car in the grid
goal : (int, int)
	The position of the goal where we want the car to navigate to in the grid
grid_size : (int, int)
    The grid size in cm
"""
def findPath(grid, car_pos, goal, grid_size):
	class node:
		def __init__(self, previous, pos, start, goal):
			self.previous = previous
			self.pos = pos
			self.start = start
			self.goal = goal
			self.g = distance(pos, start)
			self.h = distance(pos, goal)
			self.f = self.g+self.h

		def __eq__(self, pos2):
			return self.pos == pos2.pos
		def __lt__(self, pos2):
			return self.pos < pos2.pos
		def __le__(self, pos2):
			return self.pos <= pos2.pos
		def __ne__(self, pos2):
			return self.pos != pos2.pos
		def __ge__(self, pos2):
			return self.pos >= pos2.pos
		def __gt__(self, pos2):
			return self.pos > pos2.pos

		def isRoot(self):
			return self.g == 0 # starting point

		def isGoal(self):
			return self.h == 0 # ending point

		def adjacentChildren(self, grid, grid_size):
			adjacents = []
			for dy in range(-1, 2, 1):
				for dx in range(-1, 2, 1):
					new_pos = (self.pos[0] + dx, self.pos[0] + dy)
					if (new_pos == self.pos or not isValid(new_pos, grid_size)):
						continue
					if grid[new_pos[0]][new_pos[1]] == 1:
						continue
					new_node = node(previous=self, pos=new_pos, start=self.start, goal=self.goal)
					adjacents.append(new_node)
			return adjacents


	openList = []
	closedList = []
	curr_node = node(None, car_pos, car_pos, goal)
	heapq.heappush(openList, (0, curr_node))
	while (len(openList) > 0):
		curr_node = heapq.heappop(openList)[1]
		closedList.append(curr_node)

		if (curr_node.isGoal()):
			return 0
			# return curr_node.backtrack()

		children = curr_node.adjacentChildren(grid, grid_size)
		print()
		print(curr_node.pos)
		for child in children:
			print("child", child.pos)
			if child in closedList:
				continue

			skip = False
			replace = False
			i = 0
			for openChildTuple in openList:
				openChild = openChildTuple[1]
				if (openChild == child and child.g > openChild.g):
					skip = True
					break
				elif (openChild == child):
					replace = True
					break				
				i+=1

			if skip:
				continue
			elif replace:
				openList[i] = (child.f, child)
			else:
				heapq.heappush(openList, (child.f, child))

	# heapq.heappush(openList, (5, (1,2)))
	# heapq.heappush(openList, (2, (3,4)))
	# heapq.heappush(openList, (7, (1,4)))
	# heapq.heappush(openList, (1, (5,5)))
	# heapq.heappush(openList, (8, (1,1)))
	# print(heapq.heappop(openList))
	# print(heapq.heappop(openList))
	# print(heapq.heappop(openList))


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
car_direction : double
	The direction (angle) at which the car is facing
"""
def navigate(car_pos=(0,0), goal=(9,9), grid_size=(10,10), car_direction=0):
	while (car_pos != goal):
		grid = mapGrid(car_pos, grid_size, car_direction, ANGLE_RANGE, ANGLE_STEPS)
		# addClearance(grid, CAR_SIZE) # TODO: adds extra space around obstacles to allow for the size of the car
		printGrid(grid, grid_size)
		path = findPath(grid, car_pos, goal, grid_size) # path is a series of cells to visit to get to the goal
		car_pos = goal
		# (car_pos, car_direction) = moveCar(path, STEPS, car_pos, car_direction)

navigate()
	