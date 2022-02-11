import numpy as np
from enum import Enum
import math
import heapq
import picar_4wd as fc
from picar_4wd import Ultrasonic
import time


STEPS = 3 # the number of steps to take before recomputing map
ANGLE_RANGE = (-50, 50) # the minimum and maximum angle that the servo can turn. This is relative to the position of ultrasonic sensor in the car
ANGLE_STEPS = 5 # granularity of changing the angle of the servo for distance readings
SCALE = 0.01 # descrete steps taken while calculating intersecting cells along a slope
speed = 5
TURN_VALUE = 763 # used to turn car at 90 degrees. This value depends on speed

class direction(Enum):
	FORWARD = 2
	BACKWARD = -2
	RIGHT = 1
	LEFT = -1

# given an actual position in decimals, return the cell that the position belongs to
def getCell(curr_pos):
	return (math.floor(curr_pos[0]),math.floor(curr_pos[1]))

"""
Maps a distance measurement from ultrasonic to a cell in a grid

Parameters:
------------
car_pos : (int, int) 
    The current position of the car in the grid
car_direction : int
	The direction at which the car is facing. Can be one of the enum direction: FORWARD, BACKWARD, RIGHT, LEFT
measurement : (int, int)
	A distance measurement from the ultrosonic in the format (angle, distance), were distance is in cm and angle in degrees

Returns:
------------
cell : (int, int)
	A cell in the grid i.e. (x,y) of bottom left corner
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

def neighbours(pos1, pos2):
	if (pos1 == pos2):
		return True
	elif (abs(pos1[0]-pos2[0]) <= 1 and abs(pos1[1]-pos2[1]) <= 1):
		return True
	return False

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
	print(pos1)
	print(pos2)

	curr_pos = pos1
	dx = pos2[0]-pos1[0]
	dy = pos2[1]-pos1[1]
	dx_movement = SCALE*dx
	dy_movement = SCALE*dy
	print(dx_movement)
	print(dy_movement)
	curr_cell = getCell(curr_pos)
	while not neighbours(curr_cell, pos2):
		print(getCell(curr_pos))
		# print(curr_pos[0],curr_pos[1])
		curr_cell = getCell(curr_pos)
		grid[curr_cell[0]][curr_cell[1]] = 1	
		curr_pos = (dx_movement + curr_pos[0], dy_movement + curr_pos[1])
	grid[pos2[0]][pos2[1]] = 1	

# check if given position exists inside the grid
def isValid(pos, grid_size):
	return (pos[0] >= 0 and pos[1] >= 0 and pos[0] < grid_size[0] and pos[1] < grid_size[1])


"""
Gives a list of 

Parameters:
------------
angle_range : (int, int)
	The range of angles that from which the distance readings will be obtained
angle_steps: 
	Granularity of changing the angle of the servo for distance readings

Returns:
------------
distance_measures : [(int,double), (int,double) ....]
	Returns a list with angles and their corresponding distances [(angle, distance), ...]
"""
def getDistanceMeasurements(angle_range, angle_steps):
	fc.servo.set_angle(0)
	time.sleep(2)
	distance_measures = []
	for angle in range(angle_range[0], angle_range[1]+1, angle_steps):
		dis_val =  fc.get_distance_at(angle)
		distance_measures.append((angle*-1,dis_val))
	return distance_measures

"""
Maps a given grid with obstactles (where 1 represents an obstacle)

Parameters:
------------
car_pos : (int, int) 
    The current position of the car in the grid
grid_size : (int, int)
    The grid size in cm
car_direction : int
	The direction at which the car is facing. Can be one of the enum direction: FORWARD, BACKWARD, RIGHT, LEFT
angle_range : (int, int)
	The range of angles that from which the distance readings will be obtained
angle_steps: 
	Granularity of changing the angle of the servo for distance readings

Returns:
------------
grid : np.array 
    A 2D array 1cm^2 squares of the map with 1 representing an obstacle
"""
def mapGrid(car_pos=(5,0), grid_size=(10,10), car_direction=0, angle_range=ANGLE_RANGE, angle_steps=ANGLE_STEPS):
	grid = np.zeros(grid_size)
	distance_measurements = getDistanceMeasurements(angle_range, angle_steps)
	# distance_measurements = [(60,5),(30,6)]
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

# markClearance(cell = (x,y), grid, radius)

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

Returns:
------------
path : [(int,int), (int,int)...]
    A list of cells to visit to get to the goal in the shortest path
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

		def backtrack(self):
			list_pos = []
			curr_node = self
			while not curr_node.isRoot():
				list_pos.append(curr_node.pos)
				curr_node = curr_node.previous
			list_pos.append(curr_node.pos)
			list_pos.reverse()
			return list_pos



	openList = []
	closedList = []
	curr_node = node(None, car_pos, car_pos, goal)
	heapq.heappush(openList, (0, curr_node))
	while (len(openList) > 0):
		curr_node = heapq.heappop(openList)[1]
		closedList.append(curr_node)

		if (curr_node.isGoal()):
			return curr_node.backtrack()

		children = curr_node.adjacentChildren(grid, grid_size)
		# print()
		# print(curr_node.pos)
		for child in children:
			# print("child", child.pos)
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

# Moves car forward by distance amount
def moveForward(distance):
	count_forward = 0
	while (count_forward < 100):
		fc.turn_right(speed)
		count_forward += 1
	fc.stop()

# Turns the car right (90 degrees)
def right():
	count_forward = 0
	while (count_forward < TURN_VALUE):
		fc.forward(speed)
		count_forward += 1
	fc.stop()

# Turns the car left (-90 degrees)
def left():
	count_forward = 0
	while (count_forward < TURN_VALUE):
		fc.backward(speed)
		count_forward += 1
	fc.stop()
		

"""
Moves the car towards the specified direction and distance. 

Parameters:
------------
goal_direction : int
	The direction at which the car has to be moved. Can be one of the enum direction: FORWARD, BACKWARD, RIGHT, LEFT
car_direction : int
	The direction at which the car is facing. Can be one of the enum direction: FORWARD, BACKWARD, RIGHT, LEFT
distance : int
	The amount by which the car should move. This distance depends on the lenth of the cell

Returns:
------------
direction : int
    The direction at which the car is facing at the end of the function. Can be one of the enum direction: FORWARD, BACKWARD, RIGHT, LEFT
"""
def move(goal_direction, car_direction, distance):
	if (goal_direction == direction.FORWARD and car_direction == direction.RIGHT):
		left()
	elif (goal_direction == direction.FORWARD and car_direction == direction.LEFT):
		right()
	elif (goal_direction == direction.FORWARD and car_direction == direction.BACKWARD):
		right()
		right()

	elif (goal_direction == direction.RIGHT and car_direction == direction.BACKWARD):
		left()
	elif (goal_direction == direction.RIGHT and car_direction == direction.FORWARD):
		right()
	elif (goal_direction == direction.RIGHT and car_direction == direction.LEFT):
		right()
		right()

	elif (goal_direction == direction.BACKWARD and car_direction == direction.LEFT):
		left()
	elif (goal_direction == direction.BACKWARD and car_direction == direction.RIGHT):
		right()
	elif (goal_direction == direction.BACKWARD and car_direction == direction.FORWARD):
		right()
		right()

	elif (goal_direction == direction.LEFT and car_direction == direction.FORWARD):
		left()
	elif (goal_direction == direction.LEFT and car_direction == direction.BACKWARD):
		right()
	elif (goal_direction == direction.LEFT and car_direction == direction.RIGHT):
		right()
		right()

	moveForward(distance)
	return(goal_direction)

"""
Takes 'steps' number of steps towards the goal 

Parameters:
------------
path : [(int,int), (int,int)...]
    A list of cells to visit to get to the goal in the shortest path
steps : int
	The number of steps from the given path to take  
car_pos : (int, int) 
    The current position of the car in the grid
car_direction : int
	The direction at which the car is facing. Can be one of the enum direction: FORWARD, BACKWARD, RIGHT, LEFT
distance : int
	The amount by which the car should move. This distance depends on the lenth of the cell

Returns:
------------
car_pos : (int, int) 
    The current position of the car in the grid after the movement
car_direction : int
    The direction at which the car is facing at the end of the function. Can be one of the enum direction: FORWARD, BACKWARD, RIGHT, LEFT
"""
def moveCar(path, steps, car_pos, car_direction, distance):
	curr_pos = car_pos
	for cell in path[:steps]:
		if (cell[0]-curr_pos[0]) == 1:
			car_direction = move(1, car_direction, distance)
		if (cell[0]-curr_pos[0]) == -1:
			car_direction = move(-1, car_direction, distance)
		if (cell[1]-curr_pos[1]) == 1:
			car_direction = move(2, car_direction, distance)
		if (cell[1]-curr_pos[1]) == -1:
			car_direction = move(-2, car_direction, distance)
		curr_pos = cell
	return (car_pos, car_direction)




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
	The direction at which the car is facing. Can be one of the enum direction: FORWARD, BACKWARD, RIGHT, LEFT
"""
def navigate(car_pos=(0,0), goal=(9,9), grid_size=(10,10), car_direction=0):
	while (car_pos != goal):
		grid = mapGrid(car_pos, grid_size, car_direction, ANGLE_RANGE, ANGLE_STEPS)
		# addClearance(grid, CAR_SIZE) # TODO: adds extra space around obstacles to allow for the size of the car
		printGrid(grid, grid_size)
		path = findPath(grid, car_pos, goal, grid_size) # path is a series of cells to visit to get to the goal
		car_pos = goal
		print(path)
		# (car_pos, car_direction) = moveCar(path, STEPS, car_pos, car_direction)

navigate()
# navigate(car_pos=(45,0), goal=(70,70), grid_size=(90,90), car_direction=0)
# right(10)
# path = [(0,1), (0,2), (1,2), (2,2), (2,3), (2,4)]
# moveCar(path, STEPS, (0,0), 2)
# moveForward(1)
# moveForward(1)
# right()
# moveForward(1)
# left()
# moveForward(1)

# print(getDistanceMeasurements(ANGLE_RANGE, ANGLE_STEPS))