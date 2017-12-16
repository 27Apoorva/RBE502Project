#!/usr/bin/env python

import rospy
from nav_msgs.msg import GridCells
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from kobuki_msgs.msg import BumperEvent
from copy import deepcopy
from kobuki_msgs.msg import BumperEvent
from tf.transformations import euler_from_quaternion
import rospy, tf, numpy, math, random, time, os

#turtlebot dimensions
wheel_rad = 3.5 / 100.0 #cm
wheel_base = 23.0 / 100.0 #cm

# threshold that determines if value from costmap is obstacle
OBSTACLE = 92

# reads in global map
def mapCallBack(data):
	global mapData
	global width
	global height
	global mapgrid
	global resolution
	global offsetX
	global offsetY
	mapgrid = data
	resolution = data.info.resolution
	mapData = data.data
	width = data.info.width
	height = data.info.height
	offsetX = data.info.origin.position.x
	offsetY = data.info.origin.position.y
	# print data.info
	# print offsetX
	# print offsetY
	print "map updated"
	publishCells(mapData)
	# makeBoard()

# read goal point from using 2DNavGoal on RViz
def readGoal(goalPos):
	global goalX
	global goalY
	global goalPoint 
	global start
	goalPoint = Point()
	# convert goal point to cell coordinates
	goalX= int(goalPos.pose.position.x/resolution)
	goalY= int(goalPos.pose.position.y/resolution)
	goalPoint.x = goalX
	goalPoint.y = goalY

	# use robot’s current position as start point
	global startPosX
	global startPosY
	global start
	start = PoseStamped()
	# convert start point to cell coordinates
	startPosX = int(pose.position.x/resolution)
	startPosY = int(pose.position.y/resolution)
	start.pose.position.x = startPosX
	start.pose.position.y = startPosY
	print startPosX
	print startPosY



	print goalPos.pose
	print goalX
	print goalY
	# Start Astar
	aStar(Point(startPosX, startPosY, 0), goalPoint)
	


# use RViz 2D pose estimate to set start point
def readStart(startPos):

	global startPosX
	global startPosY
	global start
	start = startPos.pose
	startPosX = int(startPos.pose.pose.position.x/resolution)
	startPosY = int(startPos.pose.pose.position.y/resolution)
	start.pose.position.x = startPosX
	start.pose.position.y = startPosY
	print startPos.pose.pose
	print startPosX
	print startPosY

# early class idea for Nodes for board used for AStar (not used)
class Node:
	def __init__(pose, visited_nodes):
		self.pose = pose
		self.visited_nodes = visited_nodes

# Nodes for board used by AStar
class Node2:
	global goalX
	global goalY
	global goalPoint
	def __init__(self,pose):
		self.pose = deepcopy(pose)
		self.came_from = None
		self.neighbors = [] 
		self.g_score = 0
		self.h_score = distance_between_two_points(self.pose.position, Point(goalX/2, goalY/2, 0)) # initialize with Manhattan distance to goal
		# print self.pose.position
		# print "h score %d" % self.h_score
		self.f_score = 0
		self.aval = 1 # acts as boolean saying if cell is occupied
		self.cost = 0

	# set cell to occupied
	def set_occ(self): 
		self.aval = 0

	# set cost of cell based on cost from cost map
	def set_cost(self, cost):
		self.cost = cost

	# set the node that this node came from
	def set_came_from(self, node):
		self.came_from = node

	def set_g(self, num):
		self.g_score = num
		# print "g score %d" % self.g_score

	def calc_h(self, goal):
		self.h_score = distance_between_two_points(pose.position, Point(goalX/2, goalY/2))
		# print "h score %d" % self.h_score

	def calc_f_score(self):
		self.f_score = self.g_score + self.h_score

	#add  neighbors to this node's list of neighbors from the board
	#only add nodes that are not unknown or not obstacles
	def addNeighbors(self, aboard):
		global width
		global height
		opt_height = height /2
		opt_width = width /2 

#check the four neighbors while making sure that we are not checking outside the board

		if(self.pose.position.x > 0 and (self.pose.position.x < opt_width - 1)
			and self.pose.position.y > 0 and (self.pose.position.y < opt_height - 1 )):

			if(aboard[self.pose.position.x + 1][self.pose.position.y].aval != 0):
				self.neighbors.append(aboard[self.pose.position.x + 1][self.pose.position.y])
			if(aboard[self.pose.position.x][self.pose.position.y + 1].aval != 0):
				self.neighbors.append(aboard[self.pose.position.x][self.pose.position.y + 1])
			if(aboard[self.pose.position.x - 1][self.pose.position.y].aval != 0):
				self.neighbors.append(aboard[self.pose.position.x - 1][self.pose.position.y])
			if(aboard[self.pose.position.x][self.pose.position.y - 1].aval != 0):
				self.neighbors.append(aboard[self.pose.position.x][self.pose.position.y - 1])

		elif(self.pose.position.x == 0 and self.pose.position.y == 0):
			#can't find negative  x and negative y 
			if(aboard[self.pose.position.x + 1][self.pose.position.y].aval != 0):
				self.neighbors.append(aboard[self.pose.position.x + 1][self.pose.position.y])
			if(aboard[self.pose.position.x][self.pose.position.y + 1].aval != 0):
				self.neighbors.append(aboard[self.pose.position.x][self.pose.position.y + 1])

		elif((self.pose.position.x == opt_width -1) and (self.pose.position.y == opt_height -1)):
			#can't find positive x and positive y 
			if(aboard[self.pose.position.x - 1][self.pose.position.y].aval != 0):
				self.neighbors.append(aboard[self.pose.position.x - 1][self.pose.position.y])
			if(aboard[self.pose.position.x][self.pose.position.y - 1].aval != 0):
				self.neighbors.append(aboard[self.pose.position.x][self.pose.position.y - 1])
			
		elif((self.pose.position.x == 0) and (self.pose.position.y == opt_height -1)):
			#can't find negative x and positive y 
			if(aboard[self.pose.position.x + 1][self.pose.position.y].aval != 0):
				self.neighbors.append(aboard[self.pose.position.x + 1][self.pose.position.y])
			if(aboard[self.pose.position.x][self.pose.position.y - 1].aval != 0):
				self.neighbors.append(aboard[self.pose.position.x][self.pose.position.y - 1])

		elif((self.pose.position.x == opt_width -1) and (self.pose.position.y == 0)):
			#can't find positive x and negative y
			if(aboard[self.pose.position.x][self.pose.position.y + 1].aval != 0):
				self.neighbors.append(aboard[self.pose.position.x][self.pose.position.y + 1])
			if(aboard[self.pose.position.x - 1][self.pose.position.y].aval != 0):
				self.neighbors.append(aboard[self.pose.position.x - 1][self.pose.position.y])
			
		elif(self.pose.position.x == 0):
			#can't find negative x
			if(aboard[self.pose.position.x + 1][self.pose.position.y].aval != 0):
				self.neighbors.append(aboard[self.pose.position.x + 1][self.pose.position.y])
			if(aboard[self.pose.position.x][self.pose.position.y + 1].aval != 0):
				self.neighbors.append(aboard[self.pose.position.x][self.pose.position.y + 1])
			if(aboard[self.pose.position.x][self.pose.position.y - 1].aval != 0):
				self.neighbors.append(aboard[self.pose.position.x][self.pose.position.y - 1])
			 
		elif(self.pose.position.y == 0):
			#can't find negative y 
			if(aboard[self.pose.position.x + 1][self.pose.position.y].aval != 0):
				self.neighbors.append(aboard[self.pose.position.x + 1][self.pose.position.y])
			if(aboard[self.pose.position.x][self.pose.position.y + 1].aval != 0):
				self.neighbors.append(aboard[self.pose.position.x][self.pose.position.y + 1])
			if(aboard[self.pose.position.x - 1][self.pose.position.y].aval != 0):
				self.neighbors.append(aboard[self.pose.position.x - 1][self.pose.position.y])
			
		elif(self.pose.position.x == opt_width -1):
			#can't find positive x
			if(aboard[self.pose.position.x][self.pose.position.y + 1].aval != 0):
				self.neighbors.append(aboard[self.pose.position.x][self.pose.position.y + 1])
			if(aboard[self.pose.position.x - 1][self.pose.position.y].aval != 0):
				self.neighbors.append(aboard[self.pose.position.x - 1][self.pose.position.y])
			if(aboard[self.pose.position.x][self.pose.position.y - 1].aval != 0):
				self.neighbors.append(aboard[self.pose.position.x][self.pose.position.y - 1])
			 
		elif(self.pose.position.y == opt_height -1):
			#can't find positive y
			if(aboard[self.pose.position.x + 1][self.pose.position.y].aval != 0):
				self.neighbors.append(aboard[self.pose.position.x + 1][self.pose.position.y])
			if(aboard[self.pose.position.x - 1][self.pose.position.y].aval != 0):
				self.neighbors.append(aboard[self.pose.position.x - 1][self.pose.position.y])
			if(aboard[self.pose.position.x][self.pose.position.y - 1].aval != 0):
				self.neighbors.append(aboard[self.pose.position.x][self.pose.position.y - 1])

		#set the gscore for the neighbors after adding them
		set_g_neighbors(self)
		
		# print "self.pose.position.x %d" % self.pose.position.x
		# print "self.pose.position.y %d" % self.pose.position.y
# set the gscore of the neighbors of the given node based on a set criteria
def set_g_neighbors(node):
	#check if this is the first node
	if (node.came_from is None):
		#loop through the neighbors and calculate the cost and set the gscore
		for i in range (0, len(node.neighbors)):
			#print " first node ever"
			cost = (int(node.neighbors[i].cost/4))#cost of obstacles
			node.neighbors[i].set_g(node.g_score + 1 + cost)
	else: #this is not the first node
				#loop through the neighbors and calculate the costs and set the gscore
		for i in range (0, len(node.neighbors)):
			turnCost = 0 #initialize the turn cost to zero
			#check if a turn is needed to get to this node
			if(node.came_from.pose.position.x == node.pose.position.x and node.pose.position.x == node.neighbors[i].pose.position.x):
				pass
			elif(node.came_from.pose.position.y == node.pose.position.y and node.pose.position.y == node.neighbors[i].pose.position.y):
				pass
			else:
				#print " I am a  turn"
				turnCost = 4 #set a turn cost 
			cost = (int(node.neighbors[i].cost/4))
			#calculate the gscore
			thisGScore = node.g_score + 1 + cost + turnCost
			#print " This GSCORE {} ".format(thisGScore)
			node.neighbors[i].set_g(thisGScore)
			# print "This neighbor's gscore {} ".format(node.neighbors[i].g_score)



#add neighbors to the frontier, this function is no longer used
def add_to_frontier(node, frontier):
	for x in range(0,len(node.neighbors)):
		node.neighbors[x].set_g(node.g_score+1)
		frontier.append(node.neighbors[x])

		
# takes in two positions and calculates the distance between them
def distance_between_two_points(position1, position2):
	"""
	# straight line distance
	x = int((position1.x - position2.x)**2)
	y = int((position1.y - position2.y)**2)
	dist = int((x + y)**0.5)
	# print x
	# print y
	"""
	# Manhattan distance
	x = abs(position1.x - position2.x)
	y = abs(position1.y - position2.y)
	dist = x + y
	
	# print "dist %d" % dist
	return dist

"""
# attempt to put makeBoard into own function to be called when map updates
# not used
def makeBoard():
	global board
	global opt_board

	k = 0
	currentx = 0
	currenty = 0

	print len(mapData)
	# add visible values to board
	while(currenty < height):
		while(currentx < width):
			c = currentx + currenty*width
			mapstuff = mapData[c]
			#print mapstuff
			thisPoint = Point(currentx, currenty, 0)
			thisPose = Pose()
			thisPose.position = thisPoint
			thisNode = Node2(thisPose)
			thisNode.set_cost(mapstuff)
			if (mapstuff < OBSTACLE or mapstuff == -1):
				# thisNode.pose = thisPose
				board[currentx][currenty] = thisNode
				#print "board cell"
				#print board[currentx][currenty].pose.position 
			else:
				board[currentx][currenty] = thisNode
				board[currentx][currenty].set_occ()
			
			#print board[currentx][currenty].aval
			currentx = currentx + 1
		currentx = 0
		currenty = currenty + 1

	print"finished the board"
	print len(board)
	currentx = 0
	currenty = 0
	## optimize the board
	print "optimizing the board"
	cx = 0
	cy = 0
	opt_board = [[firstNode for x in range(opt_height)] for y in range(opt_width)]
	while(currenty < opt_height):
		while(currentx < opt_width):
			opt_cost = int((board[cx][cy].cost + board[cx + 1][cy].cost+ board[cx][cy+1].cost+ board[cx+1][cy+1].cost)/4)
			thisPoint = Point(currentx, currenty, 0)
			thisPose = Pose()
			thisPose.position = thisPoint
			thisNode = Node2(thisPose)
			#print opt_cost
			thisNode.set_cost(opt_cost)
			if(opt_cost > 45):
				thisNode.set_occ()
			opt_board[currentx][currenty] = thisNode
			# print opt_board[currentx][currenty].aval
			currentx = currentx + 1
			cx = cx + 2
		currentx = 0
		cx = 0
		cy = cy + 2
		currenty = currenty + 1
			

	print len(opt_board)
"""

# start: point	goal: pose	each node is a pose
def aStar(startPoint,goal):
	global mapData
	global width
	global height
	global mapgrid
	global resolution
	global offsetX
	global offsetY
	global opt_board
	global visited
	global startPosX
	global startPosY

	global waypointstwo
	global pose

	global count
	global bumperFlag

	bumperFlag = False

	publishCells(mapData)

	startFlag = True

	currentx = 0
	currenty = 0
	
	#dimensions of optimized board
	opt_height = height/2
	opt_width  = width/2
	
	frontier = [] # unvisited nodes that have been opened
	visited = [] # nodes that the robot has travelled to

	visit_grid = [0]*opt_height*opt_width
	frontier_grid = [0]*opt_height*opt_width

	firstPoint = Point(currentx, currenty, 0)
	firstPose = Pose()
	firstPose.position = firstPoint
	firstNode = Node2(firstPose)
	board = [[firstNode for x in range(0,height)] for y in range(0,width)] # the gridcells sent to the program
	
	
	# start_pose = start.pose
	# start_point = Point(0,0,0)
	start_pose = Pose()
	# start_pose.position.x = startPosX
	# start_pose.position.y = startPosY
	# start_pose.position.x = pose.position.x
	# start_pose.position.y = pose.position.y
	# start_point.x = startPoint.x
	# start_point.y = startPoint.y

	# create starting node
	start_pose.position.x = startPoint.x
	start_pose.position.y = startPoint.y
	start_node = Node2(start_pose)
	# start_node = Node2(deepcopy(pose))

	print "goal x: {}, goal y: {}".format(goal.x, goal.y)
	print "start x: {}, start y: {}".format(startPoint.x, startPoint.y)
	# print goal
	k = 0
	currentx = 0
	currenty = 0

	# makeBoard()
	"""
	# former code used to read in and optimize map
	# new, faster version below
	print len(mapData)
	# add visible values to board
	while(currenty < height):
		while(currentx < width):
			c = currentx + currenty*width
			mapstuff = mapData[c]
			#print mapstuff
			thisPoint = Point(currentx, currenty, 0)
			thisPose = Pose()
			thisPose.position = thisPoint
			thisNode = Node2(thisPose)
			thisNode.set_cost(mapstuff)
			if (mapstuff < OBSTACLE or mapstuff == -1):
				# thisNode.pose = thisPose
				board[currentx][currenty] = thisNode
				#print "board cell"
				#print board[currentx][currenty].pose.position 
			else:
				board[currentx][currenty] = thisNode
				board[currentx][currenty].set_occ()
			
			#print board[currentx][currenty].aval
			currentx = currentx + 1
		currentx = 0
		currenty = currenty + 1

	print"finished the board"
	print len(board)
	currentx = 0
	currenty = 0
	## optimize the board
	print "optimizing the board"
	cx = 0
	cy = 0
	opt_board = [[firstNode for x in range(opt_height)] for y in range(opt_width)]
	while(currenty < opt_height):
		while(currentx < opt_width):
			opt_cost = int((board[cx][cy].cost + board[cx + 1][cy].cost+ board[cx][cy+1].cost+ board[cx+1][cy+1].cost)/4)
			thisPoint = Point(currentx, currenty, 0)
			thisPose = Pose()
			thisPose.position = thisPoint
			thisNode = Node2(thisPose)
			#print opt_cost
			thisNode.set_cost(opt_cost)
			if(opt_cost > OBSTACLE):
				thisNode.set_occ()
			opt_board[currentx][currenty] = thisNode
			# print opt_board[currentx][currenty].aval
			currentx = currentx + 1
			cx = cx + 2
		currentx = 0
		cx = 0
		cy = cy + 2
		currenty = currenty + 1
			

	print len(opt_board)
	"""

	# reading in board and optimizing it
	cx = 0
	cy = 0
	opt_board = [[firstNode for x in range(opt_height)] for y in range(opt_width)]
	print "making board"
	while(cy < height):
		while(cx < width):
			# combine 4 cells (2x2) into one
			# cost of cell average of the 4 cells
			opt_cost = int((mapData[cx+cy*width] + mapData[(cx + 1)+cy*width]+ mapData[cx+(cy+1)*width]+ mapData[(cx+1)+(cy+1)*width])/4)
			thisPoint = Point(currentx, currenty, 0)
			thisPose = Pose()
			thisPose.position = thisPoint
			thisNode = Node2(thisPose)
			#print opt_cost
			thisNode.set_cost(opt_cost)
			# if the node is greater than obstacle threshold or in
			# unknown space, count it as an obstacle
			if((opt_cost > OBSTACLE) or (opt_cost == -1)):
				thisNode.set_occ()
			opt_board[currentx][currenty] = thisNode
			# print opt_board[currentx][currenty].aval
			currentx = currentx + 1
			cx = cx + 2
		currentx = 0
		cx = 0
		cy = cy + 2
		currenty = currenty + 1
	print "board complete"		

	print len(opt_board)

	start_node.pose.position.x = int(start_node.pose.position.x /2)
	start_node.pose.position.y = int(start_node.pose.position.y /2)

	while(opt_board[start_node.pose.position.x][start_node.pose.position.y].cost >= OBSTACLE):
		print "in wall"
		driveStraight(-0.25, 0.5)
		# if SLAM makes robot think it’s in a wall, try to get it out
		  
	current = start_node
	current.calc_f_score()
	print "current"
	print current.pose.position
	frontier.append(current)

	target = Point()
	target.x = int(goal.x /2)
	target.y = int(goal.y /2)
	target.z = 0
	print "target"
	print target
	print "running aStar"
	flag = 0
#TODO check implemntation of astar
	while(len(frontier) > 0 and flag ==0):
		# sorted(frontier, key=lambda Node2: Node2.f_score)
		current = frontier[0]
		frontier.remove(current)
		#print "current position"
		#print current.aval
		"""
		print "XXXXXXXXXXXXXXXXXXXXXXXXXXXXX"
		print "current h %d" % current.h_score
		print "current g %d" % current.g_score
		print "current f %d" % current.f_score
		"""
		# print "current g %d" % current.g_score

		if(current.pose.position == target):
			print "found the goal"
			flag = 1
			continue

		visited.append(current)

		for q in range(len(visited)):
			visit_grid[visited[q].pose.position.x+(visited[q].pose.position.y*opt_width)] = 100
		# print len(visited)
		publishVisited(visit_grid)

		for v in range(len(frontier)):
			frontier_grid[frontier[v].pose.position.x+(frontier[v].pose.position.y*opt_width)] = 100
		publishFrontier(frontier_grid)
		# print current.f_score
		current.addNeighbors(opt_board)
		#print "neihgbors size"
		#print len(current.neighbors)
		#print len(current.neighbors)
		for i in range (0, len(current.neighbors)):
			neighbor = current.neighbors[i]

			if(neighbor in visited):
				continue
			# tmp_g_score = deepcopy(current.g_score) + distance_between_two_points(current.pose.position, neighbor.pose.position)
			tmp_g_score = deepcopy(current.g_score) + 1
			if(neighbor not in frontier):
				frontier.append(neighbor)
			elif(tmp_g_score >= neighbor.g_score):
				continue
			neighbor.set_came_from(current)
			#neighbor.set_g(tmp_g_score)
			neighbor.calc_f_score()

		# sorted(frontier, key=lambda Node2: Node2.f_score)
		# sort by h then f score to get the lowest f score that’s
		# closest to the goal
		frontier.sort(key=lambda Node2: Node2.h_score)
		frontier.sort(key=lambda Node2: Node2.f_score)
		#print "NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN"
		#for p in range(0, len(frontier)):
			#print frontier[p].f_score

	print "testing ........testing"	

	# if the program can’t find a path, exit AStar and find new frontier
	if(abs(current.pose.position.x - target.x) > 3 and abs(current.pose.position.y - target.y) > 3):
		print "error can't find path ......"
		count = count + 1
		if (count >= 1):
			# rotate (180)
			# driveStraight(0.25, 0.5)
			count = 0
		return
	count = 0
	print "reconstructing the path"
	# create path by following “came_from” attribute until it is None
	pathNodes = []
	while (current.came_from is not None):
		pathNodes.append(current)
		current = current.came_from
	print "reconstructed path"
	# take list of nodes and convert into a path
	path = nodesToPath(pathNodes)

	publishPath(path)
	print len(path)
	# if no path was able to be made, turn around and move to position
	# to possibly be better able to make a path
	if(len(path)==0):
		print "error: 0 path"
		rotate (180)
		driveStraight(0.25, 0.5)
		Return
	# find waypoints based on turns in path
	waypointstwo = findWaypoints(path)
	print len(waypointstwo)
	publishWaypoints()
	#print "publishing path's waypoints"
	#publishPathWay(waypoints)
	# fixWaypoints()
	print "Done"
	# have robot drive to each waypoint sequentially
	for i in range(0,len(waypointstwo)):
		print "X: {}, Y: {}".format(waypointstwo[i].x, waypointstwo[i].y)
	# publishDrive()
	waypointstwo.reverse() # waypoints order is reversed
	# used 1 because goal is waypointstwo[0]
	for t in range(1,len(waypointstwo)):
		movePose = PoseStamped()
		movePose.pose.position.x = deepcopy(waypointstwo[t].x)
		movePose.pose.position.y = deepcopy(waypointstwo[t].y)
		# drive_pub.publish(movePose)
		# if an obstacle is hit, stop trying to drive this path
		if(bumperFlag):
			return
		navToPose(movePose)

	"""
	for a in range(1,len(waypoints)):
		movePose = Pose()
		movePose.position.x = waypoints[a].x
		movePose.position.y = waypoints[a].y
		navToPose(movePose)
	"""

	
	

#convert list of path nodes to path points
def nodesToPath(pathNodes):
	path = []
	print "converting nodes to points"
	for i in range (0, len(pathNodes)):
		path.append(pathNodes[i].pose.position)

	return path
	

#TODO
#produce list of waypoints from a given path 
# waypoints are set on turns
def findWaypoints(path):
	global waypointstwo
	waypointstwo = []
	waypointstwo.append(path[0])
	for p in range(1,len(path)-1):
		if(path[p-1].x == path[p].x and path[p].x == path[p+1].x):
			continue
		elif(path[p-1].y == path[p].y and path[p].y == path[p+1].y):
			continue
		else:
			waypointstwo.append(path[p])
	waypointstwo.append(Point(goalPoint.x/2,goalPoint.y/2,0))
	return waypointstwo
# former method used to fix waypoints for driving and RViz
def fixWaypoints():
	global waypointstwo
	res = resolution * 2
	
	for a in range (0, len(waypointstwo)):
		pos = deepcopy(waypointstwo[a])
		point=Point()
		point.x=(deepcopy(pos.x)*res)+offsetX + (0.5 * res) 
		point.y=(deepcopy(pos.y)*res)+offsetY - (.5 * res) 
		point.z = 0
		waypointstwo[a] = deepcopy(point)


		
#publish waypoints as gridcells
def publishWaypoints():
	global pubway
	print "publishing waypoints"
	path_cells = GridCells()
	path_cells.header.frame_id = 'map'
	res = resolution * 2 
	path_cells.cell_width = res
	path_cells.cell_height = res

	for x in xrange(0,len(waypointstwo)):
		pos = waypointstwo[x]
		point=Point()
		point.x=(pos.x*res)+offsetX + (0.5 * res) # added secondary offset 
		point.y=(pos.y*res)+offsetY - (.5 * res) # added secondary offset ... Magic ?
		point.z=0
		path_cells.cells.append(point)

	pubway.publish(path_cells)

""""
#publishes a waypoints path with a nav message
def publishPathWay(waypoints):
	global pubwaypath
	print "publishing waypoints on path"
	waypath = Path()
	waypath.header.frame_id = 'map'

	for x in xrange(0,len(waypoints)):
		pos = waypoints[x]
		point=Point()
		point.x=(pos.x*resolution)+offsetX + (0.5 * resolution)
		point.y=(pos.y*resolution)+offsetY - (.5 * resolution) 
		point.z=0
		posePath = Pose()
		posePath.position = point
		waypath.poses.append(posePath)

	pubwaypath.publish(waypath)
"""
# former method of telling the robot to drive
# not used
def publishDrive():
	# global waypointstwo
	global drive_pub
	global waypoint_flag
	print "publishing driving"
	
	waypoint_flag = True
	"""
	for a in range (1, len(waypointstwo)):
		while not waypoint_flag:
			pass
		print "driving to waypoint {}".format(a)
		movePose = PoseStamped()
		movePose.pose.position.x = deepcopy(waypointstwo[a].x)
		movePose.pose.position.y = deepcopy(waypointstwo[a].y)
		drive_pub.publish(movePose)
		waypoint_flag = False
	"""

	movePose = PoseStamped()
	movePose.pose.position.x = deepcopy(waypointstwo[1].x)
	movePose.pose.position.y = deepcopy(waypointstwo[1].y)
	drive_pub.publish(movePose)

		
# publish the path grid cells
def publishPath(path):
	global pubpath 
	print "publishing path"


	path_cells = GridCells()
	path_cells.header.frame_id = 'map'
	res = resolution * 2
	path_cells.cell_width = res 
	path_cells.cell_height = res

	for x in xrange(0,len(path)):
		pos = path[x]
		point=Point()
		point.x=(pos.x*res)+offsetX + (0.5 * res) # added secondary offset 
		point.y=(pos.y*res)+offsetY - (.5 * res) # added secondary offset ... Magic ?
		point.z=0
		path_cells.cells.append(point)

	pubpath.publish(path_cells)

#publishes map to rviz using gridcells type

def publishCells(grid):
	global pub
	print "publishing"

	# resolution and offset of the map
	k= 0
	cells = GridCells()
	cells.header.frame_id = 'map'

	cells.cell_width = resolution 
	cells.cell_height = resolution

	for i in range(1,height): #height should be set to hieght of grid
		k=k+1
		for j in range(1,width): #width should be set to width of grid
			k=k+1
			#print k # used for debugging
			# if (grid[k] == 100):
			if (grid[k] > OBSTACLE):
				point=Point()
				point.x=(j*resolution)+offsetX + (1.5 * resolution) # added secondary offset 
				point.y=(i*resolution)+offsetY - (.5 * resolution) # added secondary offset ... Magic ?
				point.z=0
				cells.cells.append(point)
	pub.publish(cells)

# publish cells that have been explored by AStar
def publishVisited(grid):
	global pubvisited
	# print "publishing visit"

	# resolution and offset of the map
	k=0
	cells = GridCells()
	cells.header.frame_id = 'map'
	res = resolution * 2
	cells.cell_width = res 
	cells.cell_height = res

	for i in range(1,height/2): #height should be set to hieght of grid
		k=k+1
		for j in range(1,width/2): #width should be set to width of grid
			k=k+1
			#print k # used for debugging
			if (grid[k] == 100):
				point=Point()
				point.x=(j*res)+offsetX + (1.5 * res) # added secondary offset 
				# point.x=(j*res) + (.5 * res)
				point.y=(i*res)+offsetY - (.5 * res) # added secondary offset ... Magic ?
				# point.y=(i*res) + (.5 * res)
				point.z=0
				cells.cells.append(point)
	pubvisited.publish(cells)

# publish cells that AStar may expand
def publishFrontier(grid):
	global pubfrontier
	# print "publishing frontier"

	# resolution and offset of the map
	k=0
	cells = GridCells()
	cells.header.frame_id = 'map'
	res = resolution * 2
	cells.cell_width = res 
	cells.cell_height = res

	for i in range(1,height/2): #height should be set to hieght of grid
		k=k+1
		for j in range(1,width/2): #width should be set to width of grid
			k=k+1
			#print k # used for debugging
			if (grid[k] == 100):
				point=Point()
				point.x=(j*res)+offsetX + (1.5 * res) # added secondary offset 
				# point.x=(j*res) + (.5 * res)
				point.y=(i*res)+offsetY - (.5 * res) # added secondary offset ... Magic ?
				# point.y=(i*res) + (.5 * res)
				point.z=0
				cells.cells.append(point)
	pubfrontier.publish(cells)

# publish the new frontier (i.e., the goal the robot found when looking for
# new cells to explore)
def publishFrontier2(grid):
	global pubfrontier2
	# print "publishing frontier"

	# resolution and offset of the map
	k=0
	cells = GridCells()
	cells.header.frame_id = 'map'
	res = resolution
	cells.cell_width = res 
	cells.cell_height = res

	for i in range(1,height): #height should be set to hieght of grid
		k=k+1
		for j in range(1,width): #width should be set to width of grid
			k=k+1
			#print k # used for debugging
			if (grid[k] == 100):
				point=Point()
				point.x=(j*resolution)+offsetX + (1.5 * resolution) # added secondary offset 
				# point.x=(j*res) + (.5 * res)
				point.y=(i*resolution)+offsetY - (.5 * resolution) # added secondary offset ... Magic ?
				# point.y=(i*res) + (.5 * res)
				point.z=0
				cells.cells.append(point)
	pubfrontier2.publish(cells)		   




# XXXXXXXXXXXXXXXXXXXXXX MOVEMENT XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

#keeps track of current location and orientation
def tCallback(event):
	global pose
	odom_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))
	(position, orientation) = odom_list.lookupTransform('map','base_footprint', rospy.Time(0))
	pose.position.x=position[0]
	pose.position.y=position[1]
	odomW = orientation
	q = [odomW[0], odomW[1], odomW[2], odomW[3]]
	roll, pitch, yaw = euler_from_quaternion(q)
	pose.orientation.z = yaw

# iterates through waypoints and tells robot to go to each
# not used
def navToGoal():
	global waypoints
	for pose in waypoints:
		navToPose(pose)

#publish twist messages to move the robot
def publishTwist(lin_Vel, ang_Vel):
	global pubmove
	msg = Twist()
	msg.linear.x = lin_Vel
	msg.angular.z = ang_Vel
	pubmove.publish(msg)
	rospy.sleep(0.15) #don't overload the master

#navigate to the provided pose
def navToPose(goalmove):

	global pose
	global waypoint_flag
	global nextFlag

	# convert goal position to global coordinates
	goalmove.pose.position.x = goalmove.pose.position.x * (resolution*2) + offsetX
	goalmove.pose.position.y = goalmove.pose.position.y * (resolution*2) + offsetY

	print "goal x: {}".format(goalmove.pose.position.x)
	print "goal y: {}".format(goalmove.pose.position.y)
	print "current x: {}".format(pose.position.x)
	print "current y: {}".format(pose.position.y)
	print "current rotation: {}".format(math.degrees(pose.orientation.z))

	#calculate goal coordinates
	quat = goalmove.pose.orientation
	q = [quat.x, quat.y, quat.z, quat.w]
	roll, pitch, yaw = euler_from_quaternion(q)
	desiredY = goalmove.pose.position.y
	desiredX = goalmove.pose.position.x
	desiredT = math.degrees(yaw)

	#calculate current coordinates

	currentX = pose.position.x
	currentY = pose.position.y
	currentT = math.degrees(pose.orientation.z)

	#calculate necessary displacement
	translateX = desiredX - currentX
	translateY = desiredY - currentY
	rotateT = desiredT - currentT

	print "desiredX: {}".format(translateX)
	print "desiredY: {}".format(translateY)
	print "currentT: {}".format(currentT)
	print "desiredT: {}".format(desiredT)

	#get angle of displacement vector
	initialTurn = math.degrees(math.atan2(translateY, translateX)) - currentT
	#get length of displacement vector
	distance = math.sqrt(pow(translateX, 2) + pow(translateY, 2))
	"""
	print currentX
	print currentY
	print desiredX
	print desiredY
	"""
	#reverse the initial turn angle and go to the desired angle
	# finalTurn = rotateT - initialTurn
	finalTurn = goalmove.pose.orientation.z - initialTurn
	print "initialTurn {}".format(initialTurn)
	print "distance {}".format(distance)
	print "finalTurn {}".format(finalTurn)
	rotate(initialTurn)
	time.sleep(3)
	# driveStraight(0.25, distance)
	"""
	if(distance > 0.5):
		driveStraight(0.25, 0.5)
	else:
		driveStraight(0.25, distance)
	"""
	driveStraight(0.25, distance)
	time.sleep(3)

	# if the robot is at it's goal, exit this and find a new frontier point
	"""
	if(abs(pose.position.x - goal.pose.position.x) > .1 and abs(pose.position.y - goal.pose.position.y) > .1):
		aStar(Point(int((pose.position.x-offsetX)/resolution), int((pose.position.y-offsetY)/resolution), 0), goalPoint)
		print "rerunning aStar"
	"""
	print "done with this goal"
	nextFlag = True
	
	# rotate(finalTurn)
	# waypoint_flag = True

#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
	# global pose
	initialX = pose.position.x
	initialY = pose.position.y
	print "driving forward {} distance".format(distance)
	while not rospy.is_shutdown():
		currentX = pose.position.x
		currentY = pose.position.y
		#calculate how far the turtlebot has traveled
		currentDistance = math.sqrt(pow(initialX - currentX, 2) + pow(initialY - currentY, 2))
		print currentDistance
		if (currentDistance >= distance):
			publishTwist(0, 0)
			print "done moving forward"
			break
		else:
			#smooth the speed
			publishTwist(speed, 0)
		if(bumperFlag and speed > 0):
			publishTwist(0,0)
			rospy.sleep(2)
			break

# limits the given value to fall between -180 and +180
def limTo180(x):
	while x > 180:
		x = x - 360
	while x < -180:
		x = x + 360
	return x

# robot rotates a certain angle from its current rotation
def rotate(angle):
	# global pose
	#target equals current plus displacement
	targetAngle = limTo180(math.degrees(pose.orientation.z) + angle)
	while not rospy.is_shutdown():
		print "orientation: {}".format(pose.orientation.z)
		currentAngle = math.degrees(pose.orientation.z)
		#error equals target minus current
		error = limTo180(targetAngle - currentAngle)
		if (abs(error) < 2):
			publishTwist(0, 0)
			print "done rotating"
			break
		if error <0 :
			vel = -0.55
		else:
			vel = 0.55
		#if error is negative, make rotation negative
		# if error < 0:
			# print "rotated too far"
			# vel = vel * -1
		publishTwist(0, vel)

# used to rerun AStar
# not used
def localCostReRun(stuff):
	global startFlag
	"""
	print "costMap update"
	if (start):
		print int(pose.position.x/resolution)
		print int(pose.position.y/resolution)
		aStar(Point(int(pose.position.x/resolution), int(pose.position.y/resolution), 0), goalPoint)
	"""


# XXXXXXXXXXXXXXXXXXXXXXXX FINDING FRONTIER XXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

# get the indices of cells immediately adjacent to the given index
def neighbors(i):
	neighborIndex = [i-1, i+1, i+width, i-width]
	# filter OutOfRange values
	return [i for i in neighborIndex if i >= 0 and i < width*height]

# see if the neighbors are adjacent to some known value cell
def checkLocalObstacles(neighbor):
	for twiceremoved in neighbors(neighbor):
		if mapData[twiceremoved] >= 0 and mapData[twiceremoved] < OBSTACLE:
			return True
	return False

# iterate through a cell’s neighbors, see if they are considered frontier
def addNeighborsFrontier(frontier):
	change = 0 # have there been any nodes added?
	for index in frontier:
		for neighbor in neighbors(index): # BFS
			if mapData[neighbor] == -1: # if also unknown
				if checkLocalObstacles(neighbor) and neighbor not in frontier: # and neighbors are on a boundary
					frontier.append(neighbor) # save it
					change = 1 # nodes have been added
	return change # BFS is still going or has completed

# generate a list of frontiers
def discoverFrontier():
	frontierlist = []
	# for every map cell
	for i, cell in enumerate(mapData):
		#if part of another frontier then skip
		if len(frontierlist) and i in frontierlist[-1]:
			Continue
		#if unknown and close to a known cell
		if cell == -1 and checkLocalObstacles(i):
			frontier = [i] # start a new frontier
			while(addNeighborsFrontier(frontier)): # BFS
				continue
			frontierlist.append(frontier) # save this frontier
	frontierlist.sort(key=len, reverse=True) # sort frontier list
	return frontierlist

# returns the new area that the robot will drive to to discover
def getFrontier(frontierlist):
	# adds some randomness to getting a new frontier
	# so that robot doesn’t search same frontier if problem arises
	rando = random.randint(0,3)
	if(rando < len(frontierlist)):	
		frontier = frontierlist[rando]
	else:
		rando = random.randint(0,len(frontierlist))
		frontier = frontierlist[rando]
	
	print "frontier length : {}".format(len(frontier))
	frontierpublisher = []
	print len(frontierpublisher)
	for i in range(0,width*height-1):
		if(i in frontier):
			# frontierpublisher[i] = 100
			frontierpublisher.append(100)
		else:
			# frontierpublisher[i] = 0
			frontierpublisher.append(0)
	publishFrontier2(frontierpublisher)
	print frontier[0]
	return frontier[0]

# This function sequentially calls methods to perform a trajectory.
def executeTrajectory():
	global bumperFlag
	print "hit wall"
	publishTwist(0,0)
	driveStraight(-0.25, 0.5)
	rospy.sleep(0.1)
	bumperFlag = True


# Bumper Event Callback function
def readBumper(msg):
	if (msg.state == 1):
		executeTrajectory()


#Main handler of the project
def run():
	global pub
	global pubpath
	global pubvisited
	global pubfrontier
	global pubfrontier2
	global pubway
	global pubwaypath

	global pubmove
	global pose
	global odom_list
	global drive_pub

	global start
	global gPoint
	global goalX
	global goalY
	global goalPoint

	global nextFlag
	global count


	count = 0
	start = False
	pose = Pose()

	# subscribers and publishers
	rospy.init_node('lab3')
	pubwaypath = rospy.Publisher("/waypointpath", Path, queue_size=1)
	# sub = rospy.Subscriber("/map", OccupancyGrid, mapCallBack)
	pub = rospy.Publisher("/occupancy", GridCells, queue_size=1)  
	pubpath = rospy.Publisher("/path", GridCells, queue_size=1) # you can use other types if desired
	pubway = rospy.Publisher("/waypoints", GridCells, queue_size=1)
	pubvisited = rospy.Publisher("/visited", GridCells, queue_size=1)
	pubfrontier = rospy.Publisher("/frontier", GridCells, queue_size=1)
	pubfrontier2 = rospy.Publisher("/frontier2", GridCells, queue_size=1)
	start_sub = rospy.Subscriber('initialpose/RBE', PoseWithCovarianceStamped, readStart, queue_size=1) #change topic for best results
	goal_sub = rospy.Subscriber('move_base_simple/goal/RBE', PoseStamped, readGoal, queue_size=1) #change topic for best results
	# print "made it this far"
	obs_expand_sub = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, mapCallBack)
	drive_pub = rospy.Publisher('waypointgoal/RBE', PoseStamped, queue_size=1)
	drive_sub = rospy.Subscriber('waypointgoal/RBE', PoseStamped, navToPose, queue_size=1)
	# movement stuff
	pubmove = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, None, queue_size=10)
	rospy.Timer(rospy.Duration(.01), tCallback)
	odom_list = tf.TransformListener()
	
	publocal = rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, localCostReRun)

	bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, readBumper, queue_size=1)


	# wait a second for publisher, subscribers, and TF
	rospy.sleep(2)

	# initial rotation to give a general idea of surroundings
	# only rotates 90 degrees to each side in order to avoid problems with 
# SLAM
	rotate(90)
	time.sleep(2)
	rotate(-90)
	time.sleep(2)

	rotate(-90)
	time.sleep(2)
	rotate(90)

	# main loop
	while (1 and not rospy.is_shutdown()):

		nextFlag = False

		rospy.sleep(2)  
		# find new frontiers
		frontierlist = discoverFrontier()
		if (len(frontierlist[0]) < 1):
			# if big enough frontier isn’t found, robot is done
			print "frontier too small"
			Break
		# pick frontier to go to
		goalIndex = getFrontier(frontierlist)
		gPoint = Point()
		holder = deepcopy(goalIndex)
		holderCount = 0
		# convert from index to coordinates
		while(holder > width):
			holder = holder - width
			holderCount = holderCount + 1
		gPoint.x = holder*resolution + offsetX
		gPoint.y = holderCount*resolution + offsetY
		goalX = int((gPoint.x-offsetX)/(resolution))
		goalY = int((gPoint.y-offsetY)/(resolution))
		goalPoint = Point()
		goalPoint.x = goalX
		goalPoint.y = goalY
		goalPoint.z = 0
		
		# start point as position of robot
		sPoint = Point()
		sPoint.x = int((pose.position.x-offsetX)/resolution)
		sPoint.y = int((pose.position.y-offsetY)/resolution)
		sPoint.z = 0

		print "sPoint x: {}, y: {}".format(sPoint.x, sPoint.y)
		print "goalPoint x: {}, y: {}".format(goalPoint.x, goalPoint.y)
		
		# run AStar
		aStar(sPoint, goalPoint)

		time.sleep(2)
		rospy.sleep(2)
		print("Complete")
	print "map complete"


if __name__ == '__main__':
	try:
		run()
	except rospy.ROSInterruptException:
		pass


