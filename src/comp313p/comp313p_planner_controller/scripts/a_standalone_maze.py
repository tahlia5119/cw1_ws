#! /usr/bin/env python

# Import the needed types.
from comp313p_planner_controller.occupancy_grid import OccupancyGrid
from comp313p_planner_controller.astar_planner import APlanner
import numpy as np

# Create the occupancy grid. Syntax is: number of cells in X, number of cells in Y,
# length of each cell in m
occupancyGrid = OccupancyGrid(21, 21, 0.5)

# The cells are indexed starting from 0.
# Set the state of the cells in the range [11,1]-[11,19] to be occupied.
# This corresponds to the "easy case" in the lectures

x_cells = [[5,11]*3,[0,1,2,5,8,11,14,15,16,17],[5,8,17]*2,[2,5,8,9,10,11,14,15,16,17],[2,8,14,17]*2,[0,1,2,3,4,5,8,11,12,13,14,17,18,19,20],[11,17],[17],[2,5,8,17],[2,5,8,11,12,13,14,15,16,17],[2,5,6,7,8,14,17],[2,3,4,5,8,14,17],[8,9,10,11,12,13,14,17],[8,17],[2,5,8,17],[2,5,11,14]*2]
x_cells = np.concatenate(x_cells)
y_cells = [[0]*2,[1]*2,[2]*2,[3]*10,[4]*3,[5]*3,[6]*10,[7]*4,[8]*4,[9]*15,[10]*2,[11],[12]*4,[13]*10,[14]*7,[15]*7,[16]*8,[17]*2,[18]*4,[19]*4,[20]*4]
y_cells = np.concatenate(y_cells)

for x,y in zip(x_cells,y_cells):
    occupancyGrid.setCell(x, y, 1)

# Start and goal cells
start = (0, 0)
goal = (20, 20)

# Create the planner. The first field is the title which will appear in the
# graphics window, the second the occupancy grid used.
print('Heuristics:\n1. Zero\n2. Constant\n3. Euclidean\n4. Octile\n5. Manhattan\n6. Diagonal\n') 
try:
    heuristic = int(raw_input('Please enter the number of the desired heuristic: '))
    weight = float(raw_input('Please enter the desired weight: '))
    constant = 0
    if heuristic == 2:
	loop = True  
        constant = float(raw_input('Please enter a value greater than 0 for the desired constant: '))
	while(loop):
	     if(constant>0):
		loop = False
	     else:
		print('Invalid input. Please try again.')
	 
except ValueError:
    print('Invalid input. Please try again.')

planner = APlanner('A* Search Algorithm', occupancyGrid, heuristic, weight, constant);

# This causes the planner to slow down and pause for things like key entries
planner.setRunInteractively(True)

# This specifies the height of the window drawn showing the occupancy grid. Everything
# should scale automatically to properly preserve the aspect ratio
planner.setWindowHeightInPixels(400)

# Search and see if a path can be found. Returns True if a path from the start to the
# goal was found and False otherwise
goalReached = planner.search(start, goal)

# Extract the path. This is based on the last search carried out.
path = planner.extractPathToGoal()

# Note that you can run multiple planners - each one will create and update its own window.
# See the minkowski_sum_tester as an example

print "Cells visited: " + str(planner.numberOfCellsVisited)
print "Total Travel Length: " + str(path.totalDistance())
print "Total Angle Turned: " + str(path.totalAngle())
