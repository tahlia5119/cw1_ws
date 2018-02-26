#! /usr/bin/env python

# Import the needed types.
from comp313p_planner_controller.occupancy_grid import OccupancyGrid
from comp313p_planner_controller.dij_planner import DIJPlanner
import numpy as np

# Create the occupancy grid. Syntax is: number of cells in X, number of cells in Y,
# length of each cell in m
occupancyGrid = OccupancyGrid(21, 21, 0.5)

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
planner = DIJPlanner('Dijkstra\'s Search Algorithm', occupancyGrid);

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
