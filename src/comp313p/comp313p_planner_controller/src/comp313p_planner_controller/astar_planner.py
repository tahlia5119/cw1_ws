# -*- coding: utf-8 -*-

from cell_based_forward_search import CellBasedForwardSearch
from collections import deque
from cell import CellLabel
from math import sqrt
# This class implements the A* planning algorithm. 

class APlanner(CellBasedForwardSearch):

    # self implements an A* search algorithm
    
    def __init__(self, title, occupancyGrid,heuristic,constant,hscale=0):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self.aQueue = dict()
	self.Heuristic = heuristic
	self.scale = hscale
	self.const = constant

    # Put the cell's path cost and heuristic cost into the dictionary
    def pushCellOntoQueue(self, cell):
	if cell.parent:
	    cell.pathCost = cell.parent.pathCost+cell.distance(cell.parent)
	self.aQueue[cell] = cell.pathCost + self.heuristic(cell)

    #Selecting the heuristic based on user input

    def heuristic(self, cell):
	x = abs(cell.coords[0]-self.goal.coords[0])
	y = abs(cell.coords[1] - self.goal.coords[1])
		
	switch = {	
	    1: 0,#zero
	    2: self.const,#constant
	    3: x,#euclidean
	    4: max(x,y)+(sqrt(2)-1)*min(x,y),#octile
	    5: x+y, #manhattan
	    6: (x+y)+(sqrt(2)-2)*min(x,y)#diagonal
	}
	h = switch.get(self.Heuristic,0)
	return h*(1.0+self.scale)

    # Check the queue size is zero
    def isQueueEmpty(self):
        return not self.aQueue

    # Find the cell with the smallest path cost, delete it from the dictionary and return it
    def popCellFromQueue(self):
	cell = min(self.aQueue,key=self.aQueue.get)
        del self.aQueue[cell]
        return cell
    
    # Checking parent cells of cells that are labelled as ALIVE for any shorter paths to that particular cell
    def resolveDuplicate(self, cell, parent):
        if cell.label != CellLabel.DEAD:
	    distance = parent.pathCost+cell.distance(parent)
	    if distance < cell.pathCost:
		cell.pathCost = distance
		cell.parent = parent
		self.aQueue[cell] = cell.pathCost + self.heuristic(cell)
        pass
