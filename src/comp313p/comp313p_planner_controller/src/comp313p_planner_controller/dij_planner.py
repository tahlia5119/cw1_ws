# -*- coding: utf-8 -*-

from cell_based_forward_search import CellBasedForwardSearch
from collections import deque
from cell import CellLabel

# This class implements the Dijkstra planning algorithm. 

class DIJPlanner(CellBasedForwardSearch):

    # self implements a Dijkstra search algorithm
    
    def __init__(self, title, occupancyGrid):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self.dijQueue = dict()

    # Put the cell's path cost into the dictionary
    def pushCellOntoQueue(self, cell):
	if cell.parent:
	    cell.pathCost = cell.parent.pathCost+cell.distance(cell.parent)
	self.dijQueue[cell] = cell.pathCost

    # Check the queue size is zero
    def isQueueEmpty(self):
        return not self.dijQueue

    # Find the cell with the smallest path cost, delete it from the dictionary and return it
    def popCellFromQueue(self):
	cell = min(self.dijQueue,key=self.dijQueue.get)
        del self.dijQueue[cell]
        return cell

    # Checking parent cells of cells that are labelled as ALIVE for any shorter paths to that particular cell
    def resolveDuplicate(self, cell, parent):
        if cell.label != CellLabel.DEAD:
	    distance = parent.pathCost+cell.distance(parent)
	    if distance < cell.pathCost:
		cell.pathCost = distance
		cell.parent = parent
		self.dijQueue[cell] = distance
        pass
