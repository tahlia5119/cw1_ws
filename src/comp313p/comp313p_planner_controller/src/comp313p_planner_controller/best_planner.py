# -*- coding: utf-8 -*-

from cell_based_forward_search import CellBasedForwardSearch
from collections import deque

# This class implements the Best first planning algorithm. 

class BestPlanner(CellBasedForwardSearch):

    # self implements a simple Best first search algorithm
    
    def __init__(self, title, occupancyGrid):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self.bestQueue = dict()

    # Put the cell's distance into the dictionary
    def pushCellOntoQueue(self, cell):
	score = cell.distance(self.goal)
        self.bestQueue[cell] = score

    # Check the queue size is zero
    def isQueueEmpty(self):
        return not self.bestQueue

    # Find the cell with the smallest distance
    def popCellFromQueue(self):
	cell = min(self.bestQueue,key=self.bestQueue.get)
        del self.bestQueue[cell]
        return cell

    def resolveDuplicate(self, cell, parentCell):
        # Nothing to do in self case
        pass
