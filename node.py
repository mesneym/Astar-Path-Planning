class Node:
    def __init__(self,state = None,cost = float('inf'),costToCome= float('inf'),parent= None):
        self.state = state
        self.parent = parent
        self.cost = cost
        self.costToCome = costToCome


