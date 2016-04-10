import numpy as np
import gridmap as MAP
import Queue as Q

class GridNode:
    """
    A representation of a node on the map so I can utilize priority queue
    """
    def __init__(self, pos, cost=np.float('inf'), heuristic=0, bestPrevNode=None):
        self.__pos = pos
        self.__cost = cost
        self.__heuristic = heuristic
        self.__bestPrevNode = bestPrevNode
        
    def __cmp__(self, other):
        return (self.__cost + self.__heuristic) > (other.__cost + other.__heuristic)

    def set_cost(self, newcost):
        self.__cost = newcost
        return

    def set_bestPrev(self, newNode):
        self.__bestPrevNode = newNode
        return

    def pos(self):
        return self.__pos

    def cost(self):
        return self.__cost

    def bestPrevNode(self):
        return self.__bestPrevNode

class PlannerBase:

    def __init__(self, gridmap):
        self._gridmap = gridmap
        self._mapsize = gridmap.size()
        #multiplier for sub2ind-like hash generator
        self._hashMult = np.cumprod(self._mapsize[::-1])

    def __gen_hashkey__(self, pos):
        #basically a sub2ind to get a hashable key for the built-in python dictionary
        #no error raised for dimension mismatch (private function)
        h = sum(np.multiply(self._hashMult[0:-1], pos[0:-1]))
        return (h + pos[-1])

class PlannerAccel(PlannerBase):
    """
    An accelerator class that holds all the queues and hash tables for fast planning
    """
    def __init__(self, hashMult, goalkey):
        self._to_exploreQ = Q.PriorityQueue()
        self._to_exploreH = {}
        self._explored = {}
        self._hashMult = hashMult
        self._goalkey = goalkey

    def put(self, Node):
        """
        puts GridNode in the to_explore priority queue and hash table
        """
        self._to_exploreQ.put(Node)
        hashkey = self.__gen_hashkey__(Node.pos())
        self._to_exploreH[hashkey] = Node
        return

    def pop(self):
        to_pop = self._to_exploreQ.get()
        hashkey = self.__gen_hashkey__(to_pop.pos())
        del self._to_exploreH[hashkey]
        self._explored[hashkey] = 1
        return to_pop

    def access(self, key_n):
        return self._to_exploreH[key_n]

    def update(self):
        #a hack to force the built in queue to update after i modified values of an element
        temp = self._to_exploreQ.get()
        self._to_exploreQ.put(temp)

    def is_empty(self):
        return self._to_exploreQ.empty()

    def is_destination(self, Node):
        currentKey = self.__gen_hashkey__(Node.pos())
        if currentKey == self._goalkey:
            return True
        else:
            return False

    def has_queued(self, key_n):
        return self._to_exploreH.has_key(key_n)

    def has_explored(self, key_n):
        return self._explored.has_key(key_n)
        
class PlannerHolo(PlannerBase):
    """
    A* planner for holonomic robot in world space
    """

    def plan(self, start, goal):

        if not(self._gridmap.is_inside(start)) or not(self._gridmap.is_inside(goal)):
             print 'Start or goal position out of bound. No path can be found'
             return None

        if not(self._gridmap.access(start)) or not(self._gridmap.access(goal)):
             print 'Start or goal position are occupied by obstacles, No path can be found'
             return None
        
        #construct the start node
        startHuer = np.linalg.norm(goal - start)
        startNode = GridNode(start, np.float(0), startHuer)
        goalKey = self.__gen_hashkey__(goal)

        #initialize accelerator class
        acc = PlannerAccel(self._hashMult, goalKey)

        #start by putting the start node in the accelerator
        acc.put(startNode)

        #find out the number of dimensions of map
        nd = len(self._mapsize)

        while not(acc.is_empty()):
            
            currentNode = acc.pop()
            
            if acc.is_destination(currentNode):
                pathstack = self._backtrack(currentNode)
                return pathstack

            self._expand(currentNode, goal, acc, [], nd)

        print 'Cannot find a path'
        return None

    def printpath(self, pathstack):
        if pathstack is None: return
        for p in pathstack:
            print p
        return

    def _expand(self, Node, goal, acc, vec, nd):
        #recursive nested for loops to expand adaptively to map dimension
        #this assumes that I can move to any immediately connected neighbor

        if nd > 0:
            for i in range(-1, 2):
                veccopy = list(vec)
                veccopy.append(i)
                self._expand(Node, goal, acc, veccopy, nd-1)
        else:
            #move from the node to expand
            pos_n = Node.pos() + np.array(vec)
            #check if this node is reachable
            if (self._gridmap.access(pos_n)):
                #if so, check if this node has been previously fully explored
                key_n = self.__gen_hashkey__(pos_n)
                if not(acc.has_explored(key_n)):
                    cost_proposed = Node.cost() + np.linalg.norm(vec)
                    if acc.has_queued(key_n):
                        node2mod = acc.access(key_n)
                        if cost_proposed < node2mod.cost():
                            node2mod.set_cost(cost_proposed)
                            node2mod.set_bestPrev(Node)
                            acc.update()
                    else:
                        newHeuristic = np.linalg.norm(goal - pos_n)
                        newNode = GridNode(pos_n, cost_proposed, newHeuristic, Node)
                        acc.put(newNode)

    def _backtrack(self, Node):
        #prints planned path
        n = Node
        path = [n.pos()]
        while not(n.bestPrevNode() is None):
            path.append(n.bestPrevNode().pos())
            n = n.bestPrevNode()
        path.reverse()
        return np.vstack(path)
