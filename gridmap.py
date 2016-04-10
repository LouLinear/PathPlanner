import numpy as np
import math

class GridMapD:
    #general dense grid map object
    def __init__(self, dim=[10, 10]):
        if len(dim) > 3:
            raise ValueError('World space should stay within 1D to 3D')
        if len(dim) < 1:
            raise ValueError('World space should stay within 1D to 3D')
        
        dim = np.array(dim)
        
        if not(all(dim > 0)):
            raise ValueError('Size of each dimension should be greater than zero')
        self._dim = dim
        self._denseMap = np.ones(dim, dtype = np.bool_)

    def is_inside(self, pos):
        if len(pos) != len(self._dim):
            raise ValueError('Dimension mismatch between position and map')

        for i in range(0, len(self._dim)):
            if (pos[i] > (self._dim[i] - 1)) or (pos[i] < 0):
                return False
        
        return True

    def access(self, pos):
        if self.is_inside(pos):
            ele = self._denseMap[pos[0]]
            for p in pos[1:]:
                ele = ele[p]
            return ele
        else:
            return False

    def rand_obs_gen(self, thresh=0.5):
        """
        thresh is the probability that a given cell becomes occupied
        """
        tempMap = np.random.random(self._dim)
        self._denseMap = tempMap > thresh
        return

    def add_obs(self, pos):
        if self.is_inside(pos):
            nd = len(pos)
            if (nd == 1):
                self_denseMap[pos] = False
            elif (nd == 2):
                self._denseMap[pos[0], pos[1]] = False
            else:
                self._denseMap[pos[0], pos[1], pos[2]] = False
            return

    def rm_obs(self, pos):
        if self.is_inside(pos):
            nd = len(pos)
            if (nd == 1):
                self_denseMap[pos] = True
            elif (nd == 2):
                self._denseMap[pos[0], pos[1]] = True
            else:
                self._denseMap[pos[0], pos[1], pos[2]] = True
            return

    def size(self):
        return self._dim

    def _map(self):
        return self._denseMap
            
