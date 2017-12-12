# -*- coding: utf-8 -*-
"""
Created on Sun Mar 19 12:25:07 2017

@author: sji367
"""
from scipy.io import loadmat
from heapq import heappush, heappop # for priority queue
import time
import random
import numpy as np

class node:
    """ Class for storing the information on the explored nodes. """
    def __init__(self, xPos=0, yPos=0, distance=0, priority=0):
        # Current Position
        self.xPos = xPos
        self.yPos = yPos
        # total distance already travelled to reach the node
        self.distance = distance
        # priority = distance + remaining distance estimate
        self.priority = priority
        
    def __lt__(self, other): 
        """ Function for priority queue. Smaller priority = Higher priority """
        return self.priority < other.priority
        
    def updatePriority(self, xDest, yDest):
        """ Priority = Cost to this point + remaining distance estimate """
        self.priority = self.distance + self.estimate(xDest, yDest)
        
    def calc_cost(self, dx, dy):
        """ Cost function to move to the next point"""
        self.distance+=np.sqrt(dx**2+dy**2)
            
    def estimate(self, xDest, yDest):
        """ Estimation function for the remaining distance to the goal. """
        dx = xDest - self.xPos
        dy = yDest - self.yPos
        # Euclidian Distance
        d= np.sqrt(dx**2 + dy**2)
        
        # Octile Distance
        #d = max(dx,dy)+.4*min(dx,dy)
        
        return(d)

def numRowsCols(array):
    """ Returns the number of rows and columns of a 2D matrix"""
    return len(array),len(array[0])

class A_Star(object):
    """ This Class runs the A* algorithm """
    def __init__(self, MAP, xStart, yStart, xFinish, yFinish, Connecting_Distance=1):
        self.xStart = xStart
        self.yStart = yStart
        self.xFinish = xFinish
        self.yFinish = yFinish
        self.MAP = MAP
        self.n, self.m = numRowsCols(MAP)
        self.dx, self.dy, self.num_directions = self.Neighbors_Mask(Connecting_Distance)
    
    def Neighbors_Mask(self,Connecting_Distance):
        """ Number of Neighboors one wants to investigate from each cell. A larger
            number of nodes means that the path can be alligned in more directions. 
            
        Inputs:
            Connecting_Distance=1-> Path can  be alligned along 8 different direction.
            Connecting_Distance=2-> Path can be alligned along 16 different direction.
            Connecting_Distance=3-> Path can be alligned along 32 different direction.
            Connecting_Distance=4-> Path can be alligned along 56 different direction.
            ETC......
            
        Ouputs:
            row - x vector of the mask containing all neighbors
            col - y vector of the mask containing all neighbors
            num_directions - number of directions that A* is searching
        """
        NeighborCheck=np.ones((2*Connecting_Distance+1, 2*Connecting_Distance+1))
        Dummy=2*Connecting_Distance
        Mid=Connecting_Distance
        for i in range(Connecting_Distance-1):
            NeighborCheck[i,i]=0
            NeighborCheck[Dummy-i,i]=0
            NeighborCheck[i,Dummy-i]=0
            NeighborCheck[Dummy-i,Dummy-i]=0
            NeighborCheck[Mid,i]=0
            NeighborCheck[Mid,Dummy-i]=0
            NeighborCheck[i,Mid]=0
            NeighborCheck[Dummy-i,Mid]=0
        NeighborCheck[Mid,Mid]=0
        
        # Find the locations of the new neighbor
        dy,dx = np.where(NeighborCheck==1)
        difference = np.ones_like(dx)*Connecting_Distance
        dy -= difference
        dx -= difference
        
        num_directions = len(dx)
            
        return np.ndarray.tolist(dx), np.ndarray.tolist(dy), num_directions
        
    def Reconstruct_Path(self, dir_map):
        """ Generate the path from finish to start by following the directions
            in the direction map (dir_map).
            
        Input:
            dir_map - Map of how to traverse back to the start. Contains 
                the index of dx/dy.
        
        Output:
            path - Comma seperated string of the planned path using the 
                    indices of dx/dy
        """
        path = ''
        first_time = True
        x = self.xFinish
        y = self.yFinish
        while not (x == self.xStart and y == self.yStart):
            j = dir_map[y][x]
            c = str((self.num_directions-j-1) % self.num_directions)
            if first_time:
                path=c+path
                first_time=False
            else:
                path = c + ','+ path
            x += self.dx[j]
            y += self.dy[j]
        return path
        
    def Find_Path(self):
        """ This function runs A*. It outputs the generated path as an
            string where each movement is stored as a number corrisponding to 
            the index of the neighbor mask."""
        closed_nodes_map = [] # map of closed (tried-out) nodes
        open_nodes_map = [] # map of open (not-yet-tried) nodes
        dir_map = [] # map of directions
        row = [0] * self.n
        for i in range(self.m): # create 2d arrays
            closed_nodes_map.append(list(row))
            open_nodes_map.append(list(row))
            dir_map.append(list(row))
        
        pq = [[], []] # priority queues of open (not-yet-tried) nodes
        pqi = 0 # priority queue index
        # create the start node and push into list of open nodes
        n0 = node(self.xStart, self.yStart, 0.0, 0.0)
        n0.updatePriority(self.xFinish, self.yFinish)
        heappush(pq[pqi], n0)
        open_nodes_map[self.yStart][self.xStart] = n0.priority # mark it on the open nodes map
        
        # A* search
        while len(pq[pqi]) > 0:
            # get the current node w/ the highest priority
            # from the list of open nodes
            n1 = pq[pqi][0] # top node
            n0 = node(n1.xPos, n1.yPos, n1.distance, n1.priority)
            x = n0.xPos
            y = n0.yPos
            heappop(pq[pqi]) # remove the node from the open list
            open_nodes_map[y][x] = 0
            # mark it on the closed nodes map
            closed_nodes_map[y][x] = 1
        
            # quit searching when the goal state is reached
            if x == self.xFinish and y == self.yFinish:
                # Generate the path from finish to start by following the 
                #  directions.
                return self.Reconstruct_Path(dir_map)
        
            # generate moves (child nodes) in all possible directions
            for i in range(self.num_directions):
                new_x = x + self.dx[i]
                new_y = y + self.dy[i]
                Flag=True
                if not (new_x < 0 or new_x > self.n-1 or new_y < 0 or new_y > self.m - 1
                        or self.MAP[new_y][new_x] == 1 or closed_nodes_map[new_y][new_x] == 1):
                    # Check to see if the extended path runs through any obstacles
                    if (abs(self.dx[i])>1 or abs(self.dy[i])>1):
                        # Need to check that the path does not pass an object
                        JumpCells=2*max(abs(self.dx[i]),abs(self.dy[i]))-1
                        for K in range(1,JumpCells):
                            YPOS=int(round(K*1.0*self.dy[i]/JumpCells))
                            XPOS=int(round(K*1.0*self.dx[i]/JumpCells))
                            if (self.MAP[y+YPOS][x+XPOS]==1):
                                Flag=False
                    if Flag:                
                        # generate a child node
                        m0 = node(new_x, new_y, n0.distance, n0.priority)
                        m0.calc_cost(self.dx[i], self.dy[i])
                        m0.updatePriority(self.xFinish, self.yFinish)
                        # if it is not in the open list then add into that
                        if open_nodes_map[new_y][new_x] == 0:
                            open_nodes_map[new_y][new_x] = m0.priority
                            heappush(pq[pqi], m0)
                            # mark its parent node direction
                            dir_map[new_y][new_x] = (self.num_directions-i-1) % self.num_directions
                        elif open_nodes_map[new_y][new_x] > m0.priority:
                            # update the priority info
                            open_nodes_map[new_y][new_x] = m0.priority
                            # update the parent direction info
                            dir_map[new_y][new_x] = (self.num_directions-i-1) % self.num_directions
                            # replace the node
                            # by emptying one pq to the other one
                            # except the node to be replaced will be ignored
                            # and the new node will be pushed in instead
                            while not (pq[pqi][0].xPos == new_x and pq[pqi][0].yPos == new_y):
                                heappush(pq[1 - pqi], pq[pqi][0])
                                heappop(pq[pqi])
                            heappop(pq[pqi]) # remove the wanted node
                            # empty the larger size pq to the smaller one
                            if len(pq[pqi]) > len(pq[1 - pqi]):
                                pqi = 1 - pqi
                            while len(pq[pqi]) > 0:
                                heappush(pq[1-pqi], pq[pqi][0])
                                heappop(pq[pqi])       
                            pqi = 1 - pqi
                            heappush(pq[pqi], m0) # add the better node instead
        return '','' # no route found
        
    def mark_route(self, route):
        """ This function marks the route on the map """
        WPT = []
        if len(route) > 0:
            x = self.xStart
            y = self.yStart
            self.MAP[y][x] = 2
            WPT.append([x,y])
            for i in range(len(route)):
                j = int(route[i])
                x += self.dx[j]
                y += self.dy[j]
                if i<len(route)-1:
                    if route[i]==route[i+1]:
                        self.MAP[y][x] = 3
                    else:
                        self.MAP[y][x] = 4
                        WPT.append([x,y])
            self.MAP[y][x] = 5
            WPT.append([x,y])
            
        return WPT
       
    def print_map(self, route, total_time, print_meta=True):
        """ This function prints out the map to std output where:
                '.' = space
                '0' = Obstacle
                'S' = Start
                'F' = Finish
                'X' = New Waypoint
                '@' = route (no new waypoint)
        """
        waypoints = self.mark_route(route)
        if print_meta:
            n,m = numRowsCols(self.MAP)
            print 'Map Size (X,Y): ', n, m
            print 'Start: ', self.xStart, self.yStart
            print 'Finish: ', self.xFinish, self.yFinish
            print 'Time to generate the route: {} seconds'.format(total_time)
            print 'Route: \n{}'.format(route)
            print waypoints
        
        # display the map with the route
        print 'Map:'
        for y in range(m):
            for x in range(n):
                xy = self.MAP[y][x]
                if xy == 0:
                    print '.', # space
                elif xy == 1:
                    print 'O', # obstacle
                elif xy == 2:
                    print 'S', # start
                elif xy == 3:
                    print '@', # route
                elif xy == 4:
                    print 'X', # Waypoint
                elif xy == 5:
                    print 'F', # finish
            print
            
def build_map(n=30,m=30, preset=True, filename='/home/sji367/small_grid.mat', key='new_grid'):
    """ Function to build the map
    
    Inputs:
        n - Range of y
        m - Range of x
        preset - Boolean to use the preset map or not
        filename - file name for the .mat grid
        key - Name of the key for the grid dictionary
        
    Outputs:
        the_map - Boolean obstacle map to be used by A*
        xStart, yStart - Start coordinates 
        xFinish, yFinish - Finish coordinates
        
        """
    if preset:
        the_map = []
        row = [0] * n
        for i in range(m):
            the_map.append(list(row))
        
        # fillout the map matrix with a '+' pattern
        for x in range(n / 8, n * 7 / 8):
            the_map[m / 2][x] = 1
        for y in range(m/8, m * 7 / 8):
            the_map[y][n / 2] = 1
        
        # randomly select start and finish locations from a list
        sf = []
        sf.append((0, 0, n - 1, m - 1))
        sf.append((0, m - 1, n - 1, 0))
        sf.append((n / 2 - 1, m / 2 - 1, n / 2 + 1, m / 2 + 1))
        sf.append((n / 2 - 1, m / 2 + 1, n / 2 + 1, m / 2 - 1))
        sf.append((n / 2 - 1, 0, n / 2 + 1, m - 1))
        sf.append((n / 2 + 1, m - 1, n / 2 - 1, 0))
        sf.append((0, m / 2 - 1, n - 1, m / 2 + 1))
        sf.append((n - 1, m / 2 + 1, 0, m / 2 - 1))
        (xStart, yStart, xFinish, yFinish) = random.choice(sf)
    else:
        grid = loadmat(filename)
        the_map = grid[key]
        xStart = 19
        yStart = 31
        xFinish = 67
        yFinish = 98
        
    return the_map, xStart, yStart, xFinish, yFinish

MAP, xStart, yStart, xFinish, yFinish = build_map(preset=False)
astar = A_Star(MAP, xStart, yStart, xFinish, yFinish, 5)
t0 = time.time()
route = astar.Find_Path()
t1 = time.time() - t0
astar.print_map(route.split(','),  t1)