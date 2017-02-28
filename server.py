from adjacencygraph import AdjacencyGraph as Graph
import csv
import math
import textserial
import sys
import argparse
from cs_message import *

class MinHeap:

    def __init__(self):
        self._array = []

    def add(self, key, value):
        self._array.append((key, value))
        self.fix_heap_up(len(self._array)-1)

    def pop_min(self):
        if not self._array:
            raise RuntimeError("Attempt to call pop_min on empty heap")
        retval = self._array[0]
        self._array[0] = self._array[-1]
        del self._array[-1]
        if self._array:
            self.fix_heap_down(0)
        return retval

    def fix_heap_up(self, i):
        if self.isroot(i):
            return
        p = self.parent(i)
        if self._array[i][0] < self._array[p][0]:
            self.swap(i, p)
            self.fix_heap_up(p)

    def swap(self, i, j):
        self._array[i], self._array[j] = \
            self._array[j], self._array[i]

    def isroot(self, i):
        return i == 0

    def isleaf(self, i):
        return self.lchild(i) >= len(self._array)

    def lchild(self, i):
        return 2*i+1

    def rchild(self, i):
        return 2*i+2

    def parent(self, i):
        return (i-1)//2

    def min_child_index(self, i):
        l = self.lchild(i)
        r = self.rchild(i)
        retval = l
        if r < len(self._array) and self._array[r][0] < self._array[l][0]:
            retval = r
        return retval

    def isempty(self):
        return len(self._array) == 0

    def length(self):
        return len(self._array)

    def fix_heap_down(self, i):
        if self.isleaf(i):
            return

        j = self.min_child_index(i)
        if self._array[i][0] > self._array[j][0]:
            self.swap(i, j)
            self.fix_heap_down(j)

lat_lon = dict()

def read_city_graph(city_graph):
    ''' Turns a comma seperated list into an AdjacencyGraph

    Args:
        city_graph: filename with extension .csv that exists
        in the same directory

    Returns:
        g: an AdjacencyGraph
        lat_lon: the latitude and longitude dictionary associated with the vertice key
        street_name: the street name dictionary associated with the edge key
    '''

    #Read file into a list of tokens from the file.
    with open(city_graph) as f:  # f = open('some.csv')
        reader = csv.reader(f)
        rows = []
        for row in reader:  # reads a line, or "row"
            rows += row  # list of strings
    f.close()

    vertices = []
    edges = []
    street_name = dict()
    #Parse which lines include data about a vertice and
    #which lines include data about an edge.
    #Create two lists of vertices and edges.
    #Creates the dictionary lat_lon, where the vertice number is the key and
    #the latitude and longitude are attched to the key
    #Creates the dictionary street_name, where the edge start and end
    # points tuple is the key, and the street name is attached to the key
    for i in range(len(rows)):
        if rows[i] == 'V':
            vertices.append(int(rows[i+1]))
            lat_lon[int(rows[i+1])] = ((int(float(rows[i+2])*100000)),(int(float(rows[i+3])*100000)))
        if rows[i] == 'E':
            edges.append((int(rows[i+1]), int(rows[i+2])))
            street_name[(int(rows[i+1]), int(rows[i+2]))] = (rows[i+3])
        i+=4
    #Construct the AdjacencyGraph from the
    #lists 'vertices' and 'edges'
    g = Graph()
    for v in vertices:
        if not g.is_vertex(v):
            g.add_vertex(v)
    for e in edges:
        if not g.is_edge(e):
            g.add_edge(e)
    return g, lat_lon, street_name


def cost_distance(u, v):
    '''Computes and returns the straight-line distance between the two vertices u and v.

    Args:
        u, v: The ids for two vertices that are the start and
        end of a valid edge in the graph.

    Returns:
        numeric value: the distance between the two vertices.
    '''
    #@TODO: Apparently the abs() function makes the cost function calculation
    #wrong. Redo this algorithm (should be a small change)

    a = (lat_lon[u])[0] - (lat_lon[v])[0]
    b = (lat_lon[u])[1] - (lat_lon[v])[1]

    #calculates the pythagorean distance between the 2 points
    i = math.sqrt( (a ** 2) + (b ** 2))
    return i

def least_cost_path (graph, start, dest, cost):
    """Find and return a least cost path in graph from start vertex to dest vertex.
    Efficiency: If E is the number of edges, the run-time is
      O( E log(E) ).
    Args:
      graph (Graph): The digraph defining the edges between the
        vertices.
      start: The vertex where the path starts. It is assumed
        that start is a vertex of graph.
      dest:  The vertex where the path ends. It is assumed
        that start is a vertex of graph.
      cost:  A function, taking the two vertices of an edge as
        parameters and returning the cost of the edge. For its
        interface, see the definition of cost_distance.
    Returns:
      list: A potentially empty list (if no path can be found) of
        the vertices in the graph. If there was a path, the first
        vertex is always start, the last is always dest in the list.
        Any two consecutive vertices correspond to some
        edge in graph.
    """

    #Set to keep track of all the reached vertices
    reached = set()
    #Dictionary to keep frack of how you got to the current vertice
    prev_v = dict()
    runners = MinHeap()
    #Runner is organized in minheap based upon the cost_distance, but the value
    #pertains to its event. (cost_distance, v_from, v_to)
    #First runner has zero time_arrive and goes from itself to itself.
    runners.add(0,(0,start,start))
    while not runners.isempty():
        #if we've found the destination, break
        if dest in reached:
            break
        #take the next node with the smallest distance to reach off the heap
        (key, curr_runner) = runners.pop_min()
        (w_cumulative, v_from, v_to) = curr_runner
        # print("v_from, v_to, w_cum:", v_from, v_to, w_cumulative)
        #if v_to has already been reached, restart and pop the next minimum
        if v_to in reached:
            continue

        #append v_to to reached because it has just been reached
        reached.add(v_to)
        #append to the dictionary how you just got to this node
        prev_v[v_to] = v_from

        for next_vertices in graph.neighbours(v_to):
            #compute the cost distance of going from the v_to (the currently
            #reached vertice) to all of its neighbours and add runners to the heap
            #that correspond to the neighbour vertices.
            #***Edit cost to be cost_distance of vertices for the implementation***
            # w = w_cumulative + cost_distance(v_to, v_next)
            w = w_cumulative + cost(v_to, next_vertices)
            runners.add(w, (w, v_to, next_vertices))
    #initiate a path list that we can load the ordered path into
    path = list()
    vertice = dest
    #if the destination wasn't found, return the empty path list
    if dest not in reached:
        return path
    #If it is found, start to add elements into path. Because we have to walk
    #backwards through the list based on the key organization of the prev_v dict,
    #elements are added in reversed order (ie. goes from dest->start)
    else:
        path.append(dest)
        while start not in path:
            path.append(prev_v[vertice])
            vertice = prev_v[vertice]
    #return the reversed path so it goes (start->dest)
    return list(reversed(path))

def find_vertice(graph, lat, lon):
    """Finds and returns the cloesest vertices to the given latitude and longitude
    Args:
        lat: latitude expressed as an integer in 100-000ths
         of a degree
        lon: longitude expressed as an integer in 100-000ths
         of a degree
    Returns:
        vertice: A vertice in the graph of the city that is
        closest to the requested latitude and longitude in
        terms of Euclidean distances
    """
    #@TODO: Inefficient algorithm, must revisit

    #Add a vertice to lat_lon so cost_distance can be reused to calculate
    #Euclidean distance give to vertices. 0 is an arbitrary value
    lat_lon[0]= (lat, lon)
    heap = MinHeap()
    #Add to the minheap sorting on the cost_distance between vertices.
    #The vertice is the value associated with the cost_distance in the
    #minheap
    for v in lat_lon.keys():
        if v != 0:
            heap.add(cost_distance(0, v), v)
    #pop the minimum Euclidean distance and return only the vertice
    #associated with the value
    return heap.pop_min()[1]


def protocol(serial_in, serial_out):
    """
    The server protocol that runs idefinitely.
    """

    #@TODO: no testing done. Testing must be done on this function to make sure
    #that it is properly communicating.

    graph, lat_lon, street_name = read_city_graph("edmonton-roads-2.0.1.txt")

    #define cost for server
    cost = lambda u,v: cost_distance(u, v)

    #Take input from Arduino through the serialPort
    #and using cs_message
    while True:
        while True:
            msg = receive_msg_from_client(serial_in)
            if msg[0] == "R":
                break
        #Break the coordinates into their own list

        coords = msg[2:].split()
        #Check for proper format
        if len(coords) != 4:
            continue
        #Convert to integers
        # log_msg("{}".format(coords))
        s_lat, s_lon = int(coords[0]), int(coords[1])
        d_lat, d_lon = int(coords[2]), int(coords[3])
        # log_msg(" {} {} {} {}". format(s_lat, s_lon, d_lat, d_lon))
        #find the closest start and destination vertices
        start = find_vertice(graph, s_lat, s_lon)
        end = find_vertice(graph, d_lat, d_lon)
        #find the shortest path
        path = least_cost_path(graph, start, end, cost)
        # print("PATH: ", path)
        #start route
        count = len(path)
        #start instruction counter
        instr_num = 0
        send_msg_to_client(serial_out, "N {}" .format(count))
        while instr_num != count:
            #while arduino has not responded to waypoint with proper query

            while msg[0] != "A":
                msg = receive_msg_from_client(serial_in)

            #print the waypoint
            log_msg("Recieved 'A'")
            send_msg_to_client(serial_out, "W {} {}"\
            .format(lat_lon[path[instr_num]][0], lat_lon[path[instr_num]][1]))
            #increment the counter
            instr_num += 1
        #@BUG: This algorithm design makes it print 'E' even when there is
        #no path to be found
        if receive_msg_from_client(serial_in) == 'A':
            send_msg_to_client(serial_out, "E")



def main():

    serial_port_name = '/dev/ttyACM0'
    log_msg("Opening serial port: {}".format(serial_port_name))

    # Open up the connection

    baudrate = 9600  # [bit/seconds] 115200 also works

    # Run the server protocol forever

    # The with statment ensures that if things go bad, then ser
    # will still be closed properly.
    # errors='ignore' allows any 1 byte character, not just the usual
    # ascii range [0,127]

    with textserial.TextSerial(
        serial_port_name, baudrate, errors='ignore', newline=None) as ser:
        protocol(ser, ser)


if __name__ == "__main__":
    main()
