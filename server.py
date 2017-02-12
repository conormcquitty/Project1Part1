from adjacencygraph import AdjacencyGraph as Graph
import csv
import math
from minheap import MinHeap

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

    #finds the difference between the x & y coordinates to give us one x coordinate and one y coordinate
    a = abs((lat_lon[u])[0]) - abs((lat_lon[v])[0])
    b = abs((lat_lon[u])[1]) - abs((lat_lon[v])[1])

    #calculates the pythagoran distance between the 2 points
    i = int(math.sqrt( (a ** 2) + (b ** 2)))
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

    #Setup the minheap
    reached = list()
    runners = MinHeap()
    #Runner is organized in minheap based upon the time, but the key
    #pertains to its event. (time_arrive, v_to, v_next). First
    #runner has zero time_arrive and goes from itself to itself.
    runners.add(0,(0,start,start))
    while dest not in reached:
        #take the next node with the smallest distance to reach off the heap
        (key, curr_runner) = runners.pop_min()
        (w_cumulative, v_from, v_to) = curr_runner

        #if v_to has already been reached, restart and pop the next minimum
        if v_to in reached:
            continue

        #append v_to to reached because it has just been reached
        reached.append(v_to)

        for v_next in graph.neighbours(v_to):
            #compute the cost distance of going from the v_to (the currently
            #reached vertice) to all of its neighbours and add runners to the heap
            #that correspond to the neighbour vertices.
            #***Edit cost to be cost_distance of vertices for the implementation***
            # w = w_cumulative + cost_distance(v_to, v_next)
            w = w_cumulative + cost(v_to, v_next)
            runners.add(w,(w, v_to, v_next))

    #based upon the order that things are appended to the reached list, the
    #list will already be in the correct order of vertices
    return reached

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


if __name__ == "__main__":
    graph, lat_lon, street_name = read_city_graph("edmonton-roads-2.0.1.txt")
    cost = lambda u,v: cost_distance(u, v)
    # print(least_cost_path(graph, 29770958, 30198540, cost))

    line = input().strip().split()
    if line[0] == 'R':
        s_lat, s_lon = int(line[1]), int(line[2])
        d_lat, d_lon = int(line[3]), int(line[4])

    start_vertice = find_vertice(graph, s_lat, s_lon)
    end_vertice = find_vertice(graph, d_lat, d_lon)
    # print(start_vertice)
