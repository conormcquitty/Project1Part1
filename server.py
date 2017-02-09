from adjacencygraph import AdjacencyGraph as Graph
import csv
import math

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
    lat_lon = dict()
    street_name = dict()
    #Parse which lines include data about a vertice and
    #which lines include data about an edge.
    #Create two lists of vertices and edges.
    #Creates the dictionary lat_lon, where the vertice number is the key and the latitude and longitude are attched to the key
    #Creates the dictionary street_name, where the edge start and end points tuple is the key, and the street name is attached to the key
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
    a = ((lat_lon[u])[0]) - ((lat_lon[v])[0])
    b = ((lat_lon[u])[1]) - ((lat_lon[v])[1])

    #calculates the pythagoran distance between the 2 points
    i = int(float(math.sqrt((a**2) + (b ** 2)))*100000)
    return i







if __name__ == "__main__":
    graph, lat_lon, street_name = read_city_graph("edmonton-roads-2.0.1.txt")
    a = cost_distance(29577354,29770958)
    print(a)
