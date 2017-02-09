from adjacencygraph import AdjacencyGraph as Graph
import csv

def read_city_graph(city_graph):
    ''' Turns a comma seperated list into an AdjacencyGraph

    Args:
        city_graph: filename with extension .csv that exists
        in the same directory

    Returns:
        g: an AdjacencyGraph
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
    #Parse which lines include data about a vertice and
    #which lines include data about an edge.
    #Create two lists of vertices and edges.
    for i in range(len(rows)):
        if rows[i] == 'V':
            vertices.append(int(rows[i+1]))
        if rows[i] == 'E':
            edges.append((int(rows[i+1]), int(rows[i+2])))
        i+=4
    #Construct the UndirectedAdjacencyGraph from the
    #lists 'vertices' and 'edges'
    g = UndirectedAdjacencyGraph()
    for v in vertices:
        if not g.is_vertex(v):
            g.add_vertex(v)
    for e in edges:
        if not g.is_edge(e):
            g.add_edge(e)
    return g

if __name__ == "__main__":
    
