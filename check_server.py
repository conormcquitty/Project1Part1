"""
server compliance verifier

When you run 
    python3 check_server.py
with just 
    server.py 
    adjacencygraph.py 
in the local directory you should get the following lines of output:
[1, 3, 6, 5]
[]
[3, 6]
[6, 5, 4, 3]
"""

import server
from adjacencygraph import AdjacencyGraph as Graph

graph = Graph()

verts = {1,2,3,4,5,6}

for v in verts:
    graph.add_vertex(v)

edges = [
    (1,2), (1,3), (1,6), (2,3), (2,4), 
    (3,2), (3,4), (3,6), (4,2), (4,3), (4,5), 
    (5,4), (5,6), (6,3), (6,5)]

for e in edges:
    graph.add_edge(e)

weights = {
    (1,2): 7, (1,3):9, (1,6):14, (2,1):7, (2,3):10, (2,4):15, 
    (3,1):9, (3,2):10, (3,4):11, (3,6):2, (4,2):15, (4,3):11, (4,5):6, 
    (5,4):6, (5,6):9, (6,1):14, (6,3):200, (6,5):9}

cost = lambda x, y: weights.get((x, y), float("inf"))

print(server.least_cost_path(graph, 1, 5, cost))
print(server.least_cost_path(graph, 5, 1, cost))
print(server.least_cost_path(graph, 3, 6, cost))
print(server.least_cost_path(graph, 6, 3, cost))
