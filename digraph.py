class Graph:
    '''Type to represent directed graphs.

    Attributes:
        vertices(set): identifiers representing the vertices in the graph.
        edges(list): pairs of vertices representing directed edges in the
        graph.
    '''

    def __init__(self):
        self._vertices = set()
        self._edges = list()

    def add_vertex(self, v):
        ''' Adds a vertex with identifier v to the graph.

        Args:
            v(hashable type): the vertex indifier to be added.
        '''
        self._vertices.add(v)

    def is_vertex(self, v):
        '''Checks whether v is a vertex of the graph.

        Args:
            v(hashable type): the vertex indifier to be added.

        Returns:
            bool: True if v is a vertex of the graph, False otherwise.
        '''
        return v in self._vertices

    def add_edge(self, e, autocreation=False):
        ''' Adds e to the edgle list of the graph.

        Args:
            e (tuple of two hashables): The edge to be added as a tuple. The
                edge goes from e[0] to e[1]
            autocreation (bool, default: False): Should the vertices of the
                edge be automatically added to the set of vertices if they are
                not there?

        Raises:
            RuntimeError: When one of the vertices is not in self._vertices
                and autocreation is off
        '''
        if autocreation:
            self.add_vertex(e[0])
            self.add_vertex(e[1])
        else:
            if not self.is_vertex(e[0]):
                raise RuntimeError("Attempt to create an edge with"
                                   "non-exsistant vertex: {}".format(e[0]))
            if not self.is_vertex(e[1]):
                raise RuntimeError("Attempt to create an edge with"
                                   "non-exsistant vertex: {}".format(e[1]))
        self._edges.append(e)

    def is_edge(self, e):
        ''' Checks whether an edge e exsists in self._edges

        Args:
            e (tuple of two hashables): The edge to be added as a tuple. The
                edge goes from e[0] to e[1]
        Returns:
            bool: True if e is an edge of the graph, False otherwise.
        '''
        return e in self._edges
