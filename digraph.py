import copy
from collections import deque
from .algorithms import *

class DirectedGraph:
    """Class for a directed graph. 

    A graph is characterized by a set of nodes, a set of (directed) edges and a
    dictionary of attributes (eg. a name for the graph).
    Nodes cannot be None.
    """

    def __init__(self, graph = None):
        if graph is None:
            self._nodes = {}
            self._edges = {}
        else:
            self._nodes = copy.deepcopy(graph._nodes)
            self._edges = copy.deepcopy(graph._edges)
    
    def add_node(self, n):
        self._nodes[n] = {}
        self._edges[n] = {}
            
    def add_nodes(self, nodes):
        """Adds one or more nodes to the graph.
        """
        for n in nodes:
            self.add_node(n)
    
    def add_edges(self, edges):
        for e in edges:
            if not e[0] in self._nodes:
                self.add_node(e[0])
            if not e[1] in self._nodes:
                self.add_node(e[1])
            self._edges[e[0]][e[1]] = 1

    def add_weighted_edges(self, edges):
        for e in edges:
            if not e[0] in self._nodes:
                self.add_node(e[0])
            if not e[1] in self._nodes:
                self.add_node(e[1])
            self._edges[e[0]][e[1]] = e[2]        

    def edge_weight(self, edge):
        if edge[0] in self._edges and edge[1] in self._edges[edge[0]]:
            return self._edges[edge[0]][edge[1]]
        else:
            raise GraphValueError('edge does not exist')
    
    def add_cycle(self, nodes):
        if len(nodes) < 1:
            return
        
        self.add_nodes(nodes)
        edges = []
        for i in range(len(nodes) - 1):
            edges.append([nodes[i], nodes[i + 1]])
        edges.append([nodes[-1], nodes[0]])
        self.add_edges(edges)
        
    def add_path(self, nodes):
        pass

    def num_nodes(self):
        return len(self._nodes)

    def __len__(self):
        return len(self._nodes)

    def num_edges(self):
        nout = 0
        for n, neighbors in self._edges.iteritems():
            nout += len(neighbors)
        return nout

    def remove_nodes(self, nodes):
        pass

    def remove_edges(self, edges):
        pass

    def clear(self):
        self._nodes = {}
        self._edges = {}

    def nodes(self):
        """Returns iterator over nodes"""
        return iter(self._nodes.keys())

    def edges(self):
        """Returns iterator over edges"""
        for n, neighbors in self._edges.iteritems():
            for neigh, w in neighbors.iteritems():
                yield (n, neigh, w)
                
    def neighbors(self, node):
        """Returns iterator over the neighbors of the given node"""
        return iter([n[0] for n in self._edges[node].iteritems()])

    def weighted_neighbors(self, node):
        """Returns iterator over the neighbors of the given node"""
        return self._edges[node].iteritems()
    
    def in_degree(self, n):
        pass

    def out_degree(self, n):
        pass
    
    def to_undirected(self):
        pass
    
    def transpose(self):
        """Returns the transpose graph.
        
        The transpose graph of G is the graph with the same nodes as G,
        but with the direction of all the edges reversed.
        """
        
        g = DirectedGraph()
        for n in self.nodes():
            g.add_node(n)
        new_edges = [(e[1], e[0], e[2]) for e in self.edges()]
        g.add_weighted_edges(new_edges)
        return g
    
    def components(self):
        """Finds the strongly connected components of the graph.
        
        Return value:
        A list C of connected components. Each element of C corresponds to a 
        connected component and is a list of nodes in that component.
        """
        
        comps = []
        g_rev = self.transpose()
        
        node_queue = deque(self.nodes())
        visited = {}
        for n in self.nodes():
            visited[n] = False
        
        while len(node_queue) > 0:
            n = node_queue.pop()
            if not visited[n]:
                # If you haven't added the node to a component already
                c = dfs(self, n)
                c_rev = dfs(g_rev, n)
                print c, c_rev, n
                tmp_comp = []
                for adj in c:
                    if adj in c_rev:
                        visited[adj] = True
                        tmp_comp.append(adj)
                comps.append(tmp_comp)
        return comps
    
    def __str__(self):
        """Returns a human-readable represenation of the graph"""
        
        out_str = 'Graph with {:d} nodes and {:d} edges'.format(len(self), self.num_edges())
        return out_str

    def __repr__(self):
        """Returns a human-readable represenation of the graph"""
        
        out_str = 'Graph with {:d} nodes and {:d} edges'.format(len(self), self.num_edges())
        return repr(out_str)
