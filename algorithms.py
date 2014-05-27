from .graph import *
from .digraph import *

from collections import deque
import heapq

def dfs(g, start_node = None):
    """Runs Depth First Search on a graph.
    
    Args:
    - g: A directed or undirected graph.
    - start_node: A node of g from where the search will start.
    If this is None, the search will start from a random node of g.
    
    Return value:
    A list of nodes in the order visited.
    """
    
    if not start_node is None and not start_node in g.nodes():
        raise GraphValueError('start node does not exist')
        
    visited = {}
    for n in g.nodes():
        visited[n] = False
        
    if start_node is None:
        start_node = n
        
    q = deque([start_node])
    output = []
    
    while len(q) > 0:
        # Pop from the right (top of queue)
        n = q.pop()
        if not visited[n]:
            output.append(n)
            visited[n] = True
            for adj in g.neighbors(n):
                if not visited[adj]:
                    # Append to the right (top of queue)
                    q.append(adj)
    return output

def bfs(g, start_node = None):
    """Runs Breath First Search on a graph.
    
    Args:
    - g: A directed or undirected graph.
    - start_node: A node of g from where the search will start.
    If this is None, the search will start from a random node of g.
    
    Return value:
    A list of nodes in the order visited.
    """
    if not start_node is None and not start_node in g.nodes():
        raise GraphValueError('start node does not exist')
        
    visited = {}
    for n in g.nodes():
        visited[n] = False
    
    if start_node is None:
        start_node = n
        
    q = deque([start_node])
    output = []
    
    while len(q) > 0:
        n = q.popleft()
        if not visited[n]:
            output.append(n)
            visited[n] = True
            for adj in g.neighbors(n):
                if not visited[adj]:
                    q.append(adj)
    return output

def dijkstra(g, start_node, end_node = None):
    """Finds shortest paths from a given node using Dijkstra's algorithm.
    
    Args:
    - g: graph object. It should not contain negative edges (at least
    not in the part reachable from start_node).
    - start_node: node of g from which shortest paths will be computed
    - end_node: if this is None, then shortest paths to all nodes reachable
    from start_node will be computed. Otherwise, only the shortest path to
    end_node will be computed.
    
    Return value:
    A tuple (dist, previous) where:
    - dist is a dictionary if end_node is None, such that dist[n] is the 
    distance of the shortest path to n. If end_node is not None, then
    dist is just the distance from start_node to end_node.
    - previous is a dictionary, such that previous[n] is the previous node
    in the path from start_node to n.
    """
    
    if not start_node in g.nodes():
        raise GraphValueError('start node does not exist')
    if not end_node is None and not end_node in g.nodes():
        raise GraphValueError('end node does not exist')
    
    # Dictionary of the distance of each node from start
    dist = {}
    previous = {}
    for n in g.nodes():
        previous[n] = None
        dist[n] = float('inf')
        
    # Also put the distances in a heap, so you can easily get the 
    # node with minimum distance. The items in the heap have the form
    # (dist, node_index).
    dist_heap = []
    heapq.heappush(dist_heap, (0, start_node))
    
    while len(dist_heap) > 0:
        node_dist = heapq.heappop(dist_heap)        
        n = node_dist[1]
        # Normally, we would not extract from the heap the same node
        # twice. However, in this implementation, we don't decrease the
        # nodes keys in the heap, rather we add copies with smaller keys.
        # This is not very efficient but we avoid having to track where
        # each node is stored in the heap.
        # TODO: Re-implement heap to make this easier. 
        if node_dist[0] >= dist[n]:
            continue
        dist[n] = node_dist[0]
        if n == end_node:
            break
        for adj in g.weighted_neighbors(n):
            adj_node = adj[0]
            edge_weight = adj[1]
            if edge_weight < 0:
                raise UnsupportedGraphType('Dijkstra\'s algorithm does not support negative edges') 
            new_dist = dist[n] + edge_weight
            if dist[adj_node] > new_dist:
                previous[adj_node] = n
                heapq.heappush(dist_heap, (new_dist, adj_node))
                
    if end_node is None:
        return (dist, previous)
    else:
        return (dist[end_node], previous)

def bellman_ford(g, start_node):
    """Finds shortest paths from a given node using the Bellman-Ford algorithm.
    
    Args:
    - g: graph object. It should not contain negative cycles (at least
    not in the part reachable from start_node).
    - start_node: node of g from which shortest paths will be computed
    
    Return value:
    A tuple (dist, previous) where:
    - dist is a dictionary, such that dist[n] is the distance of the shortest 
    path from start_node to n.
    - previous is a dictionary, such that previous[n] is the previous node
    in the path from start_node to n.
    """
    if not start_node in g.nodes():
        raise GraphValueError('start node does not exist')
        
    dist = {}
    previous = {}
    for n in g.nodes():
        previous[n] = None
        dist[n] = float('inf')
    dist[start_node] = 0
    
    for i in range(len(g) - 1):
        change = False
        for edge in g.edges():
            start = edge[0]
            end = edge[1]
            weight = edge[2]
            new_dist = dist[start] + weight
            if dist[end] > new_dist:
                change = True
                dist[end] = new_dist
                previous[end] = start
        if not change:
            break
            
    for edge in g.edges():
        start = edge[0]
        end = edge[1]
        weight = edge[2]
        new_dist = dist[start] + weight
        if dist[end] > new_dist:
            raise UnsupportedGraphType('Negative cycle detected. Shortest paths are ill-defined.')
    
    return (dist, previous)


def shortest_path(g, start_node, end_node = None):
    """Finds shortest paths from a given node.
    
    Args:
    - g: graph object. It should not contain negative cycles (at least
    not in the part reachable from start_node).
    - start_node: node of g from which shortest paths will be computed
    - end_node: if this is None, then shortest paths to all nodes reachable
    from start_node will be computed. Otherwise, only the shortest path to
    end_node will be computed.
    
    Return value:
    A tuple (dist, previous) where:
    - dist is a dictionary if end_node is None, such that dist[n] is the 
    distance of the shortest path to n. If end_node is not None, then
    dist is just the distance from start_node to end_node.
    - previous is a dictionary, such that previous[n] is the previous node
    in the path from start_node to n.
    """
    has_neg_edge = False
    for e in g.edges():
        if e[2] < 2:
            has_neg_edge = True
            break
    
    if has_neg_edge:
        (dist, previous) = bellman_ford(g, start_node)
        if not end_node is None:
            dist = dist[end_node]
    else:
        (dist, previous) = dijkstra(g, start_node, end_node)
    return (dist, previous)

def all_shortest_paths(g):
    raise GraphFunctionNotImplemented('')

def span_tree(g, start_node = None):
    """Finds a spanning tree for an undirected graph.
    
    Args:
    - g: an undirected graph
    - start_node: The root of the returned spanning tree, or None.
    If this is None, the root will be picked randomly.
    
    Return value:
    A dictionary previous, such that previous[n] is the "parent"
    of n in the spanning tree.
    """
    
    if isinstance(g, DirectedGraph):
        raise UnsupportedGraphType('spanning tree not defined for directed graphs')
        
    if not start_node is None and not start_node in g.nodes():
        raise GraphValueError('start node does not exist')
    
    visited = {}
    weights = {}
    previous = {}
    for n in g.nodes():
        visited[n] = False
        weights[n] = float('inf')
        previous[n] = None
        
    # Min-heap of edge weights. The items in the heap have the form
    # (edge_weight_to_node, node_index).
    weight_heap = []
    heapq.heappush(weight_heap, (0, start_node))
    weights[start_node] = 0
    
    while len(weight_heap) > 0:
        node_dist = heapq.heappop(weight_heap)        
        n = node_dist[1]
        # Normally, we would not extract from the heap the same node
        # twice. However, in this implementation, we don't decrease the
        # nodes keys in the heap, rather we add copies with smaller keys.
        # This is not very efficient but we avoid having to track where
        # each node is stored in the heap.
        # TODO: Re-implement heap to make this easier. 
        if visited[n]:
            continue
        visited[n] = True
        for adj in g.weighted_neighbors(n):
            adj_node = adj[0]
            edge_weight = adj[1]
            if not visited[adj_node] and weights[adj_node] > edge_weight:
                previous[adj_node] = n
                weights[adj_node] = edge_weight
                heapq.heappush(weight_heap, (edge_weight, adj_node))
    return previous
    
def top_sort(g):
    """Returns a (potentially partial) topological sort of g.
    
    Args:
    - g: a directed graph
    
    Return value:
    A list L of nodes of g in topological order, i.e. such that there are
    no edges going from the L[i]-th node to any of the nodes in L[0:i]. The
    return list might not contain all nodes of g if a topological order for 
    all nodes cannot be found (eg. if g has cycles).
    """
    
    if isinstance(g, Graph):
        raise UnsupportedGraphType('spanning tree not defined for undirected graphs')

    # Count the number of incoming edges to each node
    num_incoming = {}
    for n in g.nodes():
        num_incoming[n] = 0
    for e in g.edges():
        num_incoming[e[1]] += 1
        
    # Queue of nodes with no incoming edges
    roots = deque([])
    output = []
    for n in g.nodes():
        if num_incoming[n] == 0:
            roots.append(n)
    
    while len(roots) > 0:
        n = roots.popleft()
        output.append(n)
        for adj in g.neighbors(n):
            num_incoming[adj] -= 1
            if num_incoming[adj] == 0:
                roots.append(adj)
                
    return output
        
def components(g):
    """Finds the connected components of the input graph.

    Return value:
    A list C of connected components (or strongly connected components if g is
    directed). 
    """

    return g.components()
