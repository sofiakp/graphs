import copy

class Graph:
    def __init__(self):
        pass

    def components(self):
        """Finds the set of connected components of the graph.
        
        Return value:
        A list C of connected components. Each element of C corresponds to a 
        connected component and is a list of nodes in that component.
        """
        
        comps = []
    
        node_queue = deque(self.nodes())
        visited = {}
        for n in self.nodes():
            visited[n] = False
        
        while len(node_queue) > 0:
            n = node_queue.pop()
            if not visited[n]:
                c = dfs(g, n)
                for adj in c:
                    visited[adj] = True
                comps.append(c)
        return comps
