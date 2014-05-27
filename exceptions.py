class GraphError(Exception):
    pass

class GraphValueError(GraphError):
    pass

class UnsupportedGraphType(GraphError):
    pass

class GraphFunctionNotImplemented(GraphError):
    pass
