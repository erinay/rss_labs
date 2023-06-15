import numpy as np
import rospy

class Edge(object):
    def __init__(self, source, target):
        self.source = source
        self.target = target

class Graph(object):
    def __init__(self):
        self._nodes = set()
        self._edges = dict()

    def __contains__(self, node):
        return node in self._nodes

    def add_node(self, node):
        """Adds a node to the graph."""
        self._nodes.add(node)
    
    def add_edge(self, node1, node2):
        """Adds an edge between node1 and node2. Adds the nodes to the graph first
        if they don't exist."""
        self.add_node(node1)
        self.add_node(node2)
        node1_edges = self._edges.get(node1, set())
        node1_edges.add(Edge(node1, node2))
        self._edges[node1] = node1_edges
    
    def nearest(self, new_pos):
        # rospy.loginfo('searching')

        mindist = np.inf
        xnearest = None
        # str_msg = 'new pos is:'+str(new_pos)
        new_pos_x = np.asarray(new_pos)
        # rospy.loginfo(len(self._nodes))
        for node in self._nodes:
            xnear = np.asarray(node)
            dist = np.linalg.norm(new_pos_x-xnear)

            # if(dist<mindist and dist>2):
            if(dist<mindist):
                mindist = dist
                xnearest = node

        # rospy.loginfo('nearest is'+str(xnearest))
        return xnearest    

class SearchNode(object):
     def __init__(self, state, parent_node=None, cost=0):
        self._parent = parent_node
        self._state = state
        self._cost = cost

class Path(object):
    """This class computes the path from the starting state until the state specified by the search_node
    parameter by iterating backwards."""
    def __init__(self, search_node):
        self.path = []
        node = search_node
        while node is not None:
            self.path.append(node._state)
            node = node._parent
        self.path.reverse()

    def __repr__(self):
        return "Path of length %d: %s" % (len(self.path), self.path)
