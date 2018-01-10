class SearchNode(object):
    def __init__(self, state, parent_node=None, cost=0.0, child_node=None):
        self._parent = parent_node
        self._state = state
        self._cost = cost
        self._child = child_node

    def __repr__(self):
        return "<SearchNode (id: %s)| state: %s, cost: %s, parent_id: %s>" % (id(self), self.state,
                                                                              self.cost,id(self.parent))
    
    @property
    def state(self):
        return self._state

    @property
    def parent(self):
        return self._parent

    @parent.setter
    def parent(self, node):
        self._parent = node

    @property
    def cost(self):
        return self._cost

    @property
    def child(self):
        return self._child

    @child.setter
    def child(self, node):
        self._child = node