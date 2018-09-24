class QueueEntry(object):

    def __init__(self, current, previous, depth, total_cost, cost_to_goal):
        self.current = current
        self.previous = previous
        self.depth = depth
        self.total_cost = total_cost
        self.cost_to_goal = cost_to_goal

    def get_vertex(self):
        return self.current

    def get_previous(self):
        return self.previous

    def get_depth(self):
        return self.depth

    def get_cost_to_goal(self):
        return self.cost_to_goal

    def get_total_cost(self):
        return self.total_cost

    def get_data(self):
        return self.total_cost, self

    def __eq__(self, other):
        if isinstance(other, self.__class__):
            return self.total_cost == other.total_cost
        return NotImplemented

    def __ne__(self, other):
        if isinstance(other, self.__class__):
            return self.total_cost != other.total_cost
        return NotImplemented

    def __lt__(self, other):
        if isinstance(other, self.__class__):
            return self.total_cost < other.total_cost
        return NotImplemented

    def __gt__(self, other):
        if isinstance(other, self.__class__):
            return self.total_cost < other.total_cost
        return NotImplemented

    def __le__(self, other):
        if isinstance(other, self.__class__):
            return self.total_cost <= other.total_cost
        return NotImplemented

    def __ge__(self, other):
        if isinstance(other, self.__class__):
            return self.total_cost >= other.total_cost
        return NotImplemented

    def __hash__(self):
        return id(self)
