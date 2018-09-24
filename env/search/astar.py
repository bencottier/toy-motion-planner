from heapq import *
from .queue_entry import QueueEntry


class AStarSearch(object):

    def __init__(self, root, goal):
        self.root = root
        self.goal = goal
        self.path_map = {}
        self.current_entry = None
        self.pq = []
        self.goal_found = False
        self.path = []
        self.total_cost = 0.0

    def search(self):
        """
        Search the graph for a path
        :return:
        """
        self.path_map = {}
        # Push root to PQ
        heappush(self.pq,
                 QueueEntry(self.root, None, 0, 0.0, 0.0).get_data())
        while len(self.pq) != 0:
            self.current_entry = heappop(self.pq)[1]
            # Process entry and assign goal_found to result
            self.goal_found = self.process_current_entry()
            # Repeat until true, at which point we return
            if self.goal_found:
                print("A*: Goal found")
                self.total_cost = self.current_entry.get_total_cost()
                self.set_path()
                return

    def process_current_entry(self):
        # Get the vertex
        current_vertex = self.current_entry.get_vertex()
        # Check if current vertex has been expanded - if so move on
        if current_vertex in self.path_map:
            return False
        # Put vertex as a child of previous vertex in the search tree
        # This also marks the previous vertex as expanded
        # if int(current_vertex.positions[-1][1]) == 92:
        #     print("A*: root?")
        self.path_map[current_vertex] = self.current_entry.previous
        # If this vertex is the goal, we are done
        if self.is_goal(current_vertex):
            return True

        # Not at the goal, continue search
        # Get the vertices connected to the current one
        connectors = current_vertex.get_connectors()
        for v in connectors.keys():
            # Only add unexpanded vertices
            if v not in self.path_map:
                # Add new entry for this vertex to PQ
                # Total cost automatically accumulates
                # Heuristic is renewed for a new entry
                total_cost = self.current_entry.total_cost + \
                             current_vertex.get_cost(v) + \
                             current_vertex.get_cost_to_goal(self.goal) - \
                             self.current_entry.cost_to_goal
                new_vertex = QueueEntry(v, current_vertex,
                    self.current_entry.depth + 1, total_cost,
                    current_vertex.get_cost_to_goal(self.goal))
                heappush(self.pq, new_vertex.get_data())
        # We had to process the entry, so the goal was not found
        return False

    def is_goal(self, v):
        return v == self.goal

    def set_path(self):
        """
        Retrieve the path from the mappings made in the search.
        """
        v = self.path_map.get(self.goal)
        self.path.append(self.goal)
        # Backtrack path from goal until root is reached
        while v != self.root:
            self.path.append(v)
            v_prev = v
            v = self.path_map.get(v_prev)
        self.path.append(self.root)
        # Reverse path for chronological order
        self.path.reverse()

    def get_total_cost(self):
        return self.total_cost
