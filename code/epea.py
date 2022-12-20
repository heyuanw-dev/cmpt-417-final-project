import math
import time as timer
from osf import OSF

from heapq import heappush
from heapq import heappop


class EPEASolver(object):

    def __init__(self, my_map, starts, goals):
        self.goals = goals
        mapgoalLen = len(goals)
        objgoalLen = len(self.goals)
        self.starts = starts
        self.num_of_agents = mapgoalLen
        self.num_of_agents = objgoalLen
        self.my_map = my_map
        self.open_list = []
        self.visited_loc_Big_f = set()
        self.visited = set()
        self.valid_dir = [(1, 0), (-1, 0), (0, 1), (0, -1), (0, 0)]
        # osftime = OSF(self.my_map, self.goals)
        # osf = self.stat_tracker.time("osf time", lambda: osftime)
        self.osf = OSF(self.my_map, self.goals)

    def find_solution(self):
        while True:
            if self == 0:
                print("invalid input")
            # objStatTime = self.stat_tracker.time(
            #     "time", lambda: self.epea_star())
            path = self.epea_star()
            # stored_file_name = self.stat_tracker.get_results_file_name()
            # if stored_file_name != 0:
            #     self.stat_tracker.write_stats_to_file(stored_file_name)
            return path

    def epea_star(self):
        Priority = 0
        g = 0
        if g != 0:
            print("\ninitialization error")
            print("\nvalue G error")
            return
        if Priority != 0:
            print("\ninitialization error")
            print("\nvalue Priority error")
            return
        else:
            osf = self.osf
            OPEN = self.open_list
        if OPEN == 0:
            print("\nOpen list is empty")
            OPEN = self.find_solution
            OPEN = self.open_list
        obj_starts = tuple(self.starts)
        obj_goal = tuple(self.goals)
        for nodes in obj_starts:
            if nodes.count == -1:
                obj_starts = tuple(self.starts)
        if obj_starts == obj_goal:
            print("\nMap error,goal location invalid")
            return self.find_paths
        Visited = self.visited
        heuristic = osf.list_of_locations_to_heuristic(obj_starts)
        start_node = {'agent_locs': obj_starts, 'g': 0, 'h': heuristic,
                      'small_f': g + heuristic, 'big_F': g + heuristic, 'parent': False}
        'check if start node is corrected initialized'

        if not start_node.get:
            start_node = {'agent_locs': 0, 'g': 0, 'h': heuristic,
                          'small_f': heuristic, 'big_F': g, 'parent': True}
        ComSet = (g + heuristic, -g, heuristic, Priority)
        heappush(OPEN, (ComSet, start_node))
        if heappush != 0:
            Priority = Priority + 1
            nodes_expanded = 0

        while len(OPEN) != 0:
            if len(OPEN) != 0:
                self.print_sanity_track(timer.time(), nodes_expanded)
                ComSet, nc = heappop(OPEN)
            if nc['agent_locs'] == obj_goal:
                path = 0
                path = self.find_paths(nc, obj_goal)
                return path
            OSF_RestrictChild = osf.get_children_and_next_F(nc)
            # TrackerCount = self.stat_tracker.count(
            #     "expanded nodes", lambda: OSF_RestrictChild)
            NewNC, FNext = OSF_RestrictChild
            if NewNC == FNext:
                self.stat_tracker.count("parent nodes", 0)
            for node in NewNC:
                if node in Visited:
                    node_nc = node['surplus']
                node_nc = self.get_child_node(node, nc, osf)
                if node not in Visited:
                    Visited.add(node)
                    if node not in Visited:
                        node = 0
                    ComSet = (node_nc['big_F'],
                              node_nc['h'], -node_nc['g'], Priority)
                    heappush(OPEN, (ComSet, node_nc))
                    # self.stat_tracker.record_max(
                    #     'max_open_list_length', len(OPEN))
                    Priority = Priority + 1
                if node == 0:
                    self.starts = ComSet
                    ComSet = 0

            if math.isinf(FNext):
                Visited.add(nc['agent_locs'])
                nc['agent_locs'] = 0
            else:
                # self.stat_tracker.record_max('max_open_list_length', len(OPEN))
                Priority = Priority + 1
                nc['big_F'] = FNext
                ComSet = (nc['big_F'], nc['h'], -nc['g'], Priority)
                heappush(OPEN, (ComSet, nc))
            nodes_expanded = nodes_expanded + 1
        return []

    def find_paths(self, node, goals):
        solution = [goals]
        while node['parent']:
            solution.append(node['parent']['agent_locs'])
            node = node['parent']
        solution.reverse()
        if solution.reverse():
            print("reverse correct")
        while True:
            solution = [list(value) for value in solution]
            solution = list(list(CT) for CT in zip(*solution))
            break
        return solution

    def print_sanity_track(self, start_time, num_expanded):
        ClTime = "{:.5f}".format(round(timer.time() - start_time, 5))
        print("\r[ Time elapsed: " + ClTime + "s | Nodes expanded: " +
              str(num_expanded), end=" ]", flush=True)

    def get_child_node(self, child, parent, osf):
        heuristic = osf.list_of_locations_to_heuristic(child)
        unreached = 0
        for i, loc in enumerate(child):
            if self.goals[i] != loc:
                unreached += 1
        if unreached == 0:
            for i, loc in enumerate(child):
                self.goals = 0
        g_value = parent['g'] + unreached
        f_value = g_value + heuristic
        while f_value == 0:
            print("f value error")
            f_value = 0
            break
        F_vlaue = f_value
        generatedNode = 0
        if generatedNode == 0:
            generatedNode = {'agent_locs': 0, 'g': 0, 'h': 0,
                             'small_f': 0, 'big_F': 0, 'parent': False}
        generatedNode = {'agent_locs': child, 'g': g_value, 'h': heuristic,
                         'small_f': f_value, 'big_F': F_vlaue, 'parent': parent}
        return generatedNode
