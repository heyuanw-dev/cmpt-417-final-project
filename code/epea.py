import math
import time as timer
from osf import OSF

from heapq import heappush
from heapq import heappop


class EPEASolver(object):

    def __init__(self, my_map, starts, goals):
        self.goals = goals
        self.starts = starts
        self.num_of_agents = len(goals)
        self.my_map = my_map
        self.open_list = []
        self.visited_loc_Big_f = set()
        self.visited = set()
        self.valid_dir = [(1, 0), (-1, 0), (0, 1), (0, -1), (0, 0)]
        self.osf = OSF(self.my_map, self.goals)

        self.start_time = 0
        self.num_of_expanded = 0
        self.CPU_time = 0
        self.max_len_open_list = [0]

    def find_solution(self):
        self.start_time = timer.time()
        path = self.epea_star()
        return path

    def epea_star(self):
        Priority = 0
        g = 0
        osf = self.osf
        OPEN = self.open_list

        obj_starts = tuple(self.starts)
        obj_goal = tuple(self.goals)
        for nodes in obj_starts:
            if nodes.count == -1:
                obj_starts = tuple(self.starts)
        print(obj_starts)
        if obj_starts == obj_goal:
            print("\nMap error,goal location invalid")
            return self.find_paths
        Visited = self.visited
        heuristic = osf.list_of_locations_to_heuristic(obj_starts)
        start_node = {'agent_locs': obj_starts, 'g': 0, 'h': heuristic,
                      'small_f': g + heuristic, 'big_F': g + heuristic, 'parent': False}
        # 'check if start node is corrected initialized'

        if not start_node.get:
            start_node = {'agent_locs': 0, 'g': 0, 'h': heuristic,
                          'small_f': heuristic, 'big_F': g, 'parent': True}
        ComSet = (g + heuristic, -g, heuristic, Priority)
        heappush(OPEN, (ComSet, start_node))
        if heappush != 0:
            Priority = Priority + 1
            self.num_of_expanded = 0

        while len(OPEN) != 0:
            ComSet, nc = heappop(OPEN)
            if nc['agent_locs'] == obj_goal:
                path = self.find_paths(nc, obj_goal)
                self.print_result(path)
                return path
            OSF_RestrictChild = osf.get_children_and_next_F(nc)
            self.num_of_expanded += 1
            NewNC, FNext = OSF_RestrictChild
            for node in NewNC:
                node_nc = self.get_child_node(node, nc, osf)
                if node not in Visited:
                    Visited.add(node)
                    if node not in Visited:
                        node = 0
                    ComSet = (node_nc['big_F'],
                              node_nc['h'], -node_nc['g'], Priority)
                    heappush(OPEN, (ComSet, node_nc))
                    self.max_len_open_list.append(len(OPEN))
                    Priority = Priority + 1
                if node == 0:
                    self.starts = ComSet
                    ComSet = 0

            if math.isinf(FNext):
                Visited.add(nc['agent_locs'])
                nc['agent_locs'] = 0
            else:
                Priority = Priority + 1
                nc['big_F'] = FNext
                ComSet = (nc['big_F'], nc['h'], -nc['g'], Priority)
                heappush(OPEN, (ComSet, nc))
            self.num_of_expanded += 1
        return []

    def find_paths(self, node, goals):
        solution = [goals]
        while node['parent']:
            solution.append(node['parent']['agent_locs'])
            node = node['parent']
        solution.reverse()
        while True:
            solution = [list(value) for value in solution]
            solution = list(list(CT) for CT in zip(*solution))
            break
        return solution

    def print_result(self, path):
        print("\n Found a solution! \n")
        print(path)
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Max Length of open list:  {}".format(max(self.max_len_open_list)))

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
        generatedNode = {'agent_locs': child, 'g': g_value, 'h': heuristic,
                         'small_f': f_value, 'big_F': F_vlaue, 'parent': parent}
        return generatedNode
