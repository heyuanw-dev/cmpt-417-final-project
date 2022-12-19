import collections
import itertools

import time as timer

from single_agent_planner import compute_heuristics, a_star


def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def detect_collisions(node1, node2):
    if len(set(node1)) != len(node2):
        return True
    for i in range(len(node1)):
        if node1[i] == node2[i]:
            return False
    return False


class Node:
    def __init__(self, costs):
        self.child = []
        self.costs = costs

    def getCost(self):
        return self.costs

    def getAllChild(self):
        return self.child

    def expand(self):
        for i in range(len(self.costs)):
            temp = []
            for j in range(len(self.costs)):
                if i == j:
                    temp.append(self.costs[j] + 1)
                else:
                    temp.append(self.costs[j])
            self.child.append(Node(temp))


class ICT:
    def __init__(self, optCost):
        self.optCost = optCost
        self.root = Node(self.optCost)
        self.open_list = collections.deque()
        self.open_list.append(self.root)
        self.closed_list = list(optCost)

    def getOpenList(self):
        return self.open_list

    def getNextNode(self):
        return self.open_list[0]

    def isVisited(self, node):
        return node.getCost() in self.closed_list

    def popNode(self):
        return self.open_list.popleft()

    def pushNode(self, node):
        if not self.isVisited(node):
            self.closed_list.append(node.getCost())
            self.open_list.append(node)

    def expandNode(self):
        node = self.getNextNode()
        node.expand()
        lst_child = node.getAllChild()
        for i in range(len(lst_child)):
            self.pushNode(lst_child[i])


class MDD:
    def __init__(self, my_map, agent, start, goal, depth, prev_mdd=None):
        self.my_map = my_map
        self.agent = agent
        self.start = start
        self.goal = goal
        self.depth = depth
        self.mdd = None
        self.prev_mdd = prev_mdd
        self.save_config = {'path': None, 'next_queue': None, 'prev_path': None, 'visited': None}
        self.buildMDD(prev_mdd=self.prev_mdd)

    # Building the MDD is very easy. We perform a breadth-first search from the start location of agent ai down to depth
    # c and only store the partial DAG which starts at start(i) and ends at goal(i) at depth c. furthermore,
    # MDDc i can be reused to build MDDc+1i . We use the term MDDc i (x, t) to denote the node in MDDc i that
    # corresponds to location x at time t. We use the term MDDi when the depth of the MDD is not important for the
    # discussion.

    # help function to build new MDD
    def saveConfig(self, path, next_path, visited, prev_queue):
        next_path = list(next_path)
        next_path.sort(key=lambda x: sum(x[0]))
        self.save_config = {'path': path, 'next_path': next_path, 'prev_path': prev_queue, 'visited': visited}

    def getConfig(self):
        return self.save_config

    def buildMDD(self, prev_mdd=None):
        root = (self.start, 0)
        if prev_mdd is None:
            visited, queue, path = set(), collections.deque(), collections.defaultdict(set)
            queue.append(root)

        else:
            config = prev_mdd.getConfig()
            queue = collections.deque()
            queue.extend(config['next_path'])
            path = config['path']
            visited = config['visited']
            for n in config['next_path']:
                path[n].update(config['prev_path'][n])

        # BFS

        next_path = set()
        prev_path = collections.defaultdict(set)
        while queue:
            curr_loc, curr_depth = queue.popleft()
            curr_node = (curr_loc, curr_depth)
            for i in range(5):
                child_loc = move(curr_loc, i)
                child_depth = curr_depth + 1
                if child_loc[0] < 0 or child_loc[0] >= len(self.my_map) or child_loc[1] == -1 or child_loc[1] >= len(
                        max(self.my_map)):
                    continue
                if not self.my_map[child_loc[0]][child_loc[1]]:
                    child = (child_loc, child_depth)
                    if child_depth <= self.depth:
                        path[child].add(curr_node)
                        if child not in visited:
                            queue.append(child)
                            visited.add(child)
                    if child_depth == self.depth + 1:
                        next_path.add(child)
                        prev_path[child].add(curr_node)

        self.saveConfig(path=path, next_path=next_path, prev_queue=prev_path, visited=visited)

        # goal
        goal = (self.goal, self.depth)
        visited, queue, mdd = set(), collections.deque(), collections.defaultdict(set)
        if not path[goal]:
            self.mdd = None
        else:
            for n in path[goal]:
                p = (n, goal)
                queue.append(p)
                visited.add(p)
            while queue:
                curr, child = queue.popleft()
                mdd[curr].add(child)
                for p in path[curr]:
                    t = (p, curr)
                    if t not in visited:
                        visited.add(t)
                        queue.append(t)
            # print(mdd)
            self.mdd = mdd


def DFSRecusive(lst_MDD, prev, curr, max_depth, visited):
    curr_nodes, curr_depth = curr
    lst_child = []
    result_mdd = [curr]
    is_goal = True
    if prev and detect_collisions(prev, curr_nodes):
        return []
    if curr in visited:
        return []
    if curr_depth > max_depth:
        return []

    for i, node in enumerate(curr_nodes):
        mdd = lst_MDD[i]
        if curr_depth < mdd.depth or mdd.goal != node:
            is_goal = False
        if curr_depth >= mdd.depth and mdd.goal == node:
            lst_child.append([mdd.goal])
            continue
        lst_child_loc = []
        for c in mdd.mdd[(node, curr_depth)]:
            lst_child_loc.append(c[0])
        lst_child.append(lst_child_loc)
    if is_goal:
        return result_mdd
    # cross product merge node
    lst_child = list(itertools.product(*lst_child))
    visited.add(curr)
    for c in lst_child:
        child = (c, curr_depth + 1)
        if child not in visited:
            temp_mdd = DFSRecusive(lst_MDD, curr_nodes, child, max_depth, visited)
            if temp_mdd:
                result_mdd.extend(temp_mdd)
                return result_mdd

    return []


def jointMDD(lst_MDD):
    lst_root = []
    lst_depth = []
    for mdd in lst_MDD:
        lst_root.append(mdd.start)
        lst_depth.append(mdd.depth)
    visited = set()
    roots = tuple(lst_root)
    result_mdd = DFSRecusive(lst_MDD, None, (roots, 0), max(lst_depth), visited)
    return result_mdd


def findPath(mdd):
    path = [[] for i in range(len(mdd[0][0]))]

    for node in mdd:
        for agent, loc in enumerate(node[0]):
            path[agent].append(loc)
    return path


def getSumCost(paths):
    rst = []
    for path in paths:
        if not path:
            return []
        rst.append(max(len(path) - 1, 0))

    return rst


class ICTSSolver(object):
    def __init__(self, my_map, starts, goals):
        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_expanded = 0
        self.CPU_time = 0
        self.max_len_open_list = [0]

        self.open_list = []
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):

        self.start_time = timer.time()

        # step 1 get init. cost
        root = {'cost': 0,
                'paths': [],
                'constraints': []}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = getSumCost(root['paths'])

        if root['cost'] is None:
            return None
        # step 2 create ICT
        ict = ICT(root['cost'])
        # step 3 search
        open_list = ict.open_list
        # print(open_list[0].getCost())
        lst_mdds = []
        while len(open_list) > 0:
            curr_nodes = ict.getNextNode()
            curr_depth = curr_nodes.getCost()
            mdds = {}
            for agent in range(len(curr_depth)):
                node = (agent, curr_depth[agent])
                # print(node)
                if node in mdds:
                    lst_mdds.append(mdds[node])
                else:
                    prev_node = (agent, curr_depth[agent] - 1)
                    # print(prev_node)
                    if prev_node in mdds:
                        mdd = MDD(self.my_map, agent, self.starts[agent], self.goals[agent], curr_depth[agent],
                                  prev_mdd=mdds[prev_node])
                        # print(mdd.save_config['path'])
                    else:
                        mdd = MDD(self.my_map, agent, self.starts[agent], self.goals[agent], curr_depth[agent],
                                  prev_mdd=None)
                    mdds[node] = mdd
                    lst_mdds.append(mdd)
            result_mdd = jointMDD(lst_mdds)
            path = findPath(result_mdd)
            if path is not None:
                self.print_results(path)
                return path
            else:
                self.num_of_expanded += 1
                ict.expandNode()
                # print(len(open_list))
                self.max_len_open_list.append(len(open_list))
            ict.popNode()
        return None

    def print_results(self,path):
        print("\n Found a solution! \n")
        print(path)
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Max Length of open list:  {}".format(max(self.max_len_open_list)))