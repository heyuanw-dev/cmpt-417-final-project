import heapq


def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst


def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
                    or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
                continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values


def build_constraint_table(constraints, agent):
    ##############################
    # Task 1.2/1.3: Return a table that constains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the
    #               is_constrained function.
    constraints_table = dict()
    for constraint in constraints:
        if constraint['agent'] != agent:
            continue
        elif constraint['agent'] == agent:
            if constraint['timestep'] not in constraints_table:
                constraints_table[constraint['timestep']] = []
            constraints_table[constraint['timestep']].append(constraint)
        elif constraint['positive']:
            q = {'agent': constraint['agent'], 'loc': constraint['loc'][::-1], 'timestep': constraint['timestep'],
                 'positive': False}
            if q['timestep'] not in constraints_table:
                constraints_table[q['timestep']] = []
            constraints_table[q['timestep']].append(q)

    return constraints_table


def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    ##############################
    # Task 1.2/1.3: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.
    if next_time not in constraint_table:
        return False

    for constraint in constraint_table[next_time]:
        # 1.2
        if not constraint['positive']:
            if len(constraint['loc']) == 1:
                if constraint['loc'][0] == next_loc:
                    return True
            # 1.3
            else:
                if ((constraint['loc'][0] == curr_loc and constraint['loc'][1] == next_loc)
                        or (constraint['loc'][0] == next_loc and constraint['loc'][1] == curr_loc)):
                    return True
        else:
            if len(constraint['loc']) == 1:
                if constraint['loc'][0] == next_loc:
                    continue
                else:
                    return True
            # 1.3
            else:
                if ((constraint['loc'][0] == curr_loc and constraint['loc'][1] == next_loc)
                        or (constraint['loc'][0] == next_loc and constraint['loc'][1] == curr_loc)):
                    continue
                else:
                    return True

    return False


def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """

    ##############################
    # Task 1.1: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.

    open_list = []
    closed_list = dict()
    earliest_goal_timestep = 0
    h_value = h_values[start_loc]
    constraint_table = build_constraint_table(constraints, agent)
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep': 0}
    push_node(open_list, root)
    closed_list[(root['loc'], root['timestep'])] = root

    while len(open_list) > 0:
        curr = pop_node(open_list)
        #############################
        # Task 1.4: Adjust the goal test condition to handle goal constraints
        if curr['loc'] == goal_loc:
            if curr['timestep'] > len(max(my_map)) * len(my_map) + 1:
                return None
            if not constraint_table:
                return get_path(curr)
            else:
                max_timestep = max(constraint_table)
                if max_timestep < curr['timestep']:
                    return get_path(curr)
                else:
                    for i in range(max_timestep, curr['timestep'], -1):
                        if is_constrained(curr['loc'], curr['loc'], i, constraint_table):
                            earliest_goal_timestep = i + 1
                            break
                    if earliest_goal_timestep <= curr['timestep']:
                        return get_path(curr)
        for dir in range(5):
            if dir < 4:
                child_loc = move(curr['loc'], dir)
            else:
                child_loc = curr['loc']

            if child_loc[0] < 0 or child_loc[0] >= len(my_map) or child_loc[1] == -1 or child_loc[1] >= len(
                    max(my_map)):
                continue
            else:
                if my_map[child_loc[0]][child_loc[1]]:
                    continue
            if is_constrained(curr['loc'], child_loc, curr['timestep'] + 1, constraint_table):
                continue
            child = {'loc': child_loc,
                     'g_val': curr['g_val'] + 1,
                     'h_val': h_values[child_loc],
                     'parent': curr,
                     'timestep': curr['timestep'] + 1}
            if (child['loc'], child['timestep']) in closed_list:
                existing_node = closed_list[(child['loc'], child['timestep'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'], child['timestep'])] = child
                    push_node(open_list, child)
            else:
                closed_list[(child['loc'], child['timestep'])] = child
                push_node(open_list, child)
    return None  # Failed to find solutions
