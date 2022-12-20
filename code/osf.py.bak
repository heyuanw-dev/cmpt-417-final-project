import itertools


class OSF(object):
    def __init__(self, loc, goal, valid_dir):

        self.loc = loc
        self.goal = goal
        self.dir = valid_dir

        sa_osf = list(range(3))
        if goal == loc:
            sa_osf = None
        else:
            x = loc[0]
            y = loc[1]
            h = manhattan_distance(goal, loc)
            delta_f0 = []
            delta_f1 = []
            delta_f2 = []
            for dir in valid_dir:
                new_x = loc[0] + dir[0]
                new_y = loc[1] + dir[1]
                new_h = manhattan_distance((new_x, new_y), goal)
                if new_h < h:
                    delta_f0.append(dir)
                elif new_h == h:
                    delta_f1.append(dir)
                elif new_h > h:
                    delta_f2.append(dir)

            sa_osf[0] = [delta_f0, 0, 1]  # operators, delta f, F_next
            sa_osf[1] = [delta_f1, 1, 2]
            sa_osf[2] = [delta_f2, 2, float('inf')]
        self.osf = sa_osf


def manhattan_distance(loc1, loc2):
    return abs(loc1[0] - loc2[0]) + abs(loc1[1] - loc2[1])


def multi_agent_osf(locations, value, goals, my_map):
    num_of_agents = len(goals)
    osf_tables = []
    for i in range(num_of_agents):
        valid_dir = find_valid_directions(my_map, locations[i])
        osf_i = OSF(locations[i], goals[i], valid_dir)
        osf_tables.append(osf_i.osf)

    all_combination = osf_tables[0]
    for j in range(1, num_of_agents):
        all_combination = itertools.product(osf_tables[j], all_combination)
        # if j == num_of_agents-1:
        #     for n in all_combination:
        #         print(n)
        #     all_combination = all_combination[0] + all_combination[1]

    operators = []
    f_next = float('inf')
    for combination in all_combination:
        f = 0
        for row in combination:
            if row[0]:
                f = f + row[1]
        if f == value:
            temp = combination[0][0]
            for k in range(1, num_of_agents):
                temp = itertools.product(combination[k][0], temp)
            for n in temp:
                operators.append(n)

        elif f > value:
            f_next = min(f_next, f)

    print(operators, f_next)
    return operators, f_next


def find_valid_directions(my_map, loc):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
    valid_directions = []
    for i in range(5):
        x = loc[0] + directions[i][0]
        y = loc[1] + directions[i][1]
        if not my_map[x][y]:  # Check if the cell is a valid position
            valid_directions.append(directions[i])

    return valid_directions
