import heapq


def move(loc, direction):
    # W, S, E, N, current
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
    return loc[0] + directions[direction][0], loc[1] + directions[direction][1]


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


def build_constraint_table(constraints, agent, positive):
    ##############################
    # Task 1.2/1.3: Return a table that contains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the 
    #               is_constrained function.
    constraint_table = dict()
    for constraint in constraints:
        if constraint['agent'] == agent and constraint.get('positive', False) == positive:
            constraint_loc_key = tuple(constraint['loc'])
            constraint_time_step = constraint['time_step']
            try:
                constraint_table[constraint_time_step].add(constraint_loc_key)
            except KeyError:
                constraint_table[constraint_time_step] = set()
                constraint_table[constraint_time_step].add(constraint_loc_key)
    # print(f"agent {agent}: constraint_table={constraint_table}")
    return constraint_table


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


def is_constrained(curr_loc, next_loc, next_time, neg_constraint_table):
    ##############################
    # Task 1.2/1.3: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.
    constraints = []
    constraints.extend(neg_constraint_table.get(-1, []))
    constraints.extend(neg_constraint_table.get(next_time, []))
    constrained = (next_loc,) in constraints or (curr_loc, next_loc) in constraints
    # print(f"curr_loc={curr_loc}, next_loc={next_loc}, next_time={next_time}, "
    #       f"constrained={constrained}, constraints={constraints}")
    return constrained


def push_node(open_list, node):
    heapq.heappush(open_list, (
        node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


#############################
# Task 2.4: need to pass previous agent's path to calculate max_time_step
# def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints, paths):
def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """
    my_map      - binary obstacle map
    start_loc   - start position
    goal_loc    - goal position
    agent       - the agent that is being re-planned
    constraints - constraints defining where robot should or cannot go at each timestep
    max_time_step - upper bound of the search
    """

    ##############################
    # Task 1.1: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.
    open_list = []
    closed_list = dict()
    neg_constraint_table = build_constraint_table(constraints, agent, False)
    pos_constraint_table = build_constraint_table(constraints, agent, True)
    ##############################
    # Task 2.4
    # max_time_step = len(my_map) * len(my_map[0])
    # for path in paths:
    #     max_time_step += len(path)
    try:
        earliest_goal_timestep = max(neg_constraint_table)
    except ValueError:
        earliest_goal_timestep = 0
    # print(f"agent {agent}: earliest_goal_timestep={earliest_goal_timestep}")
    h_value = h_values[start_loc]
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'time_step': 0}
    push_node(open_list, root)
    closed_list[(root['loc'], 0)] = root
    while len(open_list) > 0:
        curr = pop_node(open_list)
        curr_loc = curr['loc']
        curr_time_step = curr['time_step']
        #############################
        # Task 2.4: Set an upper bound to the search
        # if curr_time_step > max_time_step:
        #     print(f"agent {agent}: max_time_step={max_time_step} reached!")
        #     return None
        #############################
        # Task 1.4: Adjust the goal test condition to handle goal constraints
        if curr_time_step >= earliest_goal_timestep and curr_loc == goal_loc:
            return get_path(curr)
        child_time_step = curr_time_step + 1
        for direction in range(5):
            child_loc = move(curr_loc, direction)
            # Out of bounds check
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
                    or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
                continue
            # Blocked by obstacle or another agent
            if my_map[child_loc[0]][child_loc[1]] or is_constrained(
                    curr_loc, child_loc, child_time_step, neg_constraint_table):
                continue
            # There should only be one positive constraint at a time step
            if pos_constraint_table.get(child_time_step):
                pos_constraint = list(pos_constraint_table[child_time_step])[0]
                if (len(pos_constraint) == 1 and child_loc != pos_constraint[0]) \
                        or (len(pos_constraint) == 2 and (curr_loc, child_loc) != pos_constraint):
                    continue
            child = {'loc': child_loc,
                     'g_val': curr['g_val'] + 1,
                     'h_val': h_values[child_loc],
                     'parent': curr,
                     'time_step': child_time_step}
            child_node_key = (child_loc, child_time_step)
            if child_node_key in closed_list:
                existing_node = closed_list[child_node_key]
                if compare_nodes(child, existing_node):
                    closed_list[child_node_key] = child
                    push_node(open_list, child)
            else:
                closed_list[child_node_key] = child
                push_node(open_list, child)

    return None  # Failed to find solutions
