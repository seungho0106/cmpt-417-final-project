import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost


class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals):
        """
        my_map   - list of lists specifying obstacle positions
        starts   - [(x1, y1), (x2, y2), ...] list of start locations
        goals    - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""
        start_time = timer.time()
        result = []
        constraints = [
            # # Task 1.2
            # {
            #     'agent': 0,
            #     'loc': [(1, 5)],
            #     'time_step': 4,
            # },
            # # Task 1.3
            # {
            #     'agent': 0,
            #     'loc': [(1, 2), (1, 3)],
            #     'time_step': 1,
            # },
            # # Task 1.4
            # {
            #     'agent': 0,
            #     'loc': [(1, 5)],
            #     'time_step': 10,
            # },
        ]

        for i in range(self.num_of_agents):  # Find path for each agent
            #############################
            # Task 2.4
            # path = a_star(self.my_map, self.starts[i], self.goals[i],
            #               self.heuristics[i], i, constraints, result)
            path = a_star(self.my_map, self.starts[i], self.goals[i],
                          self.heuristics[i], i, constraints)
            if path is None:
                raise BaseException('No solutions')
            result.append(path)

            ##############################
            # Task 2: Add constraints here
            #         Useful variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches
            ##############################
            for curr_node, next_node in zip(enumerate(path),
                                            enumerate(path[1:], start=1)):
                curr_time_step, curr_loc = curr_node
                next_time_step, next_loc = next_node
                for future_agent in range(i + 1, self.num_of_agents):
                    # Add vertex constraint
                    constraints.append({
                        'agent': future_agent,
                        'loc': [next_loc],
                        'time_step': next_time_step,
                    })
                    # Add edge constraint
                    constraints.append({
                        'agent': future_agent,
                        'loc': [curr_loc, next_loc],
                        'time_step': next_time_step,
                    })
                    constraints.append({
                        'agent': future_agent,
                        'loc': [next_loc, curr_loc],
                        'time_step': next_time_step,
                    })
            # Add additional constraints
            goal_loc = path[-1]
            for future_agent in range(i + 1, self.num_of_agents):
                constraints.append({
                    'agent': future_agent,
                    'loc': [goal_loc],
                    'time_step': -1,
                })

        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        return result
