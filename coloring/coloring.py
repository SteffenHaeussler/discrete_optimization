from collections import deque, OrderedDict

from ortools.sat.python import cp_model


def update_graph(graph, counter, node_1, node_2):

    if node_1 not in graph:
        graph[node_1] = {'adjacent': (node_2,), 'pruning': [node_2], 'color': -1, 'update_counter': 1}
        counter[node_1] = 1
    else:
        counter[node_1] += 1
        graph[node_1]['update_counter'] += 1
        graph[node_1]['adjacent'] += (node_2,)
        graph[node_1]['pruning'].append(node_2)

    return graph, counter


def get_graphs(edges):

    graph = OrderedDict()
    counter = {}
    for edge in edges:
        graph, counter = update_graph(graph, counter, edge[0], edge[1])
        graph, counter = update_graph(graph, counter, edge[1], edge[0])

    return graph, counter


def color_graph(graph, counter):

    while counter:
        stack = deque([[k for k, v in sorted(counter.items(), key=lambda item: item[1], reverse=True)][0]])
        node = stack.pop()
        counter.pop(node, None)

        color_set = list(set([graph[i]['color'] for i in graph[node]['adjacent']]))

        if len(color_set) == 1 and color_set[0] == -1:
            graph[node]['color'] = 0
        else:
            possible = set(list(range(0, max(color_set)+1)))
            diff = possible.difference(set(color_set))
            if diff:
                color = min(diff)
            else:
                color = max(possible)+1
            graph[node]['color'] = color

        for i in graph.keys():
            if node in graph[i]["pruning"]:
                graph[i]["pruning"].remove(node)

        for i in counter.keys():
            counter[i] = len(graph[i]["pruning"])

    return graph


def or_solver(node_count, edges):

    solved = False
    max_color = 1

    while not solved:

        max_color += 1

        model = cp_model.CpModel()
        variables = [model.NewIntVar(0, max_color-1, f'x{i}') for i in range(node_count)]
        for edge in edges:
            model.Add(variables[edge[0]] != variables[edge[1]])

        solver = cp_model.CpSolver()
        solver.parameters.max_time_in_seconds = 30.0
        status = solver.Solve(model)

        if status == cp_model.FEASIBLE:
            print("solution feasible")
            solved = True
            solution = [solver.Value(var) for var in variables]

    return max_color, solution

