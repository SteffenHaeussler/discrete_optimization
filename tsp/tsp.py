import math

import numpy as np
import numpy.ma as ma
import networkx as nx

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

from pyscipopt import Model, quicksum, multidict


def calculate_distance_matrix(n_nodes, points):

    dist_matrix = np.zeros([n_nodes, n_nodes])

    for i in range(0, n_nodes-1):
        for j in range(1, n_nodes):
            if i >= j:
                pass

            distance = length(points[i], points[j])

            dist_matrix[i][j] = distance
            dist_matrix[j][i] = distance

            if ((i % 10000) == 0) and (j % 10000) == 0:
                print(f"distance matrix iteration checkv{i} and {j}")

    return dist_matrix


def length(point1, point2):
    return math.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)


def greedy_solution(n_nodes, points, dist_matrix):

    solution = []
    mask = [0]*n_nodes
    unvisited = set(range(0, n_nodes))

    current = 0

    while unvisited:

        solution.append(current)
        unvisited.remove(current)

        mask[current] = 1

        current = np.argmin(ma.masked_array(dist_matrix[current], mask=mask))

    return solution


def calculate_length(path, dist_matrix, cirlce=True):
    if cirlce:
        obj = float(dist_matrix[path[-1], path[0]])
    else:
        obj = 0
    for index in range(0, len(path)-1):
        obj += float(dist_matrix[path[index], path[index+1]])
    return obj


def seq_opt_tsp(path, dist_matrix, k=2):

    path = path.copy()
    orig_length = calculate_length(path, dist_matrix, True)

    for ste in range(2,k+1)[::-1]:

        for node in range(0,len(path)):

            begin = node-ste
            end = node

            if begin < 0:
                print("test")
                sequence = path[begin:] + path[:end]
            else:
                sequence = path[begin:end]

            if begin < 0:
                new_path = sequence[::-1] + path[end:begin]
            else:
                new_path = path[:begin] + sequence[::-1] + path[end:]

            if len(new_path) != 51:
                import ipdb; ipdb.set_trace()
            new_length = calculate_length(new_path, dist_matrix, True)

            if new_length < orig_length:

                orig_length = new_length
                path = new_path.copy()

    return path


def find_shortest_path(solution, dist_matrix):

    counter = 1
    best = False
    best_length = calculate_length(solution, dist_matrix, True)

    while not best:

        print(f"{counter}. sequence iteration with length {best_length}")
        # sure calculation wont take too much time
        if len(solution) < 1000:
            seq_len = len(solution)//2
        elif len(solution) < 10000:
            seq_len = 500
        else:
            seq_len = 2

        #todo: smart way to memoize
        solution = seq_opt_tsp(solution, dist_matrix, seq_len)

        new_length = calculate_length(solution, dist_matrix, True)

        counter += 1

        if new_length < best_length:

            best_length = new_length

        else:
            best = True

    return solution


def create_or_model(dist_matrix):
    """Stores the data for the problem."""
    data = {}
    data['distance_matrix'] = dist_matrix.tolist() # yapf: disable
    data['num_vehicles'] = 1
    data['depot'] = 0

    return data


def get_solution(manager, routing, assignment):

    route_distance = 0
    index = routing.Start(0)
    solution = []

    while not routing.IsEnd(index):
        solution.append(manager.IndexToNode(index))
        previous_index = index
        index = assignment.Value(routing.NextVar(index))
        route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)

    return solution, route_distance


def get_or_solution(data, time_limit):

    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)

        return data['distance_matrix'][from_node][to_node]

    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                        data['num_vehicles'], data['depot'])

    routing = pywrapcp.RoutingModel(manager)

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()

    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.TABU_SEARCH)

    search_parameters.time_limit.seconds = time_limit
    # search_parameters.log_search = True

    assignment = routing.SolveWithParameters(search_parameters)

    if assignment:
        solution, _ = get_solution(manager, routing, assignment)

    return solution


def solve_tsp(V,c):

    def addcut(cut_edges):
        G = nx.Graph()
        G.add_edges_from(cut_edges)
        Components = list(nx.connected_components(G))
        if len(Components) == 1:
            return False
        model.freeTransform()
        for S in Components:
            model.addCons(quicksum(x[i,j] for i in S for j in S if j>i) <= len(S)-1)
        return True


    model = Model("tsp")
    model.hideOutput()
    x = {}
    for i in V:
        for j in V:
            if j > i:
                x[i,j] = model.addVar(ub=1, name="x(%s,%s)"%(i,j))
    for i in V:
        model.addCons(quicksum(x[j,i] for j in V if j < i) + \
                    quicksum(x[i,j] for j in V if j > i) == 2, "Degree(%s)"%i)
    model.setObjective(quicksum(c[i,j]*x[i,j] for i in V for j in V if j > i), "minimize")
    EPS = 1.e-6
    isMIP = False

    while True:
        model.optimize()
        edges = []
        for (i,j) in x:
            if model.getVal(x[i,j]) > EPS:
                edges.append( (i,j) )
        if addcut(edges) == False:
            if isMIP:     # integer variables, components connected: solution found
                break
            model.freeTransform()
            for (i,j) in x:     # all components connected, switch to integer model
                model.chgVarType(x[i,j], "B")
                isMIP = True

    solution = get_mip_solution(edges)

    return solution


def get_mip_solution(a):

    start = a.pop(0)
    next_node = start[1]
    sol = [start[0], start[1]]

    while a:

        next_edge = [n for n,i in enumerate(a) if next_node in i]
        next_edge = a.pop(next_edge[0])
        next_node = [n for n in next_edge if n != next_node][0]
        sol.append(next_node)

    return sol[:-1]
