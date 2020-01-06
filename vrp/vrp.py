import math

import numpy as np

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


def length(customer1, customer2):
    return math.sqrt((customer1.x - customer2.x)**2 + (customer1.y - customer2.y)**2)


def calculate_distance_matrix(nodes):

    n_nodes = len(nodes)
    dist_matrix = np.zeros([n_nodes, n_nodes])

    for i in range(0, n_nodes-1):
        for j in range(1, n_nodes):
            if i >= j:
                pass

            distance = length(nodes[i], nodes[j])

            dist_matrix[i][j] = distance
            dist_matrix[j][i] = distance

            if ((i % 10000) == 0) and (j % 10000) == 0:
                print(f"distance matrix iteration checkv{i} and {j}")

    return dist_matrix


def create_or_model(dist_matrix, vehicle_count, demand, vec_cap):
    """Stores the data for the problem."""
    data = {}
    data['distance_matrix'] = dist_matrix.tolist()
    data['num_vehicles'] = vehicle_count
    data['demands'] = demand
    data['vehicle_capacities'] = vec_cap

    data['depot'] = 0

    return data


def get_solution(data, manager, routing, assignment):
    vehicle_tours = []
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        vehicle_tour = []
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            vehicle_tour.append(node_index)
            index = assignment.Value(routing.NextVar(index))
        vehicle_tour.append(manager.IndexToNode(index))
        vehicle_tours.append(vehicle_tour)

    vehicle_tours.sort(key= lambda x: len(x), reverse=True)

    return vehicle_tours


def or_vrp_solution(dist_matrix, vehicle_count, demand, vec_cap):
    """https://developers.google.com/optimization/routing/cvrp"""

    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    # Add Capacity constraint.
    def demand_callback(from_index):
        """Returns the demand of the node."""
        # Convert from routing variable Index to demands NodeIndex.
        from_node = manager.IndexToNode(from_index)
        return data['demands'][from_node]


    data = create_or_model(dist_matrix, vehicle_count, demand, vec_cap)

    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                       data['num_vehicles'], data['depot'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(demand_callback_index, 0,
                                            data['vehicle_capacities'],
                                            True,  'Capacity')

    # Setting first solution heuristic.
    # init_search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    # init_search_parameters.first_solution_strategy = (
    #     routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # # Solve the problem.
    # initial_solution = routing.SolveWithParameters(init_search_parameters)

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    search_parameters.time_limit.seconds = 1800

    solution = routing.SolveWithParameters(search_parameters)

    if solution:
        vehicle_tours = get_solution(data, manager, routing, solution)

    return vehicle_tours
