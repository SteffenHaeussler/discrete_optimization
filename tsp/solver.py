#!/usr/bin/python
# -*- coding: utf-8 -*-
from collections import namedtuple
import time

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

import tsp


Point = namedtuple("Point", ['x', 'y'])

def solve_it(input_data):
    # Modify this code to run your optimization algorithm

    # parse the input
    lines = input_data.split('\n')

    nodeCount = int(lines[0])

    points = []
    for i in range(1, nodeCount+1):
        line = lines[i]
        parts = line.split()
        points.append(Point(float(parts[0]), float(parts[1])))


    print(nodeCount)
    start_time = time.time()

    dist_matrix = tsp.calculate_distance_matrix(nodeCount, points)
    # solution = tsp.greedy_solution(nodeCount, points, dist_matrix)
    # solution = tsp.find_shortest_path(solution, dist_matrix)

    # or tools
    data = tsp.create_or_model(dist_matrix)

    if nodeCount < 1000:

        nodes = list(range(nodeCount))

        cost = {}
        for i in range(dist_matrix.shape[0]):
            for j in range(dist_matrix.shape[1]):
                cost[(i,j)] = dist_matrix[i][j]

        solution = tsp.solve_tsp(nodes, cost)
        # solution = tsp.get_or_solution(data, time_limit)

    else:
        time_limit = min(nodeCount*3 ,1800)

        solution = tsp.get_or_solution(data, time_limit)

    end_time = time.time()

    print("This took %.2f seconds" % (end_time - start_time))

    # calculate the length of the tour
    obj = tsp.length(points[solution[-1]], points[solution[0]])
    for index in range(0, nodeCount-1):
        obj += tsp.length(points[solution[index]], points[solution[index+1]])

    # prepare the solution in the specified output format
    output_data = '%.2f' % obj + ' ' + str(0) + '\n'
    output_data += ' '.join(map(str, solution))

    return output_data


import sys

if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1:
        file_location = sys.argv[1].strip()
        with open(file_location, 'r') as input_data_file:
            input_data = input_data_file.read()
        print(solve_it(input_data))
    else:
        print('This test requires an input file.  Please select one from the data directory. (i.e. python solver.py ./data/tsp_51_1)')

