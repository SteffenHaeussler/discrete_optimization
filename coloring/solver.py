#!/usr/bin/python
# -*- coding: utf-8 -*-
import time

import coloring as color

def solve_it(input_data):
    # Modify this code to run your optimization algorithm

    # parse the input
    lines = input_data.split('\n')

    first_line = lines[0].split()
    node_count = int(first_line[0])
    edge_count = int(first_line[1])

    edges = []
    for i in range(1, edge_count + 1):
        line = lines[i]
        parts = line.split()
        edges.append((int(parts[0]), int(parts[1])))

    start_time = time.time()

    if node_count <= 500:

        max_color, solution = color.or_solver(node_count, edges)

    else:
        graph, counter = color.get_graphs(edges)
        graph = color.color_graph(graph, counter)

        solution = node_count*[0]
        for node, value in graph.items():
            solution[node] = value['color']

        max_color = len(set(solution))
    end_time = time.time()

    print("This took %.2f seconds" % (end_time - start_time))

    # prepare the solution in the specified output format
    output_data = str(max_color) + ' ' + str(0) + '\n'
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
        print('This test requires an input file.  Please select one from the data directory. (i.e. python solver.py ./data/gc_4_1)')

