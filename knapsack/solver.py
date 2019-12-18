#!/usr/bin/python
# -*- coding: utf-8 -*-

from collections import namedtuple
import time

import knapsack as ks

Item = namedtuple("Item", ['index', 'value', 'weight'])
relax_Item = namedtuple("Item", ['index', 'value', 'weight', 'v_w_ratio', 'relaxation'])

def solve_it(input_data):
    # Modify this code to run your optimization algorithm

    # parse the input
    lines = input_data.split('\n')

    firstLine = lines[0].split()
    item_count = int(firstLine[0])
    capacity = int(firstLine[1])

    items = []
    relax_items = []

    for i in range(1, item_count+1):
        line = lines[i]
        parts = line.split()
        items.append(Item(i-1, int(parts[0]), int(parts[1])))
        relax_items.append(relax_Item(i-1, int(parts[0]), int(parts[1]), round(int(parts[0])/int(parts[1]), 2), 0))

    start_time = time.time()

    # matrix = ks.knapsack_dp(items, capacity)
    # taken = ks.reconstruct_knapsack(matrix, items, capacity)
    # value = matrix[-1][-1]

    relax_items = sorted(relax_items, key=lambda x: x.v_w_ratio)[::-1]
    best = ks.bfs_branch_bound(relax_items, capacity)

    taken = [0]*len(items)
    value = best.value
    for i in best.items:
        taken[i] = 1

    end_time = time.time()
    print(value)
    print("This took %.2f seconds" % (end_time - start_time))

    # prepare the solution in the specified output format
    output_data = str(value) + ' ' + str(0) + '\n'
    output_data += ' '.join(map(str, taken))
    return output_data


if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1:
        file_location = sys.argv[1].strip()
        with open(file_location, 'r') as input_data_file:
            input_data = input_data_file.read()
        print(solve_it(input_data))
    else:
        print('This test requires an input file.  Please select one from the data directory. (i.e. python solver.py ./data/ks_4_0)')

