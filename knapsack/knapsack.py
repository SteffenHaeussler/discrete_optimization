from collections import namedtuple, deque

import numpy as np


def knapsack_dp(items, capacity):
    matrix = np.zeros([len(items)+1,capacity+1])

    for i in range(1, len(items)+1):
        for c in range(capacity+1):
            if items[i-1].weight > c:
                matrix[i][c] = matrix[i-1][c]
            else:
                matrix[i][c] = max(matrix[i-1][c], matrix[i-1][c-items[i-1].weight]+items[i-1].value)

    return matrix


def reconstruct_knapsack(matrix, items, capacity):
    remain_capacity = capacity
    index_list = [0] * len(items)
    for i in range(0, len(items))[::-1]:
        if (items[i].weight <=  remain_capacity) and \
        (matrix[i][int(remain_capacity-items[i].weight)]+items[i].value >= matrix[i][int(remain_capacity)]):
            index_list[items[i].index] = 1
            remain_capacity -= items[i].weight
        else:
            pass

    return index_list


def linear_relaxation(items, cap, update=False, max_value=0):
    solved = False
    n_items = len(items)

    for n, i in enumerate(items):
        if (cap - i.weight) >= 0:

            cap -= i.weight
            max_value += i.value

            if update:
                items[n] = items[n]._replace(relaxation = 1)

            if n == n_items-1:
                solved = True

            if cap == 0:
                solved = True
                break

        else:
            ratio = cap / i.weight
            max_value += i.value*ratio

            if update:
                items[n] = items[n]._replace(relaxation = ratio)

            break

    return solved, items, max_value


def bfs_branch_bound(items, capacity):

    State = namedtuple("State", ['position', 'value', 'free_capacity', 'estimate', 'items'])

    _, items, init_estimate = linear_relaxation(items, capacity, True)

    init_state = State(0, 0, capacity, init_estimate, ())
    solution = State(0, 0, capacity, init_estimate, ())

    stack = deque([init_state])
    n_items = len(items)

    while stack:

        current_state = stack.popleft()
        pos = current_state.position
        val = current_state.value
        cap = current_state.free_capacity
        estimate = current_state.estimate
        item_tuple = current_state.items

        #if next item is not choosen:
        if pos >= n_items:
            continue

        _, _, estimate = linear_relaxation(items[pos:], cap, False, val)

        if solution.value > estimate:
            continue

        new_state = State(pos+1, val, cap, estimate, item_tuple)
        stack.append(new_state)

        cap -= items[pos].weight

        if cap >= 0:

            val += items[pos].value
            item_tuple += (items[pos].index,)
            new_state = State(pos+1, val, cap, estimate, item_tuple)
            stack.append(new_state)

            if val >= solution.value:
                solution = new_state

    return solution
