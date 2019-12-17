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


def reconstruct_knapsack(matrix, items):
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
