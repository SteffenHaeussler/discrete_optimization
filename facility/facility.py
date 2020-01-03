import math

import numpy as np
from ortools.linear_solver import pywraplp
from pyscipopt import Model, quicksum, multidict

def length(point1, point2):
    return math.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)


def calculate_distance_matrix(customers, facilities):

    dist_matrix = np.zeros([len(customers), len(facilities)])

    for x, i in enumerate(customers):
        for y, j in enumerate(facilities):

            distance = length(i.location, j.location)

            dist_matrix[x][y] = distance

    return dist_matrix


def or_tools_mip(customers, facilities):

    dist_matrix = calculate_distance_matrix(customers, facilities)

    solver = pywraplp.Solver('SolveAssignmentProblemMIP',
                pywraplp.Solver.CBC_MIXED_INTEGER_PROGRAMMING)

    x = [solver.BoolVar(f'x{j}') for j in range(len(facilities))]

    y = []
    for i in range(len(customers)):
        t = []
        for j in range(len(facilities)):
            t.append(solver.BoolVar(f'y{i},{j}'))

        y.append(t)

    #setup constraints
    for i in range(len(customers)):
        solver.Add(sum([y[i][j] for j in range(len(facilities))]) == 1)

    # Constraint: a customer must be assigned to an open facility.
    for i in range(len(customers)):
        for j in range(len(facilities)):
            solver.Add(y[i][j] <= x[j])

    # Constraint: the capacity of each facility must not be exceeded.
    for j in range(len(facilities)):
        solver.Add(sum([y[i][j] * customers[i].demand for i in range(len(customers))]) <= facilities[j].capacity)

    # Objective: sum all the distance and setup_costs
    solver.Minimize(solver.Sum([x[j] * facilities[j].setup_cost for j in range(len(facilities))]) +
        solver.Sum([dist_matrix[i][j] * y[i][j] for i in range(len(customers))
                                               for j in range(len(facilities))]))

    SEC = 1000
    MIN = 60 * SEC
    solver.SetTimeLimit(5 * MIN)

    result_status = solver.Solve()

    total_cost = solver.Objective().Value()
    solution = [j for i in range(len(customers))
                for j in range(len(facilities))
                if y[i][j].solution_value() > 0]

    return total_cost, solution


def pyscip_solver(customers, facilities):

    dist_matrix = calculate_distance_matrix(customers, facilities)

    sci_customers = {1+v.index: v.demand for v in customers}
    sci_facilities = {1+v.index: [v.capacity, v.setup_cost] for v in facilities}

    c = {}
    for i in range(dist_matrix.shape[0]):
        for j in range(dist_matrix.shape[1]):
            c[(i+1,j+1)] = dist_matrix[i][j]

    I, d = multidict(sci_customers)
    J, M, f = multidict(sci_facilities)

    model = Model("flp")
    x,y = {},{}
    for j in J:
        y[j] = model.addVar(vtype="B", name="y(%s)"%j)
        for i in I:
            x[i,j] = model.addVar(vtype="B", name="x(%s,%s)"%(i,j))
    for i in I:
        model.addCons(quicksum(x[i,j] for j in J) == 1, "Demand(%s)"%i)
    for j in M:
        model.addCons(quicksum(x[i,j] * d[i] for i in I) <= M[j], "Capacity(%s)"%i)
    for (i,j) in x:
        model.addCons(x[i,j] <= y[j], "Strong(%s,%s)"%(i,j))
    model.setObjective(
        quicksum(f[j]*y[j] for j in J) +
        quicksum(c[i,j]*x[i,j] for i in I for j in J),
        "minimize")
    model.data = x,y

    model.setRealParam('limits/time', 1800)

    model.optimize()
    EPS = 1.e-6
    x,y = model.data

    total_cost = model.getObjVal()
    solution = [j-1 for i in sci_customers.keys() for j in sci_facilities.keys() if model.getVal(x[i,j]) > EPS]

    return total_cost, solution
