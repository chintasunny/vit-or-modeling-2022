#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import numpy as np
from ortools.linear_solver import pywraplp
from enum import Enum


class OBJ(Enum):
    wastage = 'WASTAGE'
    excess_wastage = 'EXCESS_WASTAGE'
    num_cuts = 'NUM_CUTS'

    def __str__(self):
        return self.value


def pattern_making(patterns=[], sizes=[], demand=[],
                   width=20, obj_type=OBJ.wastage):
    wastage = [width - np.dot(p, sizes) for p in patterns]
    print("wastage=", wastage)

    # model
    solver = pywraplp.Solver('VIT_Solver',
                             pywraplp.Solver.CBC_MIXED_INTEGER_PROGRAMMING)
    # decision var
    x_var = [solver.IntVar(0, solver.infinity(), "x_{}".format(i + 1))
             for i in range(len(patterns))
             ]
    print("x_var names=", x_var)

    # objective
    if obj_type == OBJ.wastage:
        solver.Minimize(solver.Sum([w * x for w, x in zip(wastage, x_var)]))

    elif obj_type == OBJ.excess_wastage:
        expr1 = solver.Sum([w * x for w, x in zip(wastage, x_var)])
        expr2 = solver.Sum(-1 * demand[i] * sizes[i] for i in range(len(sizes)))
        expr3 = solver.Sum([p[i] * sizes[i] * x for p, x in zip(patterns, x_var) for i in range(len(sizes))])
        print(expr2)
        solver.Minimize(expr1 + expr2 + expr3)
    else:  # num_cuts
        solver.Minimize(solver.Sum([x for w, x in zip(wastage, x_var)]))

    # Constraints
    for i in range(len(sizes)):
        solver.Add(solver.Sum([p[i] * x
                               for p, x in zip(patterns, x_var)])
                   >= demand[i],
                   "cons_size_{}".format(sizes[i]))

    # solve
    status = solver.Solve()
    print("status=", status)
    print('obj fn= ', solver.Objective().Value())

    for i in range(len(patterns)):
        print("number of cuts for pattern {}={}".format(
            x_var[i].name(), x_var[i].solution_value()))


if __name__ == "__main__":
    patterns = [[2, 0, 0, 0], [0, 2, 0, 0], [0, 0, 2, 1],
                [0, 0, 0, 3], [1, 1, 0, 0], [1, 0, 1, 0],
                [1, 0, 0, 1], [0, 1, 1, 0], [0, 1, 0, 2], [0, 0, 1, 2]
                ]
    size = [9, 8, 7, 6]
    demand = [511, 301, 263, 383]
    width_of_roll = 20
    pattern_making(
        patterns=patterns,
        sizes=size,
        demand=demand,
        width=width_of_roll, obj_type=OBJ.wastage)

    pattern_making(
        patterns=patterns,
        sizes=size,
        demand=demand,
        width=width_of_roll, obj_type=OBJ.excess_wastage)

    pattern_making(
        patterns=patterns,
        sizes=size,
        demand=demand,
        width=width_of_roll, obj_type=OBJ.num_cuts)
