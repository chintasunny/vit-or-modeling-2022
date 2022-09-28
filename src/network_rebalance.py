#!/usr/bin/env python2
# -*- coding: utf-8 -*-

from ortools.linear_solver import pywraplp


def network_lp():
    # Instantiate a Glop solver
    solver = pywraplp.Solver('VIT_Solver',
                             pywraplp.Solver.GLOP_LINEAR_PROGRAMMING)

    # Decision variables x_i_j: origin-destination
    x_a_1 = solver.NumVar(0.0, 20.0, 'x_a_1')
    x_a_2 = solver.NumVar(0.0, 15.0, 'x_a_2')
    x_1_3 = solver.NumVar(0.0, 5.0, 'x_a_3')
    x_2_3 = solver.NumVar(0.0, 5.0, 'x_2_3')
    x_1_b = solver.NumVar(0.0, 10.0, 'x_1_b')
    x_3_b = solver.NumVar(0.0, 10.0, 'x_3_b')
    x_2_b = solver.NumVar(0.0, 20.0, 'x_2_b')

    # Objective
    objective = solver.Objective()
    objective.SetCoefficient(x_a_1, 1)
    objective.SetCoefficient(x_a_2, 1)

    # objective sign
    objective.SetMaximization()

    # constraints: Balance constraints
    solver.Add(x_a_1 - x_1_b - x_1_3 == 0)
    solver.Add(x_a_2 - x_2_3 - x_2_b == 0)
    solver.Add(x_1_3 + x_2_3 - x_3_b == 0)

    # solve
    result_status = solver.Solve()

    # Results
    print("Status=", result_status)
    print('Maximum possible flow = ', solver.Objective().Value())

    # Solution value
    print("x_a_1=", x_a_1.solution_value())
    print("x_a_2=", x_a_2.solution_value())
    print("x_1_3=", x_1_3.solution_value())
    print("x_2_3=", x_2_3.solution_value())
    print("x_1_b=", x_1_b.solution_value())
    print("x_3_b=", x_3_b.solution_value())
    print("x_2_b=", x_2_b.solution_value())

    # Dual Value
    print("Dual value for constraint-1=", solver.constraint(0).DualValue())
    print("Dual value for constraint-2=", solver.constraint(1).DualValue())
    print("Dual value for constraint-3=", solver.constraint(2).DualValue())

    return solver.Objective().Value()


def network_ip():
    # Instantiate a Glop solver
    solver = pywraplp.Solver('Target_Solver',
                             pywraplp.Solver.CBC_MIXED_INTEGER_PROGRAMMING)

    # Decision variables x_i_j: origin-destination
    x_a_1 = solver.IntVar(0.0, 20.0, 'x_a_1')
    x_a_2 = solver.IntVar(0.0, 15.0, 'x_a_2')
    x_1_3 = solver.IntVar(0.0, 5.0, 'x_a_3')
    x_2_3 = solver.IntVar(0.0, 5.0, 'x_2_3')
    x_1_b = solver.IntVar(0.0, 10.0, 'x_1_b')
    x_3_b = solver.IntVar(0.0, 10.0, 'x_3_b')
    x_2_b = solver.IntVar(0.0, 20.0, 'x_2_b')

    # Objective
    objective = solver.Objective()
    objective.SetCoefficient(x_a_1, 1)
    objective.SetCoefficient(x_a_2, 1)

    # objective sign
    objective.SetMaximization()

    # constraints: Balance constraints
    solver.Add(x_a_1 - x_1_b - x_1_3 == 0)
    solver.Add(x_a_2 - x_2_3 - x_2_b == 0)
    solver.Add(x_1_3 + x_2_3 - x_3_b == 0)

    # solve
    result_status = solver.Solve()

    # Results
    print("Status=", result_status)
    print('Maximum possible flow = ', solver.Objective().Value())

    print("x_a_1=", x_a_1.solution_value())
    print("x_a_2=", x_a_2.solution_value())
    print("x_1_3=", x_1_3.solution_value())
    print("x_2_3=", x_2_3.solution_value())
    print("x_1_b=", x_1_b.solution_value())
    print("x_3_b=", x_3_b.solution_value())
    print("x_2_b=", x_2_b.solution_value())
    return solver.Objective().Value()


if __name__ == "__main__":
    # import solver
    lp_sol = network_lp()
    ip_sol = network_ip()
    assert lp_sol == ip_sol
    l1 = [3, 7, 9, 4]
    l2 = [6, 2, 9, 12]
    new_l = [e1 + e2 for (e1, e2) in zip(l1, l2)]
    print(new_l)
