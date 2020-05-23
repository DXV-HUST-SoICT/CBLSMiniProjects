from __future__ import print_function
from ortools.linear_solver import pywraplp

solver = pywraplp.Solver('MIP', pywraplp.Solver.CBC_MIXED_INTEGER_PROGRAMMING)

infinity = solver.infinity()

for 