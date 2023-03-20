import cvxpy as cp
import numpy as np

# Define the initial parameter size and value
n = 10
params_value = np.random.randn(n)

# Define the parameter
params = cp.Parameter(n)

# Define the variable
x = cp.Variable(n)

# Define the objective function
obj = cp.sum_squares(x)

# Define the constraints
constraints = [x >= 0]

# Define the problem
prob = cp.Problem(cp.Minimize(obj), constraints)

# Solve the problem for the initial parameter value
params.value = params_value
result = prob.solve()

# Modify the parameter value and solve the problem again
n = 5
params_value = np.random.randn(n)
params.value = params_value
result = prob.solve()
