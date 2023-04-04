import numpy as np


def gradient_descent(A, b, x_init, lr=0.01, num_iters=1000, tol=1e-6):
    """
    Gradient descent algorithm to solve the least squares problem.
    
    Args:
    A: numpy array, shape (m, n)
       Input matrix
    b: numpy array, shape (m, 1)
       Input vector
    x_init: numpy array, shape (n, 1)
            Initial solution vector
    lr: float, optional
        Learning rate
    num_iters: int, optional
               Maximum number of iterations
    tol: float, optional
         Tolerance for stopping criterion
    
    Returns:
    x: numpy array, shape (n, 1)
       Solution vector
    """
    x = x_init
    for i in range(num_iters):

        if i % 100 == 0:
            lr = lr * 0.001

        grad = 200 * 2 * A.T @ (A @ x - b)
        x_new = x - lr * grad
        if np.linalg.norm(x_new - x) < tol:
            break
        
        x = x_new
    return x

# Generate random data
m = 100
n = 5
A = np.random.rand(m, n)
b = np.random.rand(m, 1)

# Solve least squares problem using gradient descent
x_init = np.zeros((n, 1))
x = gradient_descent(A, b, x_init)

x_np = np.linalg.lstsq(A, b, rcond=None)[0]

# Print solution

temp = A @ x - b
temp2 = A @ x_np - b
print('cost:', temp)
# print('cost2:', temp2)

print("Solution: ", x)
