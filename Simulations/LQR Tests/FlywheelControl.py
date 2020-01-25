import control as c
import numpy as np
import matplotlib.pyplot as plt

# Constants
G = 1
Kt = 1
Kv = 1
R = 1
J = 1

A = np.array([(-1 * G * G * Kt) / (Kv * R * J)])

B = np.array([(G * Kt) / (R * J)])

C = np.array([1])

D = np.array([0])

sys = c.ss(A, B, C, D)