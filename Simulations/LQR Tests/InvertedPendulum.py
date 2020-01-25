# NOTE: I tried to create the equations of motion for this system using Lagrangian Mechanics by myself, and it took a lot time.
# I highly recommend that you use already-created equations of motion for this
# Reference: https://www.academia.edu/4468049/Controlling_an_Inverted_pendulum_using_state_space_modeling_method_step_by_step_design_guide_for_control_students_

import numpy as np
import control as c
import matplotlib.pyplot as plt
import math

# Constants for:
# cart mass,
# pendulum mass,
# friction coefficient,
# length to CoM of pendulum,
# moment of inertia of pendulum
# gravity

M = 0.5
m = 0.5
b = 0.1
l = 0.3
I = 0.006
g = 9.8
p = I*(M+m)+(M*m*l*l)

# t1 = (m * g * l * (M + m)) / ((I * (m + M)) + (M * m * l * l)) # First coefficient for angular acceleration (A Matrix)
# t2 = (m * l * b) / (I * (m + M) * m * M * l * l) # Second coefficient for angular acceleration (A Matrix)
# t3 = -1 * ((m * l) / ((I * (m + M)) + (M * m * l * l))) # Third coefficient for angular acceleration (B Matrix)
#
# x1 = -1 * (g * m * m * l * l) / (I * (M + m) + (M * m * l * l)) # First coefficient for linear acceleration (A Matrix)
# x2 = -1 * (b * (I + (m * l * l))) / (I * (M + m) + (M * m * l * l)) # Second coefficient for linear acceleration (A Matrix)
# x3 = (I + (m * l * l)) / (I * (M + m) + (M * m * l * l)) # Third coefficient for linear accleration (B Matrix)

# A = np.matrix([[0,  1, 0, 0],
#               [t1, 0, 0, t2],
#               [0,  0, 0, 1],
#               [x1, 0, 0, x2]])


A = np.array([[0,                1,   0,      0              ],
               [m*g*l*(M+m)/p,    0,   0,     (m*l*b)/p       ],
               [0,                0,   0,      1              ],
               [-1*m*m*g*l*l/p,   0,   0,     -1*(I+m*l*l)*b/p]])



B = np.array([[0],
               [(-1 * m * l)/p],
               [0],
               [(I+m*l*l)/p]])

C = np.array([[0, 0, 1, 0],
               [1, 0, 0, 0]])

D = np.array([[0],
              [0]])

Q = np.matrix([[1, 0, 0, 0], # Weight angular displacement
               [0, 1, 0, 0], # Weight angular velocity
               [0, 0, 1, 0], # Weight linear displacement
               [0, 0, 0, 1]]) # Weight linear velocity

R = 100 # Weight input force

K1, S, E = c.lqr(A, B, Q, R)

sys = c.ss(A, B, C, D)

x0 = np.array([[-1 * math.pi / 4],
               [0],
               [0],
               [0]])
# P = np.array([-2.433, -1.622, complex(-0.811,1.584), complex(-0.811,-1.584)]) # Stabilizes
# K2 = c.place(A, B, P)

Acl = np.array(A - B * K1)
syscl = c.ss(Acl, B, C , D)

step = c.step_response(syscl, X0=x0)


#

plt.figure(1)
yout, T = step
plt.plot(yout.T, T.T)
plt.xlabel("Time (Seconds)")
plt.ylabel("Amplitude")
plt.show()  # Graph step response with time


