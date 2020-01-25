import matplotlib.pyplot as plt
import control as c
import control.matlab as matlab
from numpy import linalg as LA
import numpy as np

A = np.array([[0, 1],
              [2, -1]])

B = np.array([[1],
              [0]])
C = np.array([1, 0])
D = np.array([0])

sys = c.ss(A, B, C, D)  # Create state-space model of sample system
stepU = c.step_response(sys)

E = LA.eigvals(A)
print(E)# One of the eigenvalues is >0 so the energy of the system will blow up to infty
P = np.array([-2, -1])  # Place poles at -2, -1
# Note: You can change the aggressiveness by changing the poles

K = c.place(A, B, P)  # Place new poles

Acl = np.array(A - B * K)  # Create a closed loop A matrix
Ecl = LA.eigvals(Acl)  # Calculate new eigenvalues (this evaluates to the poles that I placed!)

syscl = c.ss(Acl, B, C, D)  # Closed loop state space model

step = c.step_response(syscl)

Kdc = c.dcgain(syscl)  # Calculating a dc gain in order to reach intended output goal
Kr = 1 / Kdc


sysclNew = c.ss(Acl, B * Kr, C, D)

sysclNewDiscrete = matlab.c2d(sysclNew, 0.01)

stepNew = c.step_response(sysclNew)
stepDiscrete = c.step_response(sysclNewDiscrete)

plt.figure(1)
yout, T = stepDiscrete
plt.plot(yout.T, T.T)
plt.xlabel("Time (Seconds)")
plt.ylabel("Amplitude")
plt.show()  # Graph step response with time

# plt.figure(2)
# yout, T = stepNew
# plt.plot(yout.T, T.T)
# plt.xlabel("Time (Seconds)")
# plt.ylabel("Amplitude")
# plt.show()  # Graph step response with time
