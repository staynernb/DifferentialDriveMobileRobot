import os
import numpy as np
import matplotlib.pyplot as plt
import control as ct
import control.flatsys as fs

# Constantes do sistema
R = 0.1 # Raio de cada roda é 0,1m
M = 9 # Massa do robô sem as rodas mais a massa de cada roda (7 + 2*1) kg
d = 0.05 # Distância da origem no sistema de coordenadas do robô até o centro de massa
J = 1 # Momento de inércia da plataforma sobre o eixo vertical no centro de massa
b = 0.26 # Base (largura) do robô 
L = b / 2

y0 = [0, 0]
tau_r = 1
tau_l = 0
t = np.linspace(0, 5, 101)

def newton(y, t, tau_r, tau_l):
  v_u, w = y
  dydt = [d * w ** 2 + (tau_r + tau_l)/(M*R), -M * d * v_u * w / (M * d ** 2 + J) + L*(tau_r - tau_l)/R]

  return dydt

from scipy.integrate import odeint
sol = odeint(newton, y0, t, args=(tau_r, tau_l))
print(sol)

plt.plot(t, sol[:, 0], 'b', label='v_u(t)')
plt.plot(t, sol[:, 1], 'g', label='w(t)')
plt.grid()
plt.show()
