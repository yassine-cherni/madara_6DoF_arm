import control as co
import numpy as np
import matplotlib.pyplot as plt

# Paramètres du système
K = 364.583
tau = 0.07
r = 0.014
te = 0.014
tp = 0.1

T = np.arange(0, 10, tp)
Tte= np.arange(0, 10, te)

# Fonction de transfert du moteur s
num = [K]
den = [tau, 1]
G = co.tf(num, den)

#  Approximation du retard
num_delay, den_delay = co.pade(r, 1)
Delay = co.tf(num_delay, den_delay)
G_total = G * Delay

GZ = co.sample_system(G_total, te, method='zoh', alpha=0)

U = 12
_, y = co.step_response(G_total * U, T)
_, yz = co.step_response(G_total * U, Tte)

plt.title("Réponse indicielle du moteur en BO")
plt.plot(T, y,label="asservissement continu")
plt.step(Tte,yz,label = "asservissement discret")
plt.xlabel("Temps (s)")
plt.ylabel("Vitesse du moteur (rad/s)")
plt.grid(True)
plt.legend()
plt.show()
