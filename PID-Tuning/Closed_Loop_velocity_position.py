import control as co
import numpy as np
import matplotlib.pyplot as plt

K = 364.583
tau = 0.07
r = 0.014
Te = 0.014
N 	=	10

Tte = np.arange(0, 2, Te)

# Modèle moteur
num = [K]
den = [tau, 1]
Go = co.tf(num, den)

num_delay, den_delay = co.pade(r, 1)
Delay = co.tf(num_delay, den_delay)
G = Go * Delay

kp=0.0005333
kd=0.0000008
ki=0.0217687

taud	=kd/kp
taui	=kp/ki

den_c	=	np.convolve([taui,0],[taud/N,1])
num_c	=	kp*(den_c + np.array([taui*taud,0,0])+ np.array([0,taud/N,1]))
PID_C	=	co.tf(num_c,den_c)

GPID_BF = co.feedback(G*PID_C,1)
GBOZ = co.sample_system(G, Te, method='zoh', alpha=0)
PID_Z = co.sample_system(PID_C,Te, method='gbt', alpha=0.5)
GPIDZ_BF = co.feedback(GBOZ*PID_Z,1)

#consigne
v = 4275
vitesse = v * np.ones_like(Tte)
_, Yd = co.forced_response(GPIDZ_BF, Tte, vitesse)
#errur
ev = vitesse - Yd
#Commande
_, Yu = co.forced_response(PID_Z, Tte, ev)

Kp = 0.011
Integrateur = co.tf([1], [1, 0])
Gp = G * Integrateur

GpZ = co.sample_system(Gp, Te, method='zoh')
Cz = co.tf([Kp], [1], Te)

Gbf = co.feedback(Cz * GpZ, 1)

#  Consigne  de 5 tours
p=5
postion  = p * np.ones_like(Tte)
_, yp = co.forced_response(Gbf, Tte, postion)

#  errur
e = postion - yp
_, y = co.forced_response(Gbf, Tte, postion)

#  commande
u = Kp * e

plt.figure(figsize=(9,5))
plt.subplot(3,1,1)
plt.title("Response indicelle BF de vitesse ")
plt.step(Tte, Yd, where='post', label="vitesse")
plt.step(Tte, vitesse , where='post', label="consigne")
plt.ylim(0, 4500)
plt.yticks([-1,0, 1000, 2000, 3000, 4000])
plt.grid()
plt.legend()

plt.subplot(3,1,2)
plt.step(Tte, Yu, where='post', label="commande")
plt.ylim(0, 15)
plt.yticks([4, 8, 12])
plt.grid()
plt.legend()

plt.subplot(3,1,3)
plt.step(Tte, ev, where='post', label="Erreur e(t)")
plt.ylim(0, 4500)
plt.yticks([-5,0, 1000, 2000, 3000, 4000])
plt.grid()
plt.legend()
plt.xlabel("Temps (s)")
plt.figure(figsize=(10,6))

# Position
plt.subplot(3,1,1)
plt.title("Response indicelle BF de postion ")
plt.step(Tte, yp, where='post', label="Position")
plt.step(Tte, postion, where='post', label="Consigne")
plt.grid()
plt.legend()

# Erreur
plt.subplot(3,1,2)
plt.step(Tte, e, where='post', label="Erreur ")
plt.grid()
plt.legend()
#  commande

plt.subplot(3,1,3)
plt.title("commande")
plt.step(Tte, u, where='post', label="commande")
plt.grid()
plt.legend()

plt.show()

