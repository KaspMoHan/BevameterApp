import numpy as np
from matplotlib import pyplot as plt

def generate_sine_full(): # Generates a full sine wave
    x = np.linspace(0,6*np.pi,500)
    noise = np.random.normal(0,0.25,500)
    y = 2*np.sin(x) + noise
    return x,y
'''
x,y = generate_sine()
plt.plot(x,y)
plt.show()
'''

def generate_next_sine(time): # Generates a y component based on x component (e.g. time, so this gets called at
    # at a specific freq)
    noise = np.random.normal(0,0.25)
    y = 2*np.sin(time) + noise
    return y

'''
xs = []
ys = []

for t in range(500):
    x = t / 50.0
    y = generate_next_sine(x)
    xs.append(x)
    ys.append(y)

plt.plot(xs, ys, label="noisy sine")
plt.xlabel("time")
plt.ylabel("signal")
plt.legend()
plt.show()
'''
