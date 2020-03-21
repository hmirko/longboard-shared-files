#!/usr/bin/env python
from time import sleep
import numpy as np
import matplotlib.pyplot as plt

brk_a = -0.00208
brk_b = 2.066
brk_c = 0.0

acc_a = 0.00253
acc_b = -2.8884
acc_c = 1326.82982

plt.clf()
plt.pause(.001)
ax = plt.gca()
ax.set_ylim(bottom = 0, top = 1024)
ax.set_xlim(left = 0, right = 1024)
ax.autoscale_view()

x = np.linspace(0, 1024, 1024)

brk = lambda x: brk_a * x**2 + brk_b * x + brk_c
fx_brk = brk(x)
print(fx_brk)
fx_brk[512:] = np.nan

acc = lambda x: acc_a * (1024 - x) ** 2 + acc_b * (1024 - x) + acc_c
fx_acc = acc(x)
print(fx_acc)
fx_acc[:511] = np.nan

my_dpi=192
plt.figure(figsize=(800/my_dpi, 800/my_dpi), dpi=my_dpi)

print plt.get_backend()

plt.style.use('classic')
plt.plot(fx_acc, color='r')
plt.plot(fx_brk, color='b')
#    plt.show(block=False)
plt.pause(.001)
fig=plt.figure()
fig.show()
plt.show()
