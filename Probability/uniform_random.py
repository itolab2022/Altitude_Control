import numpy as np
import matplotlib.pyplot as plt

#一様乱数を生成
# rand_list = []
# for i in range (10000):
#     rand = (0 - 101) * np.random.rand() + 0
#     rand = np.sqrt(rand**2)
#     rand_list.append(rand)

# rand = np.random.uniform(low=0.0,high=101,size=10000)
rng = np.random.default_rng()
rand = rng.uniform(0,101,10000)

plt.hist(rand)
plt.show()