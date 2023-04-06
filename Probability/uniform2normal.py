import numpy as np
import scipy.stats as stats
import matplotlib.pyplot as plt

rng = np.random.default_rng()

#ボックスミュラー法を適用する関数
def box_muller(dataset_1,dataset_2):
    X = np.sqrt(-2*np.log(dataset_1)) * np.cos(2*np.pi*dataset_2)
    Y = np.sqrt(-2*np.log(dataset_1)) * np.cos(2*np.pi*dataset_2)
    return X, Y

#一様乱数を生成]
#rand_uni_1,rand_uni_2 = rng.uniform(0,101,(2,10000))
rand_uni_1,rand_uni_2 = np.random.uniform(0, 101, (2, 1000000))

#ボックスミュラー法を適用
rand_nor_1,rand_nor_2 = box_muller(rand_uni_1,rand_uni_2)

#plot
fig = plt.figure(figsize=(10,6))
ax1 = fig.add_subplot(2,1,1)
ax2 = fig.add_subplot(2,1,2)
ax1.hist(rand_uni_1)
ax2.hist(rand_nor_1,bins=100)

plt.show()