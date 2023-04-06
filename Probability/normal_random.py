import numpy as np
import matplotlib.pyplot as plt

#正規乱数を生成
rand = np.random.normal(loc = 0,scale=1,size = 10000,)
    
plt.hist(rand,bins=10)
plt.show()