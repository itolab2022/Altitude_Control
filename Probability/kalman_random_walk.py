import pprint
import numpy as np
import matplotlib.pyplot as plt
import math

#位置
Xn_pre = 0
Xn_est = 0
Xn_list = []
#位置の誤差のばらつき（分散）
sigma_xn_pre = 100
sigma_xn_est = 100
sigma_xn_list = []
#システムノイズのばらつき（分散）
sigma_x = 1
#観測した値のばらつき（分散）
sigma_l = 10
#Kalman Gain
K = (sigma_xn_pre**2)/(sigma_xn_pre**2 + sigma_l**2)
#速度
v = 5
#time
t = 10
#刻み幅
h = 0.01
#要素数
n = int((t/h) + 1)

x = np.linspace(0,t,n)


rand_loc = np.random.normal(loc = 0,scale=sigma_x,size = n,)
rand_obs = np.random.normal(loc = 0,scale=sigma_l,size = n,)

rand_loc_v = rand_loc + (v*h)

location_data_noise = np.cumsum(rand_loc_v)
observe_data_noise = location_data_noise + rand_obs


for i in range (n):
    
    last_sigma_xn_est = sigma_xn_est
    Xn_pre = Xn_est
    
    #位置の推定のばらつきの予測
    sigma_xn_pre = np.sqrt(last_sigma_xn_est**2 + sigma_x**2)
    #位置の推定のばらつきの推定
    sigma_xn_est = np.sqrt((1-K)*sigma_xn_pre**2)
    sigma_xn_list.append(sigma_xn_pre)
    #カルマンゲイン
    K = (sigma_xn_pre**2)/(sigma_xn_pre**2 + sigma_l**2)
    #位置推定式
    Xn_est = Xn_pre + K*(observe_data_noise[i] - Xn_pre)
    Xn_list.append(Xn_est)
    
fig = plt.figure(figsize=(10,6))
ax1 = fig.add_subplot(2,1,1)
ax2 = fig.add_subplot(2,1,2)

ax1.plot(x,location_data_noise,label ='True value')
ax1.plot(x,observe_data_noise,label ='observation')
ax1.plot(x,Xn_list,label ='location')

ax2.plot(x,sigma_xn_list,label ='predict scattering of location')

plt.xlabel('Time')
plt.ylabel('Location')
plt.title('observation + predict of location')
ax1.legend()
ax2.legend()
plt.show()
