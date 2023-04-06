import pprint
import numpy as np
import matplotlib.pyplot as plt
import math

x = np.linspace(0,4*np.pi,500)

rand = np.random.normal(loc = 0,scale=9,size = 500,)

observe_data = 10*np.sin(x) 

observe_data_noise = observe_data + rand


#位置
Xn_pre = 0
Xn_est = 0
Xn_list = []
#位置の誤差のばらつき（分散）
sigma_xn_pre = 100
sigma_xn_est = 100
#システムノイズのばらつき（分散）
sigma_x = 0.1
#観測した値のばらつき（分散）
sigma_l = 30
#Kalman Gain
K = (sigma_xn_pre**2)/(sigma_xn_pre**2 + sigma_l**2)

for i in range (500):
    
    last_sigma_xn_est = sigma_xn_est
    Xn_pre = Xn_est
    
    #位置の推定のばらつきの予測
    sigma_xn_pre = np.sqrt(last_sigma_xn_est**2 + sigma_x**2)
    #位置の推定のばらつきの推定
    sigma_xn_est = np.sqrt((1-K)*sigma_xn_pre**2)
    #カルマンゲイン
    K = (sigma_xn_pre**2)/(sigma_xn_pre**2 + sigma_l**2)
    #位置推定式
    Xn_est = Xn_pre + K*(observe_data_noise[i] - Xn_pre)
    Xn_list.append(Xn_est)
    
plt.plot(x,Xn_list,label ='location')
plt.plot(x,observe_data,label ='sin wave')
plt.plot(x,observe_data_noise,label ='observation')
plt.xlabel('Time')
plt.ylabel('Location')
plt.title('observation + predict of location')
plt.legend()
plt.show()
