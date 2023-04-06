import pprint
import numpy as np
from matplotlib import pyplot

a = np.loadtxt('data.csv',delimiter=',')
time = []
observation = []

for row in a:
    time.append(row[0])
    observation.append(row[1])
    
print(type(observation[0]))
#位置
Xn_pre = 0
Xn_est = 0
Xn_list = []
#位置の誤差のばらつき（分散）
sigma_xn_pre = 100
sigma_xn_est = 100
#システムノイズのばらつき（分散）
sigma_x = 1
#観測した値のばらつき（分散）
sigma_l = 9
#Kalman Gain
K = (sigma_xn_pre**2)/(sigma_xn_pre**2 + sigma_l**2)

for i in range (len(time)):
    last_sigma_xn_est = sigma_xn_est
    Xn_pre = Xn_est
    #位置の推定のばらつきの予測
    sigma_xn_pre = np.sqrt(last_sigma_xn_est**2 + sigma_x**2)
    #位置の推定のばらつきの推定
    sigma_xn_est = np.sqrt((1-K)*sigma_xn_pre**2)
    #カルマンゲイン
    K = (sigma_xn_pre**2)/(sigma_xn_pre**2 + sigma_l**2)
    #位置推定式
    Xn_est = Xn_pre + K*(observation[i] - Xn_pre)
    Xn_list.append(Xn_est)
    
pyplot.plot(time,Xn_list,label ='location')
pyplot.plot(time,observation,label ='observation')
pyplot.xlabel('Time')
pyplot.ylabel('Location')
pyplot.title('observation + predict of location')
pyplot.legend()
pyplot.show()
