import pprint
import numpy as np
import matplotlib.pyplot as plt
import math
import spring

#位置の誤差のばらつき（分散）
Sigma_xn_pre = np.matrix([[1,0],[0,1]])
Sigma_xn_est = np.matrix([[1,0],[0,1]])
Sigma_xn_list_1 = []
Sigma_xn_list_2 = []
Sigma_xn_list_3 = []
Sigma_xn_list_4 = []
#速度、位置
mu_pre = np.matrix([[0],[0]])
mu_est = np.matrix([[0],[0]])
mu_x_list = []
mu_v_list = []
#システムノイズのばらつき（分散）
stdv_Q = spring.stdv_x
Q = stdv_Q**2
#観測した値のばらつき（分散）
stdv_R = 0.005
R = stdv_R**2
#時間
t = spring.t
t_s = 0
tc = 0
#刻み幅
h = spring.h
h_kalman = 0.02
#要素数
n = int((t/h) + 1)
n_kalman = int((t/h_kalman) +1)
#観測行列
observe_mat = np.matrix([[0,1]])
#観測値
rand_loc = np.random.normal(loc = 0,scale=stdv_R,size = n,)
observation_data = spring.xn_list + rand_loc
#システム行列
System_mat = np.matrix([[1-(spring.c/spring.m*h_kalman) , (-spring.k/spring.m)*h_kalman],[h_kalman,1]])
#制御行列
Control_mat = np.matrix([[h_kalman/spring.m],[0]])
#制御入力
u = 0
#単位行列
Unit_mat = np.matrix([[1,0],[0,1]])
#カルマンゲイン
K = Sigma_xn_pre * observe_mat.T * 1/(observe_mat * Sigma_xn_pre * observe_mat.T + R)

T = np.linspace(0,t,n_kalman)

for i in range(n):
    if i > t*10:
        u = 1 
    if t_s >= tc:
        
        tc = tc + h_kalman
        
        last_mu_pre = mu_est
        last_Sigma_xn_pre = Sigma_xn_est
        
        mu_pre = (System_mat * last_mu_pre) + (Control_mat * u)
        Sigma_xn_pre = System_mat * last_Sigma_xn_pre * System_mat.T + Q
        K = Sigma_xn_pre * observe_mat.T * 1/(observe_mat * Sigma_xn_pre * observe_mat.T + R)
        
        L = np.matrix([[spring.vn_list[i]],[observation_data[i]]])
        Z = observe_mat * L
        mu_est = mu_pre + K*(Z - observe_mat*mu_pre)
        Sigma_xn_est = (Unit_mat - K*observe_mat)*Sigma_xn_pre
        
        mu_v_list.append(mu_est[0,0])
        mu_x_list.append(mu_est[1,0])
        Sigma_xn_list_1.append(Sigma_xn_est[0,0])
        Sigma_xn_list_2.append(Sigma_xn_est[0,1])
        Sigma_xn_list_3.append(Sigma_xn_est[1,0])
        Sigma_xn_list_4.append(Sigma_xn_est[1,1])
    t_s = t_s + h
# print(T.shape)
# print(spring.T_s.shape)
print(T.shape)
print(len(mu_v_list))
fig = plt.figure(figsize = (15,9),tight_layout = True)
ax1 = fig.add_subplot(4,4,1)
ax2 = fig.add_subplot(4,4,2)
ax3 = fig.add_subplot(4,4,3)
ax4 = fig.add_subplot(4,4,4)
ax5 = fig.add_subplot(4,4,5)
ax6 = fig.add_subplot(4,4,6)
ax7 = fig.add_subplot(4,4,7)
ax8 = fig.add_subplot(4,4,8)

ax1.plot(T,mu_v_list,label = "estimate velocity")
ax1.plot(spring.T_s,spring.vn_list,label = "True velocity")
ax1.grid()
ax1.set_xlabel("time")
ax1.set_ylabel("V[m/s]")

ax2.plot(T,mu_x_list,label = "estimate location")
ax2.plot(spring.T_s,spring.xn_list,label = "True location")
ax2.grid()
ax2.set_xlabel("time")
ax2.set_ylabel("location[m]")

ax3.plot(spring.T_s,observation_data,label = "Observedata")
ax3.grid()
ax3.set_xlabel("time")
ax3.set_ylabel("location[m]")

# ax4.plot(spring.T_s,spring.vn_list,label = "True velocity")
# ax4.plot(spring.T_s,spring.xn_list,label = "True location")
# ax4.grid()

ax5.plot(T,Sigma_xn_list_1,label = "estimate sigma")
ax5.set_xlim(0,1)
ax5.grid()
ax5.set_xlabel("time")
ax5.set_ylabel("sigma")

ax6.plot(T,Sigma_xn_list_2,label = "estimate sigma")
ax6.set_xlim(0,1)
ax6.grid()
ax6.set_xlabel("time")
ax6.set_ylabel("sigma")

ax7.plot(T,Sigma_xn_list_3,label = "estimate sigma")
ax7.set_xlim(0,1)
ax7.grid()
ax7.set_xlabel("time")
ax7.set_ylabel("sigma")

ax8.plot(T,Sigma_xn_list_4,label = "estimate sigma")
ax8.set_xlim(0,1)
ax8.grid()
ax8.set_xlabel("time")
ax8.set_ylabel("sigma")

ax1.legend()
ax2.legend()
ax3.legend()
# ax4.legend()
# plt.subplots_adjust(wspace=0.4,hspace=0.6)
plt.show()