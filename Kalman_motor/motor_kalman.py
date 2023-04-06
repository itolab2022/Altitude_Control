import numpy as np
import matplotlib.pyplot as plt
import motor

#位置の誤差のばらつき（分散）
Sigma_yn_pre = np.matrix([[1,0],[0,1]])
Sigma_yn_est = np.matrix([[1,0],[0,1]])
Sigma_yn_list_1 = []
Sigma_yn_list_2 = []
Sigma_yn_list_3 = []
Sigma_yn_list_4 = []
#速度、位置
mu_Yn_pre = np.matrix([[0],[0]])
mu_Yn_est = np.matrix([[0],[0]])
mu_y_list = []
mu_v_list = []
#システムノイズの分散（分散）
stdv_Q = motor.stdv_y
Q = stdv_Q**2
#観測した値のばらつき（分散）
stdv_R = 0.002
R = stdv_R**2
#時間
t = motor.t
t_s = 0
tc = 0
#刻み幅
h = motor.h
h_kalman = 0.02
#要素数
n = int((t/h) )
n_kalman = int((t/h_kalman))
#観測行列
observation_mat = np.matrix([[0,1]])
#観測値
rand_loc = np.random.normal(loc = 0,scale=stdv_R,size = n,)
observation_data = motor.yn_list
#システム行列
system_mat = motor.mat_1
#制御入力
u = motor.omega
#制御行列
control_mat = motor.mat_3
#単位行列
unit_mat = np.matrix([[1,0],[0,1]])
#カルマンゲイン
K = Sigma_yn_pre * observation_mat.T * 1 / (observation_mat * Sigma_yn_pre * observation_mat.T + R)

T = np.linspace(0,t,n_kalman)

for i in range(n):
    if t_s >= tc:
        tc = tc + h_kalman
        
        last_mu_Yn_pre = mu_Yn_est
        last_Sigma_Yn_pre = Sigma_yn_est
        
        mu_Yn_pre = (system_mat * last_mu_Yn_pre) + (control_mat * u) 
        Sigma_yn_pre = (system_mat * last_Sigma_Yn_pre * system_mat.T) + Q
        
        K = Sigma_yn_pre * observation_mat.T * 1 / ((observation_mat * Sigma_yn_pre * observation_mat.T ) + R)
        
        L = np.matrix([[motor.vn_list[i]],[observation_data[i]]])
        Z = observation_mat * L
        mu_Yn_est = mu_Yn_pre + K * (Z - observation_mat * mu_Yn_pre)
        Sigma_yn_est = (unit_mat - (K * observation_mat)) * Sigma_yn_pre
        
        mu_v_list.append(mu_Yn_est[0,0])
        mu_y_list.append(mu_Yn_est[1,0])
        Sigma_yn_list_1.append(Sigma_yn_est[0,0])
        Sigma_yn_list_2.append(Sigma_yn_est[0,1])
        Sigma_yn_list_3.append(Sigma_yn_est[1,0])
        Sigma_yn_list_4.append(Sigma_yn_est[1,1])
    t_s = t_s + h
    
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
ax1.plot(motor.T_m,motor.vn_list,label = "True velocity")
ax1.grid()
ax1.set_xlabel("time")
ax1.set_ylabel("V[m/s]")

ax2.plot(T,mu_y_list,label = "estimate location")
ax2.plot(motor.T_m,motor.yn_list,label = "True location")
ax2.grid()
ax2.set_xlabel("time")
ax2.set_ylabel("location[m]")

ax3.plot(motor.T_m,observation_data,label = "Observedata")
ax3.grid()
ax3.set_xlabel("time")
ax3.set_ylabel("location[m]")

# ax4.plot(spring.T_s,spring.vn_list,label = "True velocity")
# ax4.plot(spring.T_s,spring.xn_list,label = "True location")
# ax4.grid()

ax5.plot(T,Sigma_yn_list_1,label = "estimate sigma")
ax5.set_xlim(0,1)
ax5.grid()
ax5.set_xlabel("time")
ax5.set_ylabel("sigma")

ax6.plot(T,Sigma_yn_list_2,label = "estimate sigma")
ax6.set_xlim(0,1)
ax6.grid()
ax6.set_xlabel("time")
ax6.set_ylabel("sigma")

ax7.plot(T,Sigma_yn_list_3,label = "estimate sigma")
ax7.set_xlim(0,1)
ax7.grid()
ax7.set_xlabel("time")
ax7.set_ylabel("sigma")

ax8.plot(T,Sigma_yn_list_4,label = "estimate sigma")
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