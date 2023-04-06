import numpy as np
import matplotlib.pyplot as plt
import motor_gravity

#Kalman Filter
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
stdv_Q_k = 0.01
Q_k = stdv_Q_k**2
#観測した値のばらつき（分散）
stdv_R = 0.05
stdv_R_k = 0.1
R = stdv_R**2
R_k = stdv_R_k**2
#時間
t = 10
t_s = 0
tc = 0
#刻み幅
h = 0.001
h_kalman = 0.02
#要素数
n = int((t/h) )
n_kalman = int((t/h_kalman))

T = np.linspace(0,t,n_kalman)
T_m = np.linspace(0,t,n)

#PID
#誤差
error = 0
#目標値
r = 0
#前回の誤差 last error
last_error = 0
#制御周期
Control_T = 0.02
#Pゲイン
Kp = 20
#Iゲイン
Ki = 5
#Dゲイン
Kd = 1
#誤差の微分の近似計算
de = 0
#誤差の積分の近似計算
ie = 0

#運動方程式
##重量
m = 0.8
#プロペラ定数
Ct = 4e-5
#重力
g = 9.81
#速度
vn_list = []
vo = 0
#位置
yn_list = []
yo = 10
#制御入力
omega_o = np.sqrt(m * g / Ct)
omega = 480
# delta_T = Ct *(omega**2 - omega_o**2)
delta_T = 0
#システムノイズのばらつき（分散）
stdv_y = 0.0009

constant = np.matrix([[-g*h],[0]])
mat_1 = np.matrix([[1,0],[h,1]])
y = np.matrix([[0],[0]])
mat_3 = np.matrix([[h/m],[0]])

#観測行列
observation_mat = np.matrix([[0,1]])
#システム行列
system_mat = mat_1
system_mat[1,0] = h_kalman
#制御入力
u = 0
#制御行列
control_mat = mat_3
control_mat[0,0] = h_kalman / m
#単位行列
unit_mat = np.matrix([[1,0],[0,1]])
#カルマンゲイン
K = Sigma_yn_pre * observation_mat.T * 1 / (observation_mat * Sigma_yn_pre * observation_mat.T + R)

u_list = []
observation_list = []
for i in range(n):
    if t_s >= tc:
        tc = tc + h_kalman
        
        last_mu_Yn_pre = mu_Yn_est
        last_Sigma_Yn_pre = Sigma_yn_est
        last_error = error

        #運動方程式
        rand = np.random.normal(0,stdv_y,(1,1))
        noise = np.matrix([[rand[0,0]],[0]])
        y = (mat_1 * y)  + (mat_3*u) + noise
        yn_list.append(y[1,0])
        vn_list.append(y[0,0])

        #カルマンフィルタ
        mu_Yn_pre = (system_mat * last_mu_Yn_pre) + (control_mat * u)
        Sigma_yn_pre = (system_mat * last_Sigma_Yn_pre * system_mat.T) + Q_k
        K = Sigma_yn_pre * observation_mat.T * 1 / ((observation_mat * Sigma_yn_pre * observation_mat.T ) + R_k)
        
        rand = np.random.normal(0,stdv_R,(1,1))
        yn = y[1,0] + rand
        observation_list.append(yn)
        L = np.matrix([[y[0,0]],[yn]])
        Z = observation_mat * L
        mu_Yn_est = mu_Yn_pre + K * (Z - observation_mat * mu_Yn_pre)
        Sigma_yn_est = (unit_mat - (K * observation_mat)) * Sigma_yn_pre
        
        mu_v_list.append(mu_Yn_est[0,0][0,0])
        mu_y_list.append(mu_Yn_est[1,0][0,0])

        #PID
        error = r - mu_Yn_est[1,0]
        de = (error - last_error)/Control_T
        ie = ie + ((error + last_error)*(Control_T/2))
        u_n = Kp*error + Ki*ie + Kd*de
        u = u_n[0,0]

        u_list.append(u)
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
#ax1.plot(T_m,vn_list,label = "True velocity")
ax1.plot(T,vn_list,label = "True velocity")
ax1.grid()
ax1.set_xlabel("time")
ax1.set_ylabel("V[m/s]")

ax2.plot(T,mu_y_list,label = "estimate location")
#ax2.plot(T_m,yn_list,label = "True location")
ax2.plot(T,yn_list,label = "True location")
ax2.grid()
ax2.set_xlabel("time")
ax2.set_ylabel("location[m]")

# ax3.plot(T_m,observation_list,label = "Observedata")
# #ax3.plot(T_m,observation_list,label = "Observedata")
# ax3.plot(T,observation_list,label = "Observedata")
# ax3.grid()
# ax3.set_xlabel("time")
# ax3.set_ylabel("location[m]")

ax4.plot(T,u_list,label = "Input")
ax4.grid()
ax4.set_xlabel("time")
ax4.set_ylabel("Input")

ax5.plot(T,Sigma_yn_list_1,label = "estimate sigma")
# ax5.set_xlim(0,1)
ax5.grid()
ax5.set_xlabel("time")
ax5.set_ylabel("sigma")

ax6.plot(T,Sigma_yn_list_2,label = "estimate sigma")
# ax6.set_xlim(0,1)
ax6.grid()
ax6.set_xlabel("time")
ax6.set_ylabel("sigma")

ax7.plot(T,Sigma_yn_list_3,label = "estimate sigma")
# ax7.set_xlim(0,1)
ax7.grid()
ax7.set_xlabel("time")
ax7.set_ylabel("sigma")

ax8.plot(T,Sigma_yn_list_4,label = "estimate sigma")
# ax8.set_xlim(0,1)
ax8.grid()
ax8.set_xlabel("time")
ax8.set_ylabel("sigma")

ax1.legend()
ax2.legend()
ax3.legend()
# ax4.legend()
# plt.subplots_adjust(wspace=0.4,hspace=0.6)
plt.show()