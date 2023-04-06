import numpy as np
import matplotlib.pyplot as plt

#重量
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
#時間
t = 10
#刻み幅
h = 0.001
#要素数
n = int((t/h))
#制御入力
omega_o = np.sqrt(m * g / Ct)
omega = 480
delta_omega = np.sqrt(omega**2 - omega_o**2)
# delta_T = Ct *(omega**2 - omega_o**2)
delta_T = 0
#システムノイズのばらつき（分散）
stdv_y = 0.0009
Sigma_y = stdv_y **2

T_m = np.linspace(0,t,n)

constant = np.matrix([[-g*h],[0]])
mat_1 = np.matrix([[1,0],[h,1]])
y = np.matrix([[0],[0]])
mat_3 = np.matrix([[h/m],[0]])

T = np.linspace(0,t,n)

for i in range(n) :
    
    rand = np.random.normal(0,stdv_y,(1,1))
    noise = np.matrix([[rand[0,0]],[0]])
    y = (mat_1 * y)  + (mat_3*delta_T) + noise
    yn_list.append(y[1,0])
    
    
    vn_list.append(y[0,0])
# plt.plot(T,yn_list,label = "final")
# plt.grid()
# plt.xlabel('Time')
# plt.ylabel('location')
# plt.legend()
# plt.show()