import pprint
import numpy as np
import matplotlib.pyplot as plt
import math

#重量
m = 5
#ばね定数
k = 10
#ダンピング係数
c = 1
#速度
vn_list = []
#位置
xn_list = []
#制御入力
un = 0
#時間
t = 10
#刻み幅
h = 0.001
#要素数
n = int((t/h) + 1)


#システムノイズのばらつき（分散）
stdv_x = 15
Sigma_x = stdv_x **2

T_s = np.linspace(0,t,n)

mat_1 = np.matrix([[1-(c/m*h) , (-k/m)*h],[h,1]])
x = np.matrix([[0],[0]])
mat_3 = np.matrix([[h/m],[0]])

for i in range(n):
    if i > t*10:
        rand = np.random.normal(0,stdv_x,(1,1))
        un = 1 + rand[0,0]

    x = (mat_1 * x) + (mat_3)*un
    xn_list.append(x[1,0])
    vn_list.append(x[0,0])

# plt.plot(T_s,xn_list,label = "true location")
# plt.grid()
# plt.xlabel('Time')
# plt.ylabel('location')
# plt.legend()
# plt.show()
    
