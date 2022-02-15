import json
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.patches as mpathes


################
#直线整齐排列工况#
###############
v1 = []
for i in np.arange(2, 15, 2):
    v1.append( float(i))

direct_x_left = []
direct_y_left = []
direct_x_right = []
direct_y_right = []

for i in v1:
    direct_x_left.append(i)
    direct_y_left.append(5)
    
    direct_x_right.append(i)
    direct_y_right.append(8.75)

x1_L = np.array([17.5, 8.73])
x2_L = np.array([19.97, 8.81])
x3_L = np.array([22.52, 8.79])
x4_L = np.array([24.38, 8.75])
x5_L = np.array([27.18, 8.96])
x6_L = np.array([29.56, 8.75])


x1_R = np.array([17.2, 5.02])
x2_R = np.array([20.23, 4.95])
x3_R = np.array([22.58, 5.10])
x4_R = np.array([24.14, 5.02])
x5_R = np.array([26.5, 4.98])
x6_R = np.array([28.3, 5.02])
x7_R = np.array([32.0, 5])


for i in range(1,7):
    exec('direct_x_left.append(float(x%s_L[0]))'%i)
    exec('direct_y_left.append(float(x%s_L[1]))'%i)

for i in range(1,8):
    exec('direct_x_right.append(float(x%s_R[0]))'%i)
    exec('direct_y_right.append(float(x%s_R[1]))'%i)



################
#    弯道工况   # 
###############

curve_x_left =[]
curve_Y_left =[]
curve_x_right =[]
curve_Y_right =[]

# v2 = []

# for i in np.arange(5, 25,2):
#     v2.append(i)

# for i in v2:
#     curve_x_left.append(i)
#     circle_y_R = np.sqrt(10**2+2**2 -(i-10)**2) +2
#     curve_Y_left.append(circle_y_R)

# v3 = []
# for i in np.arange(5, 20, 2):
#     v3.append(i)

# for i in v3:
#     curve_x_right.append(i)
#     circle_y_L = np.sqrt(6.85**2+2**2 -(i-10)**2) +2
#     curve_Y_right.append(circle_y_L)

x_L1 = np.array([4.95,10.92])
x_L2 = np.array([6.79,11.69])
x_L3 = np.array([9.02,12.18])
x_L4 = np.array([10.98,12.16])
x_L5 = np.array([12.96,11.73])
x_L6 = np.array([14.97,10.91])
x_L7 = np.array([16.53,9.75])
x_L8 = np.array([18,8.25])
x_L9 = np.array([19,6.89])

x_R1 = np.array([5.04, 7.24])
x_R2 = np.array([6.92, 8.41])
x_R3 = np.array([9, 9.06])
x_R4 = np.array([11.36, 8.97])
x_R5 = np.array([12.92, 8.50])
x_R6 = np.array([14.70, 7.30])
x_R7 = np.array([16, 5.62])
x_R8 = np.array([17, 4.35])

for i in range(1,9):
    exec('curve_x_left.append( float(x_L%s[0]) )'%i)
    exec('curve_Y_left.append(float(x_L%s[1]))'%i)
    exec('curve_x_right.append(float(x_R%s[0]))'%i)
    exec('curve_Y_right.append(float(x_R%s[1]))'%i)




plt.figure(1)
plt.scatter(direct_x_left,direct_y_left, color = "r")
plt.scatter(direct_x_right,direct_y_right, color = "r")
plt.axis("equal")
plt.show()

plt.figure(2)
plt.scatter(curve_x_left, curve_Y_left, color = "r")
plt.scatter(curve_x_right, curve_Y_right, color = "r")

plt.axis("equal")
plt.show()


cone_dict = {"cone": [{"direct_x_L":direct_x_left}, {"direct_y_L":direct_y_left}, {"direct_x_R":direct_x_right}, {"direct_y_R":direct_y_right},
                            {"curve_x_L":curve_x_left}, {"curve_y_L":curve_Y_left}, {"curve_x_R":curve_x_right}, {"curve_y_R":curve_Y_right}
            ]}

with open("./cone_direct.json", "w") as f:
    json.dump(cone_dict,f, indent=2, sort_keys=True)

