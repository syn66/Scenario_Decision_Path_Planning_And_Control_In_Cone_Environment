import json 
import numpy as np
from matplotlib import pyplot as plt

v_y1 = []
for i in np.arange(0,30, 0.5):
    v_y1.append(i)


direct_x_L = []
direct_y_L = []

direct_scatter_X_R = []
direct_scatter_Y_R = []


for i in v_y1:
    direct_x_L.append(5)
    direct_y_L.append(i)


v1 = []
for i in np.arange(0,30,2.5):
    v1.append(i)

for i in v1:
    direct_scatter_X_R.append(8.75)
    direct_scatter_Y_R.append(i)



value_X_2 = []
for i in np.arange(12, 28, 0.2):
    value_X_2.append(i)

cureve_x_L = []
cureve_y_L = []
cureve_x_R = []
cureve_y_R = []

for i in value_X_2:
    cureve_x_L.append(i)
    circle_y_L = np.sqrt(38**2+10**2 -(i-50)**2) - 10
    cureve_y_L.append(circle_y_L)

valueX_3 =[]
for i in np.arange(15.75, 30, 2.5):
    valueX_3.append(i)

for i in valueX_3:
    cureve_x_R.append(i)
    circle_y_R = np.sqrt(34.25**2+10**2 -(i-50)**2) - 10
    cureve_y_R.append(circle_y_R)




plt.plot(direct_x_L, direct_y_L)
plt.scatter(direct_scatter_X_R,direct_scatter_Y_R)
plt.plot(cureve_x_L,cureve_y_L)
plt.scatter(cureve_x_R,cureve_y_R)
plt.axis("equal")
plt.show()

dict_lanekeep = {"LaneKeepScene": [ {"X_Direct_L":direct_x_L}, {"Y_Direct_L":direct_y_L},
                                    {"X_Direct_cone_R":direct_scatter_X_R}, {"Y_Direct_cone_R":direct_scatter_Y_R},
                                    {"X_Curve_L":cureve_x_L}, {"Y_Curve_L":cureve_y_L},
                                    {"X_Curve_cone_R":cureve_x_R}, {"Y_Curve_cone_R":cureve_y_R},                                    
                                    ]}

with open("./ConeLanekeepscene.json", "w") as f:
    json.dump(dict_lanekeep, f, indent=2, sort_keys= True)
