import json
import numpy as np
from matplotlib import pyplot as plt
from numpy.core.fromnumeric import sort

value_Y_1 = []
for i in np.arange(0, 30, 0.5):
    value_Y_1.append(i)

direct_x_L = []
direct_y_L = []

direct_x_R = []
direct_y_R = []

for i in value_Y_1:
    direct_x_L.append(5)
    direct_x_R.append(8.75)
    direct_y_L.append(i)
    direct_y_R.append(i)


value_X_2 = []
for i in np.arange(12, 30, 0.2):
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
for i in np.arange(15.75,30, 0.2):
    valueX_3.append(i)

for i in valueX_3:
    cureve_x_R.append(i)
    circle_y_R = np.sqrt(34.25**2+10**2 -(i-50)**2) - 10
    cureve_y_R.append(circle_y_R)

plt.plot(direct_x_L, direct_y_L)
plt.plot(direct_x_R, direct_y_R)
plt.plot(cureve_x_L, cureve_y_L)
plt.plot(cureve_x_R, cureve_y_R)
plt.axis("equal")
plt.show()

dict_lanekeep = {"LaneKeepScene": [ {"X_Direct_L":direct_x_L}, {"Y_Direct_L":direct_y_L},
                                    {"X_Direct_R":direct_x_R}, {"Y_Direct_R":direct_y_R},
                                    {"X_Curve_L":cureve_x_L}, {"Y_Curve_L":cureve_y_L},
                                    {"X_Curve_R":cureve_x_R}, {"Y_Curve_R":cureve_y_R},                                    
                                    ]}

with open("./Lanekeepscene.json", "w") as f:
    json.dump(dict_lanekeep, f, indent=2, sort_keys= True)