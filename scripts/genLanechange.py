import json
import numpy as np
from matplotlib import pyplot as plt


laneleft_x = []
laneleft_y = []

lanemiddle_X = []
lanemiddle_Y = []

laneright_x = []
laneright_y = []

cone_x = []
cone_y = []

v1 = []
for i in np.arange(0, 20, 0.2):
    v1.append(i)

for i in v1:
    laneleft_x.append(i)
    laneright_x.append(i)
    lanemiddle_X.append(i)
    laneleft_y.append(12.5)
    lanemiddle_Y.append(8.75)
    laneright_y.append(5)

xy1 = np.array([13, 6])
xy2 = np.array([13, 7])
xy3 = np.array([15, 6])
xy4 = np.array([15, 7])

for i in range(1,5):
    exec('cone_x.append( float(xy%s[0]))'%i)
    exec('cone_y.append(float(xy%s[1]))'%i)


plt.plot(laneleft_x, laneleft_y)
plt.plot(lanemiddle_X, lanemiddle_Y)
plt.plot(laneright_x, laneright_y)
plt.scatter(cone_x,cone_y)




plt.axis("equal")
plt.show()


lanechange_dict = {"Lanechange": [ {"laneleft_X":laneleft_x}, {"laneleft_Y":laneleft_y}, {"lanemiddle_X":lanemiddle_X}, {"lanemiddle_Y":lanemiddle_Y}, {"laneright_X":laneright_x}, {"laneright_Y": laneright_y}, {"cone_X":cone_x}, {"cone_Y":cone_y}]}


with open("lanechange.json", "w") as f:
    json.dump(lanechange_dict,f, indent=2, sort_keys=True)