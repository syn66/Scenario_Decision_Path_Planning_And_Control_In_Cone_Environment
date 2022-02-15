#coding=utf-8

import json
import numpy as np
from matplotlib import pyplot as plt

with open('/home/syn/毕业设计/路径规划/project/roadmap/direct_roadmap.json', "r") as f:
    data = json.load(f)

plt.plot(data['LaneAndConeCoordinate'][0]['X_l'], data['LaneAndConeCoordinate'][1]['Y_l'])
plt.plot(data['LaneAndConeCoordinate'][2]['X_m'], data['LaneAndConeCoordinate'][3]['Y_m'])
plt.plot(data['LaneAndConeCoordinate'][4]['X_r'], data['LaneAndConeCoordinate'][5]['Y_r'])
plt.plot(data['LaneAndConeCoordinate'][6]['X_c'], data['LaneAndConeCoordinate'][7]['Y_c'])

plt.axis("equal")
plt.show()

