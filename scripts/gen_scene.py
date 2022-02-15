#coding=utf-8
import json
# from types import TracebackType
# from types import LambdaType
from matplotlib import colors, pyplot as plt
import numpy as np
import matplotlib.patches as mpathes
from numpy.lib import index_tricks

laneleft_X = []
laneleft_Y = []

lanemiddle_x =[]
lanemiddle_y = []

laneright_x = []
laneright_y =[]

track_x =[]
track_y = []

# 第一段路
a = []
for i in np.arange(15,55,0.3):
    a.append(i)
    a.sort(reverse=True)

# print(a)
for i in a:
    laneleft_Y.append(5)
    laneleft_X.append(i)


laneleft_X.sort(reverse=True)
laneleft_Y.sort(reverse=True)

# zhongjian 
b = []
for i in np.arange(15,55,0.3):
    b.append(i)
    b.sort(reverse=True)


for i in b:
    lanemiddle_y.append(8.75)
    lanemiddle_x.append(i)


lanemiddle_x.sort(reverse=True)
lanemiddle_y.sort(reverse=True)

# youbian
c = []
for i in np.arange(15,55,0.3):
    c.append(i)
    c.sort(reverse=True)


for i in c:
    laneright_y.append(12.5)
    laneright_x.append(i)

laneright_x.sort(reverse=True)
laneright_y.sort(reverse=True)

#track

d = []
for i in np.arange(15,55,0.3):
    d.append(i)
    d.sort(reverse=True)

for i in d:
    track_x.append(i)
    track_y.append(6.875)



# 第二段路
value = []
for i in np.arange(5, 15, 0.3):
    value.append(i)
    value.sort(reverse=True)

def trans(x1, y1, y):
    dist = y1-y
    return x1, y-dist

# print(value)
for i in value:
    circle_X_2_m = i;
    # print(i)
    
    circle_y_2_m = np.sqrt(10*10 -(circle_X_2_m-15)**2) + 15

    x_tran_m, y_tran_m = trans(circle_X_2_m, circle_y_2_m, 15)
    laneleft_X.append(x_tran_m)
    laneleft_Y.append(y_tran_m)

#zhongjian1
value_M = []
for i in np.arange(8.75, 15, 0.3):
    value_M.append(i)
    value_M.sort(reverse=True)


for i in value_M:
    circle_X_2 = i;
    # print(i)
    
    circle_y_2 = np.sqrt(6.25*6.25 -(circle_X_2-15)**2) + 15

    x_tran, y_tran = trans(circle_X_2, circle_y_2, 15)
    lanemiddle_x.append(x_tran)

    lanemiddle_y.append(y_tran)

#youbian
value_R = []
for i in np.arange(12.5, 15, 0.3):
    value_R.append(i)
    value_R.sort(reverse=True)

for i in value_R:
    circle_X_2 = i;
    # print(i)
    
    circle_y_2 = np.sqrt(2.5*2.5 -(circle_X_2-15)**2) +15

    x_tran, y_tran = trans(circle_X_2, circle_y_2, 15)
    laneright_x.append(x_tran)

    laneright_y.append(y_tran)

#track
value_t = []
for i in np.arange(6.875, 15, 0.3):
    value_t.append(i)
    value_t.sort(reverse=True)

for i in value_t:
    circle_X_2 = i;
    # print(i)
    
    circle_y_2 = np.sqrt(8.125*8.125 -(circle_X_2-15)**2) +15

    x_tran, y_tran = trans(circle_X_2, circle_y_2, 15)
    track_x.append(x_tran)

    track_y.append(y_tran)


# 左侧车道第3段路 y
for i in np.arange(15,75,0.3):
    laneleft_Y.append(i)

    laneleft_X.append(5)

# zhongjian1
for i in np.arange(15,75,0.3):
    lanemiddle_y.append(i)

    lanemiddle_x.append(8.75)

#右边
for i in np.arange(15,75,0.3):
    laneright_y.append(i)

    laneright_x.append(12.5)

#track
for i in np.arange(15,75,0.3):
    track_y.append(i)

    track_x.append(6.875)


# 第4段路 圆弧 x
for i in np.arange(5, 15, 0.3):
    circle_X = i;
    # print(i)

    circle_y = np.sqrt(10*10 -(circle_X-15)**2) +75
    laneleft_X.append(circle_X)
    laneleft_Y.append(circle_y)

#zhingjian
for i in np.arange(8.75, 15, 0.3):
    circle_X_m = i;
    # print(i)

    circle_y_m = np.sqrt(6.25*6.25 -(circle_X_m-15)**2) +75
    lanemiddle_x.append(circle_X_m)
    lanemiddle_y.append(circle_y_m)

    #youbian
for i in np.arange(12.5, 15, 0.3):
    circle_X_m = i;
    # print(i)

    circle_y_m = np.sqrt(2.5*2.5 -(circle_X_m-15)**2) +75
    laneright_x.append(circle_X_m)
    laneright_y.append(circle_y_m) 

#track
for i in np.arange(6.875, 15, 0.3):
    circle_X_m = i
    # print(i)

    circle_y_m = np.sqrt(8.125*8.125 -(circle_X_m-15)**2) +75
    track_x.append(circle_X_m)
    track_y.append(circle_y_m) 

#  第\5段路 x
for i in np.arange(15,45,0.3):
    laneleft_X.append(i)
    laneleft_Y.append(85)

    #zhingjian
for i in np.arange(15,45,0.3):
    lanemiddle_x.append(i)

    lanemiddle_y.append(81.25)

for i in np.arange(15,45,0.3):
    laneright_x.append(i)

    laneright_y.append(77.5)

    #track
for i in np.arange(15,45,0.3):
    track_x.append(i)

    track_y.append(83.125)


#第6段路 圆弧

for i in np.arange(45, 85, 0.3):
    circle_X_1 = i
    circle_y_1 = np.sqrt((10**2+50**2)-(circle_X_1-35)**2) +35
    laneleft_X.append(circle_X_1)
    laneleft_Y.append(circle_y_1)

    #zhongjian
for i in np.arange(45, 81.25, 0.3):
    circle_X_1 = i
    circle_y_1 = np.sqrt((10**2+46.25**2)-(circle_X_1-35)**2) +35
    lanemiddle_x.append(circle_X_1)
    lanemiddle_y.append(circle_y_1)

for i in np.arange(45,77.5 , 0.3):
    circle_X_1 = i
    circle_y_1 = np.sqrt((10**2+42.5**2)-(circle_X_1-35)**2) + 35
    laneright_x.append(circle_X_1)
    laneright_y.append(circle_y_1)

#track
for i in np.arange(45,83.125 , 0.3):
    circle_X_1 = i
    circle_y_1 = np.sqrt((10**2+48.125**2)-(circle_X_1-35)**2) + 35
    track_x.append(circle_X_1)
    track_y.append(circle_y_1)

# 第七段
value1 = []
for i in np.arange(29, 45, 0.3):
    value1.append(i)

value1.sort(reverse=True)

for i in value1:
    laneleft_X.append(85)
    laneleft_Y.append(i)

#zhong jian

value2 = []
for i in np.arange(29, 45, 0.3):
    value2.append(i)

value2.sort(reverse=True)

for i in value2:
    lanemiddle_x.append(81.25)
    lanemiddle_y.append(i)

value3 = []
for i in np.arange(29, 45, 0.3):
    value3.append(i)

value3.sort(reverse=True)

for i in value3:
    laneright_x.append(77.5)
    laneright_y.append(i)

#track

value4 = []
for i in np.arange(29, 45, 0.3):
    value4.append(i)

value4.sort(reverse=True)

for i in value4:
    track_x.append(83.125)
    track_y.append(i)

# zhuitong zuobiao
cone_x = []
cone_y = []





# obstacle
xy1 = np.array([5.8, 39])
xy2 = np.array([5.8, 41])
xy3 = np.array([7.5, 39])
xy4 = np.array([7.5, 41])


xy5 = np.array([27, 78])
xy6 = np.array([28, 79])
xy7 = np.array([29, 80])
xy8 = np.array([30, 81])

xy9 = np.array([32.5, 81])
xy10 = np.array([35, 81.25])
xy11 = np.array([37.5, 81.5])
xy12 = np.array([40, 81.25])
xy13 = np.array([42.5, 81])
xy14 = np.array([45, 81.25])
xy15 = np.array([47.5, 80.5])
xy16 = np.array([50, 80])
xy17 = np.array([52.5, 79])
xy18 = np.array([55, 78])
xy19 = np.array([57.5, 76.5])
xy20 = np.array([60, 75.3])
xy21 = np.array([62.5, 73.4])
xy22 = np.array([65, 71.3])
xy23 = np.array([67.5, 69.2])
xy24 = np.array([70, 66.8])
xy25 = np.array([72.5, 64.1])


xy26 = np.array([81.2, 28.4])
xy27 = np.array([80, 26.4])
xy28 = np.array([85.1, 27.5])
xy29 = np.array([79.1, 24.2])
xy30 = np.array([83.74, 24.3])
xy31 = np.array([77.6, 21.5])

xy32 = np.array([81.9, 20.9])
xy33 = np.array([75.7, 18.9])
xy34 = np.array([79.7, 17.7])
xy35 = np.array([73.5, 16.4])
xy36 = np.array([78.2, 15.7])

xy37 = np.array([71.3, 14.3])
xy38 = np.array([74.8, 12.3])
xy39 = np.array([68.1, 12.2])
xy40 = np.array([71.6, 9.9])

xy41 = np.array([69.1, 8.51])

xy42 = np.array([65, 10.7])
xy43 = np.array([66.4, 7.19])
xy44 = np.array([62.5, 9.85])
xy45 = np.array([63, 6.07])

xy46 = np.array([59.3, 9.03])
xy47 = np.array([59.0, 5.23])
xy48 = np.array([56.2, 8.95])
xy49 = np.array([55.5, 4.96])








circle1 = mpathes.Circle(xy1,0.3,color= 'r')
circle2 = mpathes.Circle(xy2,0.3,color= 'r')
circle3 = mpathes.Circle(xy3,0.3,color= 'r')
circle4 = mpathes.Circle(xy4,0.3,color= 'r')
circle5 = mpathes.Circle(xy5,0.3,color= 'r')
circle6 = mpathes.Circle(xy6,0.3,color= 'r')
circle7 = mpathes.Circle(xy7,0.3,color= 'r')
circle8 = mpathes.Circle(xy8,0.3,color= 'r')
circle9 = mpathes.Circle(xy9,0.3,color= 'r')
circle10 = mpathes.Circle(xy10,0.3,color= 'r')
circle11 = mpathes.Circle(xy11,0.3,color= 'r')
circle12 = mpathes.Circle(xy12,0.3,color= 'r')
circle13 = mpathes.Circle(xy13,0.3,color= 'r')
circle14 = mpathes.Circle(xy14,0.3,color= 'r')
circle15 = mpathes.Circle(xy15,0.3,color= 'r')
circle16 = mpathes.Circle(xy16,0.3,color= 'r')
circle17 = mpathes.Circle(xy17,0.3,color= 'r')
circle18 = mpathes.Circle(xy18,0.3,color= 'r')
circle19 = mpathes.Circle(xy19,0.3,color= 'r')
circle20 = mpathes.Circle(xy20,0.3,color= 'r')
circle21 = mpathes.Circle(xy21,0.3,color= 'r')
circle22 = mpathes.Circle(xy22,0.3,color= 'r')
circle23 = mpathes.Circle(xy23,0.3,color= 'r')
circle24 = mpathes.Circle(xy24,0.3,color= 'r')
circle25 = mpathes.Circle(xy25,0.3,color= 'r')
circle26 = mpathes.Circle(xy26,0.3,color= 'r')
circle27 = mpathes.Circle(xy27,0.3,color= 'r')
circle28 = mpathes.Circle(xy28,0.3,color= 'r')
circle29 = mpathes.Circle(xy29,0.3,color= 'r')
circle30= mpathes.Circle(xy30,0.3,color= 'r')
circle31 = mpathes.Circle(xy31,0.3,color= 'r')
circle32 = mpathes.Circle(xy32,0.3,color= 'r')

circle33 = mpathes.Circle(xy33,0.3,color= 'r')
circle34 = mpathes.Circle(xy34,0.3,color= 'r')
circle35 = mpathes.Circle(xy35,0.3,color= 'r')
circle36 = mpathes.Circle(xy36,0.3,color= 'r')
circle37 = mpathes.Circle(xy37,0.3,color= 'r')
circle38 = mpathes.Circle(xy38,0.3,color= 'r')
circle39 = mpathes.Circle(xy39,0.3,color= 'r')
circle40 = mpathes.Circle(xy40,0.3,color= 'r')
circle41 = mpathes.Circle(xy41,0.3,color= 'r')
circle42 = mpathes.Circle(xy42,0.3,color= 'r')
circle43 = mpathes.Circle(xy43,0.3,color= 'r')
circle44 = mpathes.Circle(xy44,0.3,color= 'r')
circle45 = mpathes.Circle(xy45,0.3,color= 'r')
circle46 = mpathes.Circle(xy46,0.3,color= 'r')
circle47 = mpathes.Circle(xy47,0.3,color= 'r')
circle48 = mpathes.Circle(xy48,0.3,color= 'r')
circle49 = mpathes.Circle(xy49,0.3,color= 'r')
# circle50 = mpathes.Circle(xy50,0.3,color= 'r')
# circle51 = mpathes.Circle(xy51,3,color= 'r')
# circle52 = mpathes.Circle(xy52,3,color= 'r')
# circle53 = mpathes.Circle(xy53,3,color= 'r')
# circle54 = mpathes.Circle(xy54,3,color= 'r')
# circle55 = mpathes.Circle(xy55,3,color= 'r')
# circle56 = mpathes.Circle(xy56,3,color= 'r')

# circle57 = mpathes.Circle(xy57,3,color= 'r')
# circle58 = mpathes.Circle(xy58,3,color= 'r')
# circle59 = mpathes.Circle(xy59,3,color= 'r')
# circle60 = mpathes.Circle(xy60,3,color= 'r')
# circle61 = mpathes.Circle(xy61,3,color= 'r')
# circle62 = mpathes.Circle(xy62,3,color= 'r')
# circle63 = mpathes.Circle(xy63,3,color= 'r')
# circle64 = mpathes.Circle(xy64,3,color= 'r')
# circle65= mpathes.Circle(xy65,3,color= 'r')
# circle66 = mpathes.Circle(xy66,3,color= 'r')
# circle67 = mpathes.Circle(xy67,3,color= 'r')
# circle68 = mpathes.Circle(xy68,3,color= 'r')
# circle69 = mpathes.Circle(xy69,3,color= 'r')
# circle70 = mpathes.Circle(xy70,3,color= 'r')
# circle71 = mpathes.Circle(xy71,3,color= 'r')
# circle72 = mpathes.Circle(xy72,3,color= 'r')
# circle73 = mpathes.Circle(xy73,3,color= 'r')
# circle74 = mpathes.Circle(xy74,3,color= 'r')
# circle75 = mpathes.Circle(xy75,3,color= 'r')
# circle76 = mpathes.Circle(xy76,3,color= 'r')
# circle77 = mpathes.Circle(xy77,3,color= 'r')
# circle78 = mpathes.Circle(xy78,3,color= 'r')
# circle79 = mpathes.Circle(xy79,3,color= 'r')

# circle80 = mpathes.Circle(xy80,3,color= 'r')
# circle81 = mpathes.Circle(xy81,3,color= 'r')
# circle82 = mpathes.Circle(xy82,3,color= 'r')
# circle83 = mpathes.Circle(xy83,3,color= 'r')
# circle84 = mpathes.Circle(xy84,3,color= 'r')
# circle85 = mpathes.Circle(xy85,3,color= 'r')
# circle86 = mpathes.Circle(xy86,3,color= 'r')
# circle87 = mpathes.Circle(xy87,3,color= 'r')
# circle88 = mpathes.Circle(xy88,3,color= 'r')
# circle89 = mpathes.Circle(xy89,3,color= 'r')
# circle90 = mpathes.Circle(xy90,3,color= 'r')
# circle91 = mpathes.Circle(xy91,3,color= 'r')
# circle92 = mpathes.Circle(xy92,3,color= 'r')

# circle93 = mpathes.Circle(xy93,3,color= 'r')
# circle94 = mpathes.Circle(xy94,3,color= 'r')
# circle95 = mpathes.Circle(xy95,3,color= 'r')
# circle96 = mpathes.Circle(xy96,3,color= 'r')
# circle97 = mpathes.Circle(xy97,3,color= 'r')
# circle98 = mpathes.Circle(xy98,3,color= 'r')
# circle99 = mpathes.Circle(xy99,3,color= 'r')
# circle100 = mpathes.Circle(xy100,3,color= 'r')

# circle101 = mpathes.Circle(xy101,3,color= 'r')
# circle102 = mpathes.Circle(xy102,3,color= 'r')
# circle103 = mpathes.Circle(xy103,3,color= 'r')
# circle104 = mpathes.Circle(xy104,3,color= 'r')
# # circle105 = mpathes.Circle(xy105,3,color= 'r')




fig,ax = plt.subplots()





ax.add_patch(circle1)
ax.add_patch(circle2)
ax.add_patch(circle3)
ax.add_patch(circle4)
ax.add_patch(circle5)
ax.add_patch(circle6)
ax.add_patch(circle7)
ax.add_patch(circle8)
ax.add_patch(circle9)
ax.add_patch(circle10)
ax.add_patch(circle11)
ax.add_patch(circle12)
ax.add_patch(circle13)
ax.add_patch(circle14)
ax.add_patch(circle15)
ax.add_patch(circle16)
ax.add_patch(circle17)
ax.add_patch(circle18)
ax.add_patch(circle19)
ax.add_patch(circle20)
ax.add_patch(circle21)
ax.add_patch(circle22)
ax.add_patch(circle23)
ax.add_patch(circle24)
ax.add_patch(circle25)
ax.add_patch(circle26)
ax.add_patch(circle27)
ax.add_patch(circle28)
ax.add_patch(circle29)
ax.add_patch(circle30)
ax.add_patch(circle31)
ax.add_patch(circle32)

ax.add_patch(circle33)
ax.add_patch(circle34)
ax.add_patch(circle35)
ax.add_patch(circle36)
ax.add_patch(circle37)
ax.add_patch(circle38)
ax.add_patch(circle39)
ax.add_patch(circle40)
ax.add_patch(circle41)
ax.add_patch(circle42)
ax.add_patch(circle43)
ax.add_patch(circle44)
ax.add_patch(circle45)
ax.add_patch(circle46)
ax.add_patch(circle47)
ax.add_patch(circle48)
ax.add_patch(circle49)
# ax.add_patch(circle50)
# ax.add_patch(circle51)
# ax.add_patch(circle52)
# ax.add_patch(circle53)
# ax.add_patch(circle54)
# ax.add_patch(circle55)
# ax.add_patch(circle56)

# ax.add_patch(circle57)
# ax.add_patch(circle58)
# ax.add_patch(circle59)
# ax.add_patch(circle60)
# ax.add_patch(circle61)
# ax.add_patch(circle62)
# ax.add_patch(circle63)
# ax.add_patch(circle64)
# ax.add_patch(circle65)
# ax.add_patch(circle66)
# ax.add_patch(circle67)
# ax.add_patch(circle68)
# ax.add_patch(circle69)
# ax.add_patch(circle70)
# ax.add_patch(circle71)
# ax.add_patch(circle72)
# ax.add_patch(circle73)
# ax.add_patch(circle74)
# ax.add_patch(circle75)
# ax.add_patch(circle76)
# ax.add_patch(circle77)
# ax.add_patch(circle78)
# ax.add_patch(circle79)
# ax.add_patch(circle80)

# ax.add_patch(circle81)
# ax.add_patch(circle82)
# ax.add_patch(circle83)
# ax.add_patch(circle84)
# ax.add_patch(circle85)
# ax.add_patch(circle86)
# ax.add_patch(circle87)
# ax.add_patch(circle88)
# ax.add_patch(circle89)
# ax.add_patch(circle90)
# ax.add_patch(circle91)
# ax.add_patch(circle92)

# ax.add_patch(circle93)
# ax.add_patch(circle94)
# ax.add_patch(circle95)
# ax.add_patch(circle96)
# ax.add_patch(circle97)

# ax.add_patch(circle98)
# ax.add_patch(circle99)
# ax.add_patch(circle100)

# ax.add_patch(circle101)
# ax.add_patch(circle102)
# ax.add_patch(circle103)
# ax.add_patch(circle104)
# # ax.add_patch(circle105)

# # ax.add_patch(circle97)

for i in range(1,50):
    exec('cone_x.append( float(xy%s[0]) )'%i)
    exec('cone_y.append( float(xy%s[1]) )'%i)
# print(cone_x)







ax.scatter(laneleft_X,laneleft_Y, s=0.5)
ax.scatter(lanemiddle_x, lanemiddle_y,linestyle='--', color = 'g', linewidth=1, s=0.5)
ax.scatter(laneright_x, laneright_y, s=0.5)
ax.scatter(track_x,track_y,s=0.5)
# ax.scatter(cone_x, cone_y, s=0.5)



# ax.scatter()
plt.axis('equal')
plt.show()


laneleft_dict = {'LaneLeft':[{'X_l':laneleft_X}, {'Y_l':laneleft_Y}]}
lanemiddle_dict = {'LaneMiddle':[{'X_m':lanemiddle_x}, {'Y_m':lanemiddle_y}]}
laneright_dict = {'LaneRight':[{'X_r':laneright_x}, {'Y_r':laneright_y}]}



# cone_dict = {'Cone':[{'X_c':cone_x}, {'Y_c':cone_y}]}

# road_map_dict = {[ {'Lane':[laneleft_dict, lanemiddle_dict, laneright_dict] }, {'Cone': [{'X_c':cone_x}, {'Y_c':cone_y}]} ]}

road_map_dict = {'LaneAndConeCoordinate' : [ {'X_l':laneleft_X}, {'Y_l':laneleft_Y}, {'X_m':lanemiddle_x}, {'Y_m':lanemiddle_y}, {'X_r':laneright_x}, {'Y_r':laneright_y}, {'X_c':cone_x}, {'Y_c':cone_y}] }

track = {'track' : [ {'X':track_x}, {'Y':track_y}] }


with open("./road_map.json","w") as f:
    json.dump(road_map_dict,f, indent=2, sort_keys=True)

with open("./track.json","w") as f:
    json.dump(track,f, indent=2, sort_keys=True)


# 直道场景
Lane_coor_L_x = []
Lane_coor_L_y = []

Lane_coor_M_x = []
Lane_coor_M_y = []

Lane_coor_R_x = []
Lane_coor_R_y = []

coor_coor_x = []

coor_coor_y = []
coor_coor_y1 = []

direct_track_x = []
direct_track_y = []


v1=[]
for i in np.arange(0,160, 0.5):
    v1.append(i)

for i in v1:
    Lane_coor_L_x.append(i)
    Lane_coor_L_y.append(12.5)

    Lane_coor_M_x.append(i)
    Lane_coor_M_y.append(8.75)

    Lane_coor_R_x.append(i)
    Lane_coor_R_y.append(5)

v2 = []
for i in np.arange(0,35, 0.5):
    v2.append(i)

for i in v2:
    direct_track_x.append(i)
    direct_track_y.append(10.625)

v3 = []
for i in np.arange(35, 45, 0.5):
    v3.append(i)

a = 10.625
for i in v3:

    direct_track_x.append(i)

    a -= 0.1875
    direct_track_y.append(a)


v4 = []
for i in np.arange(45, 95,0.5):
    v4.append(i)

for i in v4:
    direct_track_x.append(i)
    direct_track_y.append(6.875)

v5 = []
b = 6.875
for i in np.arange(95, 105,0.5):
    v5.append(i)

for i in v5:
    direct_track_x.append(i)
    b+=0.1875
    direct_track_y.append(b)

v6 = []
for i in np.arange(105, 215,0.5):
    v6.append(i)

for i in  v6:
    direct_track_x.append(i)
    direct_track_y.append(10.625)


v10 = []
for i in np.arange(160,220, 2.5):
    v2.append(i)

for i in v10:
    coor_coor_x.append(i)
    coor_coor_y.append(8.75)
    coor_coor_y1.append(12.5)




xy_c1 = np.array([39,11])
xy_c2 = np.array([39,9.5])
xy_c3 = np.array([41,11])
xy_c4 = np.array([41,9.5])

xy_c5 = np.array([100,5.5])
xy_c6 = np.array([101,6.5])
xy_c7 = np.array([102,7.5])
xy_c8 = np.array([103,8.5])

xy_c9 = np.array([105,8.75])
xy_c10 = np.array([107.5,9])
xy_c11 = np.array([110,8.75])
xy_c12 = np.array([112.5,8.5])
xy_c13 = np.array([115,8.75])
xy_c14 = np.array([117.5,9])
xy_c15 = np.array([120,8.75])
xy_c16 = np.array([122.5,8.5])
xy_c17 = np.array([125,8.75])
xy_c18 = np.array([127.5,9])
xy_c19 = np.array([130,8.75])
xy_c20 = np.array([132.5,9])
xy_c21 = np.array([135,8.75])
xy_c22 = np.array([137.5,8.5])
xy_c23 = np.array([140,8.75])
xy_c24 = np.array([142.5,9])
xy_c25 = np.array([145,8.75])
xy_c26 = np.array([147.5,8.5])
xy_c27 = np.array([150,8.75])


xy_c28 = np.array([162,12.5])
xy_c29 = np.array([162.5,8.75])
xy_c30 = np.array([164.5,12.5])
xy_c31 = np.array([164,8.75])
xy_c32 = np.array([167,12.5])
xy_c33 = np.array([167,8.75])
xy_c34 = np.array([170,12.5])
xy_c35 = np.array([170,8.75])
xy_c36 = np.array([172.5,12.5])
xy_c37 = np.array([172,8.75])
xy_c38 = np.array([175,12.5])
xy_c39 = np.array([174.5,8.75])
xy_c40 = np.array([177.5,12.5])

xy_c41 = np.array([176.5,8.75])
xy_c42 = np.array([180,12.5])
xy_c43 = np.array([179.5,8.75])
xy_c44 = np.array([182.5,12.5])
xy_c45 = np.array([182,8.75])
xy_c46 = np.array([185,12.5])
xy_c47 = np.array([184.5,8.75])
xy_c48 = np.array([187.5,12.5])
xy_c49 = np.array([187,8.75])

xy_c50 = np.array([190,12.5])
xy_c51 = np.array([189.5,8.75])
xy_c52 = np.array([192.5,12.5])
xy_c53 = np.array([192,8.75])
xy_c54 = np.array([195,12.5])
xy_c55 = np.array([194.5,8.75])
xy_c56 = np.array([197.5,12.5])
xy_c57 = np.array([197,8.75])
xy_c58 = np.array([200,12.5])
xy_c59 = np.array([199.5,8.75])

xy_c60 = np.array([202.5,12.5])
xy_c61 = np.array([202,8.75])
xy_c62 = np.array([205,12.5])
xy_c63 = np.array([204.5,8.75])
xy_c64 = np.array([207.5,12.5])
xy_c65 = np.array([207,8.75])

xy_c66 = np.array([210,12.5])
xy_c67 = np.array([209.5,8.75])
xy_c68 = np.array([212.5,12.5])
xy_c69 = np.array([212,8.75])

















circle_c1 = mpathes.Circle(xy_c1,0.3,color= 'r')
circle_c2 = mpathes.Circle(xy_c2,0.3,color= 'r')
circle_c3 = mpathes.Circle(xy_c3,0.3,color= 'r')
circle_c4 = mpathes.Circle(xy_c4,0.3,color= 'r')

circle_c5 = mpathes.Circle(xy_c5,0.3,color= 'r')
circle_c6 = mpathes.Circle(xy_c6,0.3,color= 'r')
circle_c7 = mpathes.Circle(xy_c7,0.3,color= 'r')
circle_c8 = mpathes.Circle(xy_c8,0.3,color= 'r')

circle_c9 = mpathes.Circle(xy_c9,0.3,color= 'r')
circle_c10 = mpathes.Circle(xy_c10,0.3,color= 'r')
circle_c11 = mpathes.Circle(xy_c11,0.3,color= 'r')
circle_c12 = mpathes.Circle(xy_c12,0.3,color= 'r')
circle_c13 = mpathes.Circle(xy_c13,0.3,color= 'r')
circle_c14 = mpathes.Circle(xy_c14,0.3,color= 'r')
circle_c15 = mpathes.Circle(xy_c15,0.3,color= 'r')
circle_c16 = mpathes.Circle(xy_c16,0.3,color= 'r')
circle_c17 = mpathes.Circle(xy_c17,0.3,color= 'r')
circle_c18 = mpathes.Circle(xy_c18,0.3,color= 'r')
circle_c19 = mpathes.Circle(xy_c19,0.3,color= 'r')
circle_c20 = mpathes.Circle(xy_c20,0.3,color= 'r')
circle_c21 = mpathes.Circle(xy_c21,0.3,color= 'r')
circle_c22 = mpathes.Circle(xy_c22,0.3,color= 'r')
circle_c23 = mpathes.Circle(xy_c23,0.3,color= 'r')
circle_c24 = mpathes.Circle(xy_c24,0.3,color= 'r')
circle_c25 = mpathes.Circle(xy_c25,0.3,color= 'r')
circle_c26 = mpathes.Circle(xy_c26,0.3,color= 'r')
circle_c27 = mpathes.Circle(xy_c27,0.3,color= 'r')

circle_c28 = mpathes.Circle(xy_c28,0.3,color= 'r')
circle_c29 = mpathes.Circle(xy_c29,0.3,color= 'r')
circle_c30 = mpathes.Circle(xy_c30,0.3,color= 'r')
circle_c31 = mpathes.Circle(xy_c31,0.3,color= 'r')
circle_c32 = mpathes.Circle(xy_c32,0.3,color= 'r')
circle_c33 = mpathes.Circle(xy_c33,0.3,color= 'r')
circle_c34 = mpathes.Circle(xy_c34,0.3,color= 'r')
circle_c35 = mpathes.Circle(xy_c35,0.3,color= 'r')
circle_c36 = mpathes.Circle(xy_c36,0.3,color= 'r')
circle_c37 = mpathes.Circle(xy_c37,0.3,color= 'r')
circle_c38 = mpathes.Circle(xy_c38,0.3,color= 'r')
circle_c39 = mpathes.Circle(xy_c39,0.3,color= 'r')

circle_c40 = mpathes.Circle(xy_c40,0.3,color= 'r')
circle_c41 = mpathes.Circle(xy_c41,0.3,color= 'r')
circle_c42 = mpathes.Circle(xy_c42,0.3,color= 'r')
circle_c43 = mpathes.Circle(xy_c43,0.3,color= 'r')
circle_c44 = mpathes.Circle(xy_c44,0.3,color= 'r')
circle_c45 = mpathes.Circle(xy_c45,0.3,color= 'r')
circle_c46 = mpathes.Circle(xy_c46,0.3,color= 'r')
circle_c47 = mpathes.Circle(xy_c47,0.3,color= 'r')
circle_c48 = mpathes.Circle(xy_c48,0.3,color= 'r')
circle_c49 = mpathes.Circle(xy_c49,0.3,color= 'r')
circle_c50 = mpathes.Circle(xy_c50,0.3,color= 'r')

circle_c51 = mpathes.Circle(xy_c51,0.3,color= 'r')
circle_c52 = mpathes.Circle(xy_c52,0.3,color= 'r')
circle_c53 = mpathes.Circle(xy_c53,0.3,color= 'r')
circle_c54 = mpathes.Circle(xy_c54,0.3,color= 'r')
circle_c55 = mpathes.Circle(xy_c55,0.3,color= 'r')
circle_c56 = mpathes.Circle(xy_c56,0.3,color= 'r')
circle_c57 = mpathes.Circle(xy_c57,0.3,color= 'r')
circle_c58 = mpathes.Circle(xy_c58,0.3,color= 'r')
circle_c59 = mpathes.Circle(xy_c59,0.3,color= 'r')
circle_c60 = mpathes.Circle(xy_c60,0.3,color= 'r')
circle_c61 = mpathes.Circle(xy_c61,0.3,color= 'r')
circle_c62 = mpathes.Circle(xy_c62,0.3,color= 'r')

circle_c63 = mpathes.Circle(xy_c63,0.3,color= 'r')
circle_c64 = mpathes.Circle(xy_c64,0.3,color= 'r')
circle_c65 = mpathes.Circle(xy_c65,0.3,color= 'r')
circle_c66 = mpathes.Circle(xy_c66,0.3,color= 'r')
circle_c67 = mpathes.Circle(xy_c67,0.3,color= 'r')
circle_c68 = mpathes.Circle(xy_c68,0.3,color= 'r')
circle_c69 = mpathes.Circle(xy_c69,0.3,color= 'r')















fig1,ax1 = plt.subplots()

ax1.add_patch(circle_c1)
ax1.add_patch(circle_c2)
ax1.add_patch(circle_c3)
ax1.add_patch(circle_c4)

ax1.add_patch(circle_c5)
ax1.add_patch(circle_c6)
ax1.add_patch(circle_c7)
ax1.add_patch(circle_c8)

ax1.add_patch(circle_c9)
ax1.add_patch(circle_c10)
ax1.add_patch(circle_c11)
ax1.add_patch(circle_c12)
ax1.add_patch(circle_c13)
ax1.add_patch(circle_c14)
ax1.add_patch(circle_c15)
ax1.add_patch(circle_c16)
ax1.add_patch(circle_c17)
ax1.add_patch(circle_c18)
ax1.add_patch(circle_c19)
ax1.add_patch(circle_c20)
ax1.add_patch(circle_c21)
ax1.add_patch(circle_c22)
ax1.add_patch(circle_c23)
ax1.add_patch(circle_c24)
ax1.add_patch(circle_c25)
ax1.add_patch(circle_c26)
ax1.add_patch(circle_c27)
ax1.add_patch(circle_c28)
ax1.add_patch(circle_c29)
ax1.add_patch(circle_c30)

ax1.add_patch(circle_c31)
ax1.add_patch(circle_c32)
ax1.add_patch(circle_c33)
ax1.add_patch(circle_c34)
ax1.add_patch(circle_c35)
ax1.add_patch(circle_c36)
ax1.add_patch(circle_c37)
ax1.add_patch(circle_c38)
ax1.add_patch(circle_c39)
ax1.add_patch(circle_c40)

ax1.add_patch(circle_c41)
ax1.add_patch(circle_c42)
ax1.add_patch(circle_c43)
ax1.add_patch(circle_c44)
ax1.add_patch(circle_c45)

ax1.add_patch(circle_c46)
ax1.add_patch(circle_c47)
ax1.add_patch(circle_c48)
ax1.add_patch(circle_c49)
ax1.add_patch(circle_c50)

ax1.add_patch(circle_c51)
ax1.add_patch(circle_c52)
ax1.add_patch(circle_c53)
ax1.add_patch(circle_c54)
ax1.add_patch(circle_c55)
ax1.add_patch(circle_c56)
ax1.add_patch(circle_c57)
ax1.add_patch(circle_c58)
ax1.add_patch(circle_c59)
ax1.add_patch(circle_c60)
ax1.add_patch(circle_c61)
ax1.add_patch(circle_c62)
ax1.add_patch(circle_c63)
ax1.add_patch(circle_c64)
ax1.add_patch(circle_c65)
ax1.add_patch(circle_c66)
ax1.add_patch(circle_c67)
ax1.add_patch(circle_c68)
ax1.add_patch(circle_c69)



# ax1.add_patch(circle_c40)








# ax1.add_patch(circle_c21)
# ax1.add_patch(circle_c22)
# ax1.add_patch(circle_c23)
# ax1.add_patch(circle_c24)


# ax.add_patch(circle105

ax1.scatter(Lane_coor_L_x, Lane_coor_L_y, s=0.3)
ax1.scatter(Lane_coor_M_x, Lane_coor_M_y, s=0.3)
ax1.scatter(Lane_coor_R_x, Lane_coor_R_y, s=0.3)
ax1.scatter(direct_track_x, direct_track_y, s=0.3)


# ax1.scatter(coor_coor_x, coor_coor_y, s=0.3)
# ax1.scatter(coor_coor_x, coor_coor_y1, s=0.3)


plt.axis("equal")
plt.show()

cone_coor_x = []
cone_coor_y = []

# coor_coor_x.append( float(xy_c1[0]))
# coor_coor_x.append( float(xy_c2[0]))
# coor_coor_x.append( float(xy_c3[0]))
# coor_coor_x.append( float(xy_c4[0]))
# coor_coor_x.append( float(xy_c5[0]))
# coor_coor_x.append( float(xy_c6[0]))
# coor_coor_x.append( float(xy_c7[0]))
# coor_coor_x.append( float(xy_c8[0]))
# coor_coor_x.append( float(xy_c9[0]))
# coor_coor_x.append( float(xy_c10[0]))
# coor_coor_x.append( float(xy_c11[0]))
# coor_coor_x.append( float(xy_c12[0]))
# coor_coor_x.append( float(xy_c13[0]))
# coor_coor_x.append( float(xy_c14[0]))
# coor_coor_x.append( float(xy_c15[0]))
# coor_coor_x.append( float(xy_c16[0]))
# coor_coor_x.append( float(xy_c17[0]))
# coor_coor_x.append( float(xy_c18[0]))
# coor_coor_x.append( float(xy_c19[0]))
# coor_coor_x.append( float(xy_c20[0]))
# coor_coor_x.append( float(xy_c21[0]))
# coor_coor_x.append( float(xy_c22[0]))
# coor_coor_x.append( float(xy_c23[0]))
# coor_coor_x.append( float(xy_c24[0]))
# coor_coor_x.append( float(xy_c25[0]))
# coor_coor_x.append( float(xy_c26[0]))
# coor_coor_x.append( float(xy_c27[0]))
# coor_coor_x.append( float(xy_c28[0]))
# coor_coor_x.append( float(xy_c29[0]))
# coor_coor_x.append( float(xy_c30[0]))
# coor_coor_x.append( float(xy_c31[0]))
# coor_coor_x.append( float(xy_c32[0]))
# coor_coor_x.append( float(xy_c33[0]))
# coor_coor_x.append( float(xy_c34[0]))
# coor_coor_x.append( float(xy_c35[0]))
# coor_coor_x.append( float(xy_c36[0]))
# coor_coor_x.append( float(xy_c37[0]))
# coor_coor_x.append( float(xy_c38[0]))
# coor_coor_x.append( float(xy_c39[0]))
# coor_coor_x.append( float(xy_c40[0]))
# coor_coor_x.append( float(xy_c41[0]))
# coor_coor_x.append( float(xy_c42[0]))
# coor_coor_x.append( float(xy_c43[0]))
# coor_coor_x.append( float(xy_c44[0]))
# coor_coor_x.append( float(xy_c45[0]))
# coor_coor_x.append( float(xy_c46[0]))
# coor_coor_x.append( float(xy_c47[0]))
# coor_coor_x.append( float(xy_c48[0]))
# coor_coor_x.append( float(xy_c49[0]))
# coor_coor_x.append( float(xy_c50[0]))
# coor_coor_x.append( float(xy_c51[0]))
# coor_coor_x.append( float(xy_c52[0]))
# coor_coor_x.append( float(xy_c53[0]))
# coor_coor_x.append( float(xy_c54[0]))
# coor_coor_x.append( float(xy_c55[0]))
# coor_coor_x.append( float(xy_c56[0]))
# coor_coor_x.append( float(xy_c57[0]))
# coor_coor_x.append( float(xy_c58[0]))
# coor_coor_x.append( float(xy_c59[0]))
# coor_coor_x.append( float(xy_c60[0]))
# coor_coor_x.append( float(xy_c61[0]))
# coor_coor_x.append( float(xy_c62[0]))
# coor_coor_x.append( float(xy_c63[0]))
# coor_coor_x.append( float(xy_c64[0]))
# coor_coor_x.append( float(xy_c65[0]))
# coor_coor_x.append( float(xy_c66[0]))
# coor_coor_x.append( float(xy_c67[0]))
# coor_coor_x.append( float(xy_c68[0]))
# coor_coor_x.append( float(xy_c69[0]))
# coor_coor_x.append( float(xy_c70[0]))



for i in range(1,50):
    exec('cone_coor_x.append( float(xy_c%s[0]) )'%i)
    exec('cone_coor_y.append( float(xy_c%s[1]) )'%i)
# print(cone_x)


# print(cone_x)

direct_dict = {'LaneAndConeCoordinate' : [ {'X_l':Lane_coor_L_x}, {'Y_l':Lane_coor_L_y}, {'X_m':Lane_coor_M_x}, {'Y_m':Lane_coor_M_y}, {'X_r':Lane_coor_R_x}, {'Y_r':Lane_coor_R_y}, {'X_c':cone_coor_x}, {'Y_c':cone_coor_y}] }

direct_track_dict = {"track" :[ {"X_t":direct_track_x}, {"Y_T":direct_track_y}]}

with open("./direct_roadmap.json", "w") as f:
    json.dump(direct_dict, f, indent=2, sort_keys=True)

with open("./direct_track_lanechange.json", "w") as f:
    json.dump(direct_track_dict, f, indent=2, sort_keys=True)

