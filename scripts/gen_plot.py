"""
Quintic Polynomial
"""

import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.pyplot import MultipleLocator

from matplotlib.font_manager import FontProperties



class QuinticPolynomial:
    def __init__(self, x0, v0, a0, x1, v1, a1, T):
        A = np.array([[T ** 3, T ** 4, T ** 5],
                      [3 * T ** 2, 4 * T ** 3, 5 * T ** 4],
                      [6 * T, 12 * T ** 2, 20 * T ** 3]])
        b = np.array([x1 - x0 - v0 * T - a0 * T ** 2 / 2,
                      v1 - v0 - a0 * T,
                      a1 - a0])
        X = np.linalg.solve(A, b)

        self.a0 = x0
        self.a1 = v0
        self.a2 = a0 / 2.0
        self.a3 = X[0]
        self.a4 = X[1]
        self.a5 = X[2]

    def calc_xt(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t ** 2 + \
                self.a3 * t ** 3 + self.a4 * t ** 4 + self.a5 * t ** 5

        return xt

    def calc_dxt(self, t):
        dxt = self.a1 + 2 * self.a2 * t + \
            3 * self.a3 * t ** 2 + 4 * self.a4 * t ** 3 + 5 * self.a5 * t ** 4

        return dxt

    def calc_ddxt(self, t):
        ddxt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t ** 2 + 20 * self.a5 * t ** 3

        return ddxt

    def calc_dddxt(self, t):
        dddxt = 6 * self.a3 + 24 * self.a4 * t + 60 * self.a5 * t ** 2

        return dddxt


class Trajectory:
    def __init__(self):
        self.t = []
        self.x = []
        self.y = []
        self.vx= []
        self.vy = []
        self.ax = []
        self.ay = []
        self.jerkx = []
        self.jerky = []



def simulation():

    dt = 0.1  # T tick [s]

    T = 5
    T1 =4
    T2 = 3

    # path = Trajectory()

    path = Trajectory()
    #######################
    ##纵向位移50m, 纵向车速10m/s###########
    #######################
    xqp = QuinticPolynomial(0, 10, 0, 50, 10, 0, T)
    yqp = QuinticPolynomial(0, 0, 0, 3.75, 0, 0, T)

    for t in np.arange(0.0, T + dt, dt):
        path.t.append(t)
        path.x.append(xqp.calc_xt(t))
        path.y.append(yqp.calc_xt(t))

        vx = xqp.calc_dxt(t)
        vy = yqp.calc_dxt(t)
        path.vx.append(vx)
        path.vy.append(vy)

        ax = xqp.calc_ddxt(t)
        ay = yqp.calc_ddxt(t)
        path.ax.append(ax)
        path.ay.append(ay)

        jx = xqp.calc_dddxt(t)
        jy = yqp.calc_dddxt(t)
        path.jerkx.append(jx)
        path.jerky.append(jy)


    path1 = Trajectory()
    #######################
    ##纵向位移40m, 纵向车速10m/s###########
    #######################
    xqp1 = QuinticPolynomial(0, 10, 0, 40, 10, 0, T1)
    yqp1 = QuinticPolynomial(0, 0, 0, 3.75, 0, 0, T1)

    for t in np.arange(0.0, T1 + dt, dt):
        path1.t.append(t)
        path1.x.append(xqp1.calc_xt(t))
        path1.y.append(yqp1.calc_xt(t))

        vx1 = xqp1.calc_dxt(t)
        vy1 = yqp1.calc_dxt(t)
        path1.vx.append(vx1)
        path1.vy.append(vy1)

        ax1 = xqp1.calc_ddxt(t)
        ay1 = yqp1.calc_ddxt(t)
        path1.ax.append(ax1)
        path1.ay.append(ay1)

        jx1 = xqp1.calc_dddxt(t)
        jy1 = yqp1.calc_dddxt(t)
        path1.jerkx.append(jx1)
        path1.jerky.append(jy1)
    

    path2 = Trajectory()
    #######################
    ##纵向位移30m, 纵向车速10m/s###########
    #######################
    xqp2 = QuinticPolynomial(0, 10, 0, 30, 10, 0, T2)
    yqp2 = QuinticPolynomial(0, 0, 0, 3.75, 0, 0, T2)

    for t in np.arange(0.0, T2 + dt, dt):
        path2.t.append(t)
        path2.x.append(xqp2.calc_xt(t))
        path2.y.append(yqp2.calc_xt(t))

        vx2 = xqp2.calc_dxt(t)
        vy2 = yqp2.calc_dxt(t)
        path2.vx.append(vx2)
        path2.vy.append(vy2)

        ax2 = xqp2.calc_ddxt(t)
        ay2 = yqp2.calc_ddxt(t)
        path2.ax.append(ax2)
        path2.ay.append(ay2)

        jx2 = xqp2.calc_dddxt(t)
        jy2 = yqp2.calc_dddxt(t)
        path2.jerkx.append(jx2)
        path2.jerky.append(jy2)

    path3 = Trajectory()
    #######################
    ##纵向位移60m, 纵向车速5m/s###########
    #######################
    xqp3 = QuinticPolynomial(0, 5, 0, 60, 5, 0, 12)
    yqp3 = QuinticPolynomial(0, 0, 0, 3.75, 0, 0, 12)

    for t in np.arange(0.0, 12 + dt, dt):
        path3.t.append(t)
        path3.x.append(xqp3.calc_xt(t))
        path3.y.append(yqp3.calc_xt(t))

        vx3 = xqp3.calc_dxt(t)
        vy3 = yqp3.calc_dxt(t)
        path3.vx.append(vx3)
        path3.vy.append(vy3)

        ax3 = xqp3.calc_ddxt(t)
        ay3 = yqp3.calc_ddxt(t)
        path3.ax.append(ax3)
        path3.ay.append(ay3)

        jx3 = xqp3.calc_dddxt(t)
        jy3 = yqp3.calc_dddxt(t)
        path3.jerkx.append(jx3)
        path3.jerky.append(jy3)

    path4 = Trajectory()
    #######################
    ##纵向位移60m, 纵向车速10m/s###########
    #######################
    xqp4 = QuinticPolynomial(0, 10, 0, 60, 10, 0, 6)
    yqp4 = QuinticPolynomial(0, 0, 0, 3.75, 0, 0, 6)

    for t in np.arange(0.0, 6+ dt, dt):
        path4.t.append(t)
        path4.x.append(xqp4.calc_xt(t))
        path4.y.append(yqp4.calc_xt(t))

        vx4 = xqp4.calc_dxt(t)
        vy4 = yqp4.calc_dxt(t)
        path4.vx.append(vx4)
        path4.vy.append(vy4)

        ax4 = xqp4.calc_ddxt(t)
        ay4 = yqp4.calc_ddxt(t)
        path4.ax.append(ax4)
        path4.ay.append(ay4)

        jx4 = xqp4.calc_dddxt(t)
        jy4 = yqp4.calc_dddxt(t)
        path4.jerkx.append(jx4)
        path4.jerky.append(jy4)

    path5 = Trajectory()
    #######################
    ##纵向位移60m, 纵向车速15m/s###########
    #######################
    xqp5 = QuinticPolynomial(0, 15, 0, 60, 15, 0, 4)
    yqp5 = QuinticPolynomial(0, 0, 0, 3.75, 0, 0, 4)

    for t in np.arange(0.0, 4 + dt, dt):
        path5.t.append(t)
        path5.x.append(xqp5.calc_xt(t))
        path5.y.append(yqp5.calc_xt(t))

        vx5 = xqp5.calc_dxt(t)
        vy5 = yqp5.calc_dxt(t)
        path5.vx.append(vx5)
        path5.vy.append(vy5)

        ax5 = xqp5.calc_ddxt(t)
        ay5 = yqp5.calc_ddxt(t)
        path5.ax.append(ax5)
        path5.ay.append(ay5)

        jx5 = xqp5.calc_dddxt(t)
        jy5 = yqp5.calc_dddxt(t)
        path5.jerkx.append(jx5)
        path5.jerky.append(jy5)

    # for i in range(len(path.t)):
        # plt.cla()
    ax = plt.gca()
    plt.figure(1)


    plt.plot(path.x, path.t, linewidth=2, color='gray')
    myfont = FontProperties(fname='/usr/share/fonts/truetype/wqy/wqy-microhei.ttc') 
    plt.rcParams['axes.unicode_minus']=False

    # ax.xaxis.set_major_locator(x_major_locator)
    # ax.yaxis.set_major_locator(y_major_locator)

    plt.figure(2)
    # plt.axis("equal")
    plt.title("横向位移",FontProperties = myfont)
    plt.ylim(0,3.75)
    plt.xlim(0,5)
    l1, =plt.plot(path.t, path.y, linewidth=2, color='black')
    l2, =plt.plot(path1.t, path1.y,linewidth=2, color='blue', linestyle = '--')
    l3, =plt.plot(path2.t, path2.y,linewidth=2, color='orange', linestyle = '-.')
    plt.legend(handles=[l1,l2,l3],labels=['x=50m','x=40m', 'x=30m'],loc='best')
    plt.xlabel("t/s")
    plt.ylabel("Y/m")
    plt.savefig("./x1.svg")


    plt.figure(4)
    plt.xlim(0,5)
    plt.ylim(0,2.4)
    plt.title("横向速度",FontProperties = myfont)
    plt.plot(path.t, path.vy, linewidth=2, color='black')
    plt.plot(path1.t, path1.vy,linewidth=2, color='blue', linestyle = '--')
    plt.plot(path2.t, path2.vy,linewidth=2, color='orange', linestyle = '-.')
    plt.legend(handles=[l1,l2,l3],labels=['x=50m','x=40m', 'x=30m'],loc='best')
    plt.xlabel("t/s")
    plt.ylabel("vy/(m/s)")
    plt.savefig("./v1.svg")


    plt.figure(5)
    plt.xlim(0,5)
    plt.ylim(-2.5, 2.5)
    plt.title("横向加速度",FontProperties = myfont)
    plt.plot(path.t, path.ay, linewidth=2, color='black')
    plt.plot(path1.t, path1.ay,linewidth=2, color='blue', linestyle = '--')
    plt.plot(path2.t, path2.ay,linewidth=2, color='orange', linestyle = '-.')
    plt.legend(handles=[l1,l2,l3],labels=['x=50m','x=40m', 'x=30m'],loc='best')
    plt.xlabel("t/s")
    plt.ylabel("ay/(m/s2)")
    plt.savefig("./a1.svg")


    plt.figure(6)
    plt.xlim(0,5)
    plt.ylim(-4.3, 8.5)
    plt.title("横向急动度",FontProperties = myfont)
    plt.plot(path.t, path.jerky, linewidth=2, color='black')
    plt.plot(path1.t, path1.jerky,linewidth=2, color='blue', linestyle = '--')
    plt.plot(path2.t, path2.jerky,linewidth=2, color='orange', linestyle = '-.')
    plt.legend(handles=[l1,l2,l3],labels=['x=50m','x=40m', 'x=30m'],loc='best')
    plt.xlabel("t/s")
    plt.ylabel("jy/(m/s3)")
    plt.savefig("./jerk1.svg")


    plt.figure(7)
    # plt.axis("equal")
    plt.title("横向位移",FontProperties = myfont)
    plt.ylim(0,3.75)
    plt.xlim(0,12)
    l4, =plt.plot(path3.t, path3.y, linewidth=2, color='black')
    l5, =plt.plot(path4.t, path4.y,linewidth=2, color='blue', linestyle = '--')
    l6, =plt.plot(path5.t, path5.y,linewidth=2, color='orange', linestyle = '-.')
    plt.legend(handles=[l4,l5,l6],labels=['vx=5/s','vx=10m/s', 'vx=15m/s'],loc='best')
    plt.xlabel("t/s")
    plt.ylabel("Y/m")
    plt.savefig("./x2.svg")


    plt.figure(8)
    plt.xlim(0,12)
    plt.ylim(0,1.8)
    plt.title("横向速度",FontProperties = myfont)
    plt.plot(path3.t, path3.vy, linewidth=2, color='black')
    plt.plot(path4.t, path4.vy,linewidth=2, color='blue', linestyle = '--')
    plt.plot(path5.t, path5.vy,linewidth=2, color='orange', linestyle = '-.')
    plt.legend(handles=[l4,l5,l6],labels=['vx=5/s','vx=10m/s', 'vx=15m/s'],loc='best')
    plt.xlabel("t/s")
    plt.ylabel("vy/(m/s)")
    plt.savefig("./v2.svg")

    plt.figure(9)
    plt.xlim(0,12)
    plt.ylim(-1.4, 1.4)
    plt.title("横向加速度",FontProperties = myfont)
    plt.plot(path3.t, path3.ay, linewidth=2, color='black')
    plt.plot(path4.t, path4.ay,linewidth=2, color='blue', linestyle = '--')
    plt.plot(path5.t, path5.ay,linewidth=2, color='orange', linestyle = '-.')
    plt.legend(handles=[l4,l5,l6],labels=['vx=5/s','vx=10m/s', 'vx=15m/s'],loc='best')
    plt.xlabel("t/s")
    plt.ylabel("ay/(m/s2)")
    plt.savefig("./a2.svg")


    plt.figure(10)
    plt.xlim(0,12)
    plt.ylim(-2, 3.6)
    plt.title("横向急动度",FontProperties = myfont)
    plt.plot(path3.t, path3.jerky, linewidth=2, color='black')
    plt.plot(path4.t, path4.jerky,linewidth=2, color='blue', linestyle = '--')
    plt.plot(path5.t, path5.jerky,linewidth=2, color='orange', linestyle = '-.')
    plt.legend(handles=[l4,l5,l6],labels=['vx=5/s','vx=10m/s', 'vx=15m/s'],loc='best')
    plt.xlabel("t/s")
    plt.ylabel("jy/(m/s3)")
    plt.savefig("./jerk2.svg")

    plt.show()


if __name__ == '__main__':
    simulation()
