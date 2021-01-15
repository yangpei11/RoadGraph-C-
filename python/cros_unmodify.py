# -*- coding: utf-8 -*-
"""
Created on Mon Oct 28 18:35:02 2019

@author: 30866
"""
import math
import numpy as np
import matplotlib.pyplot as plt
class Intersection:
    
    def __init__(self,ox,oy,x,y):
        self.ox = ox
        self.oy = oy
        self.road=[]
        self._n = len(x)
        for i in range(len(x)):
            vx = x[i]-ox
            vy = y[i]-oy
            l = (vx**2+vy**2)**0.5           
            nx = vx/l
            ny = vy/l
            #求角度
            if vx ==0:
                if vy>=0:
                    theta = 0.5*math.pi
                else:
                    theta = 1.5*math.pi
            else:
                theta = math.atan(vy*1.0/vx)
            if (vx<0):
                theta = math.pi+theta
            self.road.append((x[i],y[i],nx,ny,theta))
    def findL(self,r):
        self.r = r
        self.theta=[]
        #把点沿角度排序
        self.road.sort(key=lambda L:L[4])
        tmin = 100
        imin = -1
        #寻找最小夹角
        for i in range(self._n):
            #第一个减去最后一个
            ti = self.road[i][4]-self.road[i-1][4]
            ti = abs(ti)
            if(ti>= 2*math.pi):
                ti = ti-2*math.pi
            if(ti>= math.pi):
                ti = 2*math.pi - ti;
            self.theta.append(ti)
            if(ti<tmin):
                tmin=ti
                imin=i
        #计算最小边
        L = r/math.tan(0.5*tmin)
        #考虑约束：
        return L
    def plot(self,d,r):
        #r+d -d =r
        L = self.findL(r+d)
        plt.xlim(5,35)
        plt.ylim(5,35)
        plt.gca().set_aspect('equal',adjustable = 'box')
        #绘制第i条弧线,位于第i点与i-1之间
        for i in range(self._n):
            if abs(math.pi-self.theta[i])>0.01:#不为直线
                #旋转方向:
                xx = []
                yy = []
                n1=np.array([self.road[i-1][2],self.road[i-1][3],0])
                n2=np.array([self.road[i][2],self.road[i][3],0])
                o = np.cross(n2,n1)
                o = o/np.linalg.norm(o)
                p = np.cross(n1,o)
                p2 = np.cross(n2,o)
                lam = np.sign(o[2])
                #圆心
                n3 = n1+n2
                n3 = n3/np.linalg.norm(n3)
                ox = self.ox + n3[0]*L/math.cos(0.5*self.theta[i])
                oy = self.oy + n3[1]*L/math.cos(0.5*self.theta[i])
                #初始点
                xx.append(self.road[i-1][0]+d*p[0])
                yy.append(self.road[i-1][1]+d*p[1])
                curvex=[self.ox+L*self.road[i-1][2]+d*p[0]]
                curvey=[self.oy+L*self.road[i-1][3]+d*p[1]]
                lam = -1    
                for _i in range(1,int((math.pi-self.theta[i])/0.01)):
                    x0 = (curvex[0]-ox)*math.cos(lam*_i*0.01)-(curvey[0]-oy)*math.sin(lam*_i*0.01)+ox
                    y0 = (curvex[0]-ox)*math.sin(lam*_i*0.01)+(curvey[0]-oy)*math.cos(lam*_i*0.01)+oy
                    curvex.append(x0)
                    curvey.append(y0)
                    
                #结束点
                curvex.append(self.ox+L*self.road[i][2]-d*p2[0])
                curvey.append(self.oy+L*self.road[i][3]-d*p2[1])
                for i in range(0, len(curvex)):
                    print(curvex[i], curvey[i])
                print(1/0)
                xx = xx + curvex
                yy = yy + curvey
                #终点
                xx.append(self.road[i][0]-d*p2[0])
                yy.append(self.road[i][1]-d*p2[1])
                plt.plot(xx,yy)
            else:#绘制直线
                #计算上一个弧线以便获取方向，负值是循环的
                n1=np.array([self.road[i-2][2],self.road[i-2][3],0])
                n2=np.array([self.road[i-1][2],self.road[i-1][3],0])
                o = np.cross(n2,n1)
                o = o/np.linalg.norm(o)
                p = np.cross(n1,o)
                p2 = np.cross(n2,o)
                xx = []
                yy = []
                #初始点
                xx.append(self.road[i-1][0]+d*p2[0])
                yy.append(self.road[i-1][1]+d*p2[1])
                
                #旋转起始点
                xx.append(self.ox + L*self.road[i-1][2]+d*p2[0])
                yy.append(self.oy + L*self.road[i-1][3]+d*p2[1])
                #旋转结束点
                xx.append(self.ox + L*self.road[i][2]+d*p2[0])
                yy.append(self.oy + L*self.road[i][3]+d*p2[1])
                #结束点
                xx.append(self.road[i][0]+d*p2[0])
                yy.append(self.road[i][1]+d*p2[1])
                plt.plot(xx,yy)
                