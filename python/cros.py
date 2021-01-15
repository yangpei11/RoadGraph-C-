# -*- coding: utf-8 -*-
"""
Created on Mon Oct 28 18:35:02 2019

@author: 30866
"""
import math
import numpy as np
import matplotlib.pyplot as plt
from graph import Point
import uuid
 
def create_uid():
    return str(uuid.uuid1())

class Intersection:
    
    def __init__(self,ox,oy,x,y,pointId,pointMap,Id, oneDegreePoints):
        self.ox = ox
        self.oy = oy
        self.pointMap = pointMap
        self.oneDegreePoints = oneDegreePoints
        self.Id = Id
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
            self.road.append((x[i],y[i],nx,ny,theta, pointId[i]))
    def findL(self,r):
        self.r = r
        self.theta=[]
        #把点沿角度排序
        self.road.sort(key=lambda L:L[4])
        tmin = 100
        imin = -1
        Lmin = 1<<30
        #寻找最小夹角
        for i in range(self._n):
            #第一个减去最后一个
            Lmin = min(Lmin, math.sqrt( (self.ox-self.road[i][0])**2 + (self.oy-self.road[i][1])**2))
            ti = self.road[i][4]-self.road[i-1][4]
            if(ti < 0):
                ti = ti+2*math.pi
            self.theta.append(ti)
            if(ti<tmin):
                tmin=ti
                imin=i
        L = r/math.tan(0.5*tmin)
        L = min(L, Lmin)
        #考虑约束： 是否考虑要在里面选最短的和L比较
        return L
    def plot(self,d,r):
        #r+d -d =r
        L = self.findL(r+d)
        #plt.xlim(0,35)
        #plt.ylim(5,35)
        #plt.gca().set_aspect('equal',adjustable = 'box')
        #绘制第i条弧线,位于第i点与i-1之间
        for i in range(self._n):
            if abs(math.pi-self.theta[i])>0.01:#不为直线
                #旋转方向:
                xx = []
                yy = []
                n1=np.array([self.road[i-1][2],self.road[i-1][3],0])
                n2=np.array([self.road[i][2],self.road[i][3],0])
                o = np.cross(n1,n2)
                o = o/np.linalg.norm(o)
                lam = np.sign(o[2])
                p = np.cross(n1,o)*(-lam)
                p2 = np.cross(n2,o)*(-lam)
                
                #圆心
                n3 = n1+n2
                n3 = n3/np.linalg.norm(n3)
                if(lam>=0):
                    ox = self.ox + n3[0]*L/(math.cos(0.5*self.theta[i]))
                    oy = self.oy + n3[1]*L/(math.cos(0.5*self.theta[i]))
                else:
                    ox = self.ox + n3[0]*L/(math.cos(math.pi-0.5*self.theta[i]))
                    oy = self.oy + n3[1]*L/(math.cos(math.pi-0.5*self.theta[i]))
                #初始点
                '''
                xx.append(self.road[i-1][0]+d*p[0])
                yy.append(self.road[i-1][1]+d*p[1])
                '''
                '''
                print(">>>>>>>")
                print(L)
                print(self.ox+L*self.road[i-1][2])
                print(self.oy+L*self.road[i-1][3])
                print(">>>>>>>")
                '''
                self.pointMap[ self.road[i-1][5] ].adjVertex.remove(self.Id)
                point = Point(1, create_uid(), self.ox+L*self.road[i-1][2], self.oy+L*self.road[i-1][3])
                self.pointMap[ self.road[i-1][5] ].adjVertex.append(point.pointId)
                self.pointMap[point.pointId] = point
                point.adjVertex.append(self.road[i-1][5])
                self.oneDegreePoints.append(point.pointId)

                curvex=[self.ox+L*self.road[i-1][2]+d*p[0]]
                curvey=[self.oy+L*self.road[i-1][3]+d*p[1]]    
                for _i in range(1,int(lam*(math.pi-self.theta[i])/0.01)):
                    x0 = (curvex[0]-ox)*math.cos(-lam*_i*0.01)-(curvey[0]-oy)*math.sin(-lam*_i*0.01)+ox
                    y0 = (curvex[0]-ox)*math.sin(-lam*_i*0.01)+(curvey[0]-oy)*math.cos(-lam*_i*0.01)+oy
                    curvex.append(x0)
                    curvey.append(y0)
                    
                #结束点

                curvex.append(self.ox+L*self.road[i][2]-d*p2[0])
                curvey.append(self.oy+L*self.road[i][3]-d*p2[1])
                xx = xx + curvex
                yy = yy + curvey
                #终点
                '''
                xx.append(self.road[i][0]-d*p2[0])
                yy.append(self.road[i][1]-d*p2[1])
                '''
                plt.plot(xx,yy, color = 'coral')
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
                '''
                xx.append(self.road[i-1][0]+d*p2[0])
                yy.append(self.road[i-1][1]+d*p2[1])
                '''
                self.pointMap[ self.road[i-1][5] ].adjVertex.remove(self.Id)
                point = Point(1, create_uid(), self.ox+L*self.road[i-1][2], self.oy+L*self.road[i-1][3])
                self.pointMap[ self.road[i-1][5] ].adjVertex.append(point.pointId)
                self.pointMap[point.pointId] = point
                point.adjVertex.append(self.road[i-1][5])
                self.oneDegreePoints.append(point.pointId)

                #旋转起始点
                xx.append(self.ox + L*self.road[i-1][2]+d*p2[0])
                yy.append(self.oy + L*self.road[i-1][3]+d*p2[1])
                #旋转结束点
                xx.append(self.ox + L*self.road[i][2]+d*p2[0])
                yy.append(self.oy + L*self.road[i][3]+d*p2[1])
                #结束点
                '''
                xx.append(self.road[i][0]+d*p2[0])
                yy.append(self.road[i][1]+d*p2[1])
                '''
                plt.plot(xx,yy,color = 'coral')
                