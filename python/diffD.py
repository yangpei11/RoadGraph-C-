# -*- coding: utf-8 -*-
"""
Created on Mon Oct 28 18:35:02 2019

@author: 30866
"""
import math
import numpy as np
import matplotlib.pyplot as plt
class Intersection:
    
    def __init__(self,ox,oy,x,y,d):
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
                theta = math.atan(vy/(1.0*vx))
            if (vx<0):
                theta = math.pi+theta
            self.road.append([x[i],y[i],nx,ny,theta,d[i]])
    def findL(self,r):
        self.r = r
        self.theta=[]
        self.L = []
        #把点沿角度排序
        self.road.sort(key=lambda L:L[4])
        tmin = 100
        imin = -1
        #寻找最小夹角
        for i in range(self._n):
            #第一个减去最后一个
            ti = self.road[i][4]-self.road[i-1][4]
            #ti = abs(ti)
            if(ti<0):
                ti = ti+2*math.pi;
            #if(ti>= 2*math.pi):
            #    ti = ti-2*math.pi
            #if(ti>= math.pi):
            #   ti = 2*math.pi - ti;
            self.theta.append(ti)
            L2 = math.sqrt((self.road[i][0]-self.ox)**2+(self.road[i][1]-self.oy)**2)
            L1 = math.sqrt((self.road[i-1][0]-self.ox)**2+(self.road[i-1][1]-self.oy)**2)
            if(ti > math.pi * 175.0/180):
                self.L.append( (L1, L2, 0) )
                continue
            if(ti > math.pi/2):# 如果是钝角
                L = math.sqrt(self.road[i-1][5]**2 + self.road[i][5]**2 - 2*self.road[i-1][5]*self.road[i][5]*abs(math.cos(ti)))/math.sin(ti)
            else:
                L = math.sqrt(self.road[i-1][5]**2 + self.road[i][5]**2 + 2*self.road[i-1][5]*self.road[i][5]*abs(math.cos(ti)))/math.sin(ti)
            d1 = self.road[i-1][5]
            d2 = self.road[i][5]
            l1 = math.sqrt(L**2-d1**2)
            l2 = math.sqrt(L**2-d2**2)
            l3 = r/math.tan(ti/2.0)
            
            if(ti > math.pi/2 and abs(math.asin(d1/L)+math.asin(d2/L)-ti) > 0.1):
                if(d1>d2):
                    l1 = -l1
                else:
                    l2 = -l2
            '''
            print(ti*180/math.pi)
            print("l1=" + str(l1) + " L1=" + str(L1))
            print("l2=" + str(l2) + " L2=" + str(L2))
            print("l3=" + str(l3))
            print("r=" + str(r))
            '''
            
            if( l1 > L1 ):
                angle = ti - math.atan(d1/L1)
                LL = math.sqrt(L1**2 + d1**2)
                self.road[i][5] = math.sin(angle) * LL
                self.L.append( (L1, math.sqrt(LL*LL - self.road[i][5]**2), 0 ) )
                continue
            
            if( l2 > L2):
                angle = ti - math.atan(d2/L2)
                LL = math.sqrt(L2**2 + d2**2)
                self.road[i][5] = math.sin(angle) * LL
                self.L.append( (LL*math.cos(angle), L2, 0) )
                continue
            
            
            
            if(l1 + l3 < L1 and l2 + l3 < L2 and l3 < r):
                l = min(L1-l1, L2-l2, r)
                self.L.append( (l1+l, l2+l, l*math.tan(ti/2.0)) )     
            else:
                l = min(L1-l1, L2-l2, l3)
                self.L.append( (l1 + l, l2 + l, l*math.tan(ti/2.0)))
        
            #self.L.append( (l1 + l3, l2 + l3))



            #L1 = math.sqrt((self.road[i][0]-self.ox)**2+(self.road[i][1]-self.oy)**2)
            #L2 = math.sqrt((self.road[i-1][0]-self.ox)**2+(self.road[i-1][1]-self.oy)**2)
            #Lu = d/math.tan(0.5*ti)
            #Lc = r/math.tan(0.5*ti)
            #tempL = min(Lc,L1,L2)
            #self.L.append(tempL)
            #if(ti<tmin):
            #    tmin=ti
            #    imin=i
        #考虑约束：
    def plot(self,r):
        #r+d -d =r
        self.findL(r)
        #print(self.L)
        #plt.xlim(5,35)
        #plt.ylim(5,35)
        #plt.gca().set_aspect('equal',adjustable = 'box')
        #绘制第i条弧线,位于第i点与i-1之间
        for i in range(self._n):
            L1, L2, R = self.L[i]
            if abs(math.pi-self.theta[i])>0.001:#不为直线
                xx = []
                yy = []
                #旋转方向:
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
                '''
                if(lam>=0):
                    ox = self.ox + n3[0]*L/(math.cos(0.5*self.theta[i]))
                    oy = self.oy + n3[1]*L/(math.cos(0.5*self.theta[i]))
                else:
                    ox = self.ox + n3[0]*L/(math.cos(math.pi-0.5*self.theta[i]))
                    oy = self.oy + n3[1]*L/(math.cos(math.pi-0.5*self.theta[i]))
                '''
                ox = self.ox + L1 * self.road[i-1][2] + (R + self.road[i-1][5]) * p[0]
                oy = self.oy + L1 * self.road[i-1][3] + (R + self.road[i-1][5]) * p[1]
                #初始点
                xx.append(self.road[i-1][0]+self.road[i-1][5]*p[0])
                yy.append(self.road[i-1][1]+self.road[i-1][5]*p[1])
                curvex=[ self.ox+L1*self.road[i-1][2]+self.road[i-1][5]*p[0] ]
                curvey=[ self.oy+L1*self.road[i-1][3]+self.road[i-1][5]*p[1] ]
                for _i in range(1,int(lam*(math.pi-self.theta[i])/0.01)):
                    x0 = (curvex[0]-ox)*math.cos(-lam*_i*0.01)-(curvey[0]-oy)*math.sin(-lam*_i*0.01)+ox
                    y0 = (curvex[0]-ox)*math.sin(-lam*_i*0.01)+(curvey[0]-oy)*math.cos(-lam*_i*0.01)+oy
                    curvex.append(x0)
                    curvey.append(y0)
                    
                #结束点
                curvex.append(self.ox+L2*self.road[i][2]-self.road[i][5]*p2[0])
                curvey.append(self.oy+L2*self.road[i][3]-self.road[i][5]*p2[1])
                xx = xx + curvex
                yy = yy + curvey
                #终点
                xx.append(self.road[i][0]-self.road[i][5]*p2[0])
                yy.append(self.road[i][1]-self.road[i][5]*p2[1])
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
                xx.append(self.road[i-1][0]+self.road[i-1][5]*p2[0])
                yy.append(self.road[i-1][1]+self.road[i-1][5]*p2[1])
                
                #旋转起始点
                '''
                xx.append(self.ox+L1*self.road[i-1][2]+self.road[i-1][5]*p2[0])
                yy.append(self.oy+L1*self.road[i-1][2]+self.road[i-1][5]*p2[0])
                #旋转结束点
                xx.append(self.ox + L2*self.road[i][2]+self.road[i][5]*p2[0])
                yy.append(self.oy + L2*self.road[i][3]+self.road[i][5]*p2[1])
                '''
                #结束点
                xx.append(self.road[i][0]+self.road[i][5]*p2[0])
                yy.append(self.road[i][1]+self.road[i][5]*p2[1])
                plt.plot(xx,yy)


x = [1, 9, 8]
y = [4, 4, 7]
d = [1.0, 1.5, 2.5]
intersection = Intersection(5, 4, x, y,d)

'''
for i in range(0, len(x)):
    xx = [5]
    yy = [4]
    xx.append(x[i])
    yy.append(y[i])
    plt.plot(xx, yy)
    '''

intersection.plot(0.5)


plt.xlim(0,15)
plt.ylim(-3,15)
plt.gca().set_aspect('equal',adjustable = 'box')

plt.show()
        
        
        
