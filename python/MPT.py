# *-* coding:utf-8 *-*
import matplotlib.pyplot as plt
import numpy as np
import math

class Road:
    def __init__(self, x, y):
        self.xx = []
        self.yy = []
        if(len(x) != 2 ):
            self.xx.append(x[0])
            self.yy.append(y[0])
            pre = 0
            for i in range(1, len(x)-1):
                if( self.getDistance((x[i], y[i]),(x[pre], y[pre])) < 0.1 or self.getDistance( (x[i],y[i]), (x[i+1],y[i+1])) < 0.1):
                    continue
                l1 = (x[pre]-x[i], y[pre]-y[i])
                l2 = (x[i+1]-x[i], y[i+1]-y[i])
                l1 = self.normalize(l1)
                l2 = self.normalize(l2)
                if(self.vectorCalc(l1, l2, "*") > math.cos(175.0/185*math.pi)):
                    self.xx.append(x[i])
                    self.yy.append(y[i])
                    pre = i;
            self.xx.append(x[-1])
            self.yy.append(y[-1])
        else:
            self.xx = x
            self.yy = y

    def getVectorLength(self, e):
        return math.sqrt(e[0]**2 +e[1]**2)

    def normalize(self, e):
        length = self.getVectorLength(e)
        return (e[0]*1.0/length, e[1]*1.0/length)

    #生成AB向量
    def getVector(self, A, B):
        return (B[0]-A[0], B[1]-A[1])

    def vectorCalc(self, e1, e2, operator):
        if(operator == '+'):
            return (e1[0]+e2[0], e1[1]+e2[1])
        elif(operator == '-'):
            return (e2[0]-e1[0], e2[1]-e1[1])
        else:
            return (e1[0]*e2[0] + e1[1]*e2[1])

    def getAngle(self, A, B):
        return math.acos( ((A[0]*B[0]+A[1]*B[1])*1.0)/(math.sqrt(A[0]**2+A[1]**2)*math.sqrt(B[0]**2+B[1]**2)) )

    #A为要移动的点, e是移动的单位向量, d是移动的距离
    def mov(self, A, e, d):
        movVec = (e[0]*d, e[1]*d)
        return self.vectorCalc(A, movVec, '+')

    def getDistance(self, x, y):
        return math.sqrt( (x[0]-y[0])**2 + (x[1]-y[1])**2 )


    def alpha_assign(self, A, s, e, L, f):
        r_min = 1<<30
        i_min = e
        alpha_low = 0
        alpha_high = 0
        if(s+1 >= e):
            return
        alpha_b = min(L[s]- A[s], L[s+1])
        r_current = max(f[s]*A[s], f[s+1]*alpha_b)
        if(r_current < r_min):
            r_min = r_current
            i_min = s
            alpha_low = A[s]
            alpha_high = alpha_b
        for i in range(s+1, e-1):
            alpha_a = min( L[i-1], L[i]*f[i]*1.0/(f[i]+f[i+1]) )
            alpha_b = min( L[i+1], L[i]-alpha_a)
            r_current = max(f[i]*alpha_a, f[i+1]*alpha_b)
            if(r_current < r_min):
                r_min = r_current
                i_min = i
                alpha_low = alpha_a
                alpha_high = alpha_b
        alpha_a = min(L[e-2], L[e-1]-A[e])
        r_current = max(f[e-1]*alpha_a, f[e]*A[e])
        if(r_current < r_min):
            r_min = r_current
            i_min = e-1
            alpha_low = alpha_a
            alpha_high = A[e]
        A[i_min]  = alpha_low
        A[i_min+1] = alpha_high
        self.alpha_assign(A, s, i_min, L, f)
        self.alpha_assign(A, i_min+1, e, L, f)



    def Init(self, x, y, L, f, n):
        for i in range(1, len(x)-1):
            if(i == 1):
                f.append(0)
            v1 = (x[i]-x[i-1], y[i]-y[i-1])
            v2 = (x[i+1]-x[i], y[i+1]-y[i])
            n1 = self.normalize(v1)
            n.append(n1)
            n2 = self.normalize(v2)
            if(i == len(x)-2):
                n.append(n2)
            L.append(self.getVectorLength(v1))
            if(i == len(x)-2):
                L.append( self.getVectorLength(v2) )
            try:
                f.append( math.sqrt( (1+self.vectorCalc(n1,n2,'*'))/(1-self.vectorCalc(n1,n2,'*')) ) )
            except Exception as e:
                print(n1)
                print(n2)
                return
            if(i == len(x)-2):
                f.append(0)

    def drawArc(self, A, B, C, d, width):
        n1 = self.normalize( self.getVector(A, B) )
        n2 = self.normalize( self.getVector(B, C) )
        b = self.normalize( self.vectorCalc(n1, n2, '-') )
        beishu = d*math.sqrt( (1+self.vectorCalc(n1, n2, '*'))*1.0/(1-self.vectorCalc(n1,n2, '*')) + 1)
        O = self.vectorCalc(B, (b[0]*beishu, b[1]*beishu),'+')
        M = self.mov(B, n1, -d)
        N = self.mov(B, n2, d)
        OM = self.getVector(O, M)
        ON = self.getVector(O, N)
        #判断OM是怎么旋转到ON
        start = OM
        angle = self.getAngle(OM, ON)
        newX = OM[0]*math.cos(angle) - OM[1]*math.sin(angle)
        newY = OM[0]*math.sin(angle) + OM[1]*math.cos(angle)
        roateFlag = 1
        if( abs(newX-ON[0]) <0.01 and abs(newY-ON[1]) < 0.01):
            roateFlag = 1
        else:
            #顺势转
            roateFlag = 0
        normalize_OM = self.normalize(OM)
    #print(1/0)
        start1 = self.vectorCalc( (normalize_OM[0]*width, normalize_OM[1]*width), OM, '+')
        start2 = self.vectorCalc( (normalize_OM[0]*width, normalize_OM[1]*width), OM, '-')
        arcX1 = []
        arcY1 = []
        arcX2 = []
        arcY2 = []


        roateAngle = 0
        while(roateAngle < angle):
            tmp = roateAngle
            if(roateFlag == 0):
                roateAngle = -roateAngle
            
            X1 = start1[0]*math.cos(roateAngle) - start1[1]*math.sin(roateAngle)
            Y1 = start1[0]*math.sin(roateAngle) + start1[1]*math.cos(roateAngle)
            X1 = X1 + O[0]
            Y1 = Y1 + O[1]
            arcX1.append(X1)
            arcY1.append(Y1)
            
            X2 = start2[0]*math.cos(roateAngle) - start2[1]*math.sin(roateAngle)
            Y2 = start2[0]*math.sin(roateAngle) + start2[1]*math.cos(roateAngle)
            X2 = X2 + O[0]
            Y2 = Y2 + O[1]
            arcX2.append(X2)
            arcY2.append(Y2)
            '''
            if(roateFlag):
                X = start[0]*math.cos(roateAngle) - start[1]*math.sin(roateAngle)
                Y = start[0]*math.sin(roateAngle) + start[1]*math.cos(roateAngle)
            else:
                X = start[0]*math.cos(-roateAngle) - start[1]*math.sin(-roateAngle)
                Y = start[0]*math.sin(-roateAngle) + start[1]*math.cos(-roateAngle)
                '''
            roateAngle = tmp
            roateAngle = roateAngle + 0.01
        return (arcX1, arcY1, arcX2, arcY2, M, N)

    def Collinear(self, e1, e2):
        if( abs(e1[0] * e2[1] - e1[1]*e2[0]) <= 0.001):
            return True
        else:
            return False

    def vectorRoate(self, e, angle):
        return (e[0]*math.cos(angle) - e[1]*math.sin(angle), e[0]*math.sin(angle) + e[1]*math.cos(angle))

    def getCentralAxis(self, X1, Y1, X2, Y2):
        X = []
        Y = []
        for i in range(0, len(X1)):
            X.append( (X1[i]+X2[i])/2.0)
            Y.append( (Y1[i]+Y2[i])/2.0)
        return (X,Y)

    def plot(self):
        x = self.xx
        y = self.yy
        if(len(x) == 1):
            return
        if(len(x) == 2):
            if( self.getDistance((x[0], y[0]),(x[1], y[1])) < 0.1):
                return
            result1X = []
            result1Y = []
            result2X = []
            result2Y = []
            e = self.normalize(self.vectorRoate(self.getVector((x[0], y[0]), (x[1], y[1])), math.pi/2.0))
            p = self.vectorCalc( (e[0]*0.5, e[1]*0.5), (x[0], y[0]), '+')
            result1X.append(p[0])
            result1Y.append(p[1])
            q = self.vectorCalc( (e[0]*0.5, e[1]*0.5) , (x[0], y[0]), '-')
            result2X.append(q[0])
            result2Y.append(q[1])

            p = self.vectorCalc( (e[0]*0.5, e[1]*0.5), (x[1], y[1]), '+')
            result1X.append(p[0])
            result1Y.append(p[1])
            q = self.vectorCalc( (e[0]*0.5, e[1]*0.5) , (x[1], y[1]), '-')
            result2X.append(q[0])
            result2Y.append(q[1])
            plt.plot(result1X, result1Y, color = 'skyblue')
            X, Y = self.getCentralAxis(result1X, result1Y, result2X, result2Y)
            plt.plot(X, Y, color = 'skyblue')
            plt.plot(result2X, result2Y, color = 'skyblue')
            return


        A = [0]*len(x)
        L = []
        f = []
        n = []
        self.Init(x, y, L, f, n)
        self.alpha_assign(A, 0, len(x)-1, L, f)
        startPoint = (x[0], y[0])
        result1X = []
        result1Y = []
        result2X = []
        result2Y = []
        for i in range(1, len(x)-1):
            arcX1, arcY1, arcX2, arcY2, M, N = self.drawArc( (x[i-1], y[i-1]), (x[i], y[i]), (x[i+1], y[i+1]), A[i], 0.5)
            e = self.normalize(self.vectorRoate(self.getVector((x[i-1], y[i-1]), (x[i], y[i])), math.pi/2.0))
            p = self.vectorCalc( (e[0]*0.5, e[1]*0.5), startPoint, '+')
            result1X.append(p[0])
            result1Y.append(p[1])
            q = self.vectorCalc( (e[0]*0.5, e[1]*0.5) , startPoint, '-')
            result2X.append(q[0])
            result2Y.append(q[1])
            if( self.Collinear(self.getVector(p, (arcX1[0], arcY1[0])), self.getVector(startPoint, (x[i], y[i]))) ):
                result1X = result1X + arcX1
                result1Y = result1Y + arcY1
                result2X = result2X + arcX2
                result2Y = result2Y + arcY2
            else:
                result1X = result1X + arcX2
                result1Y = result1Y + arcY2
                result2X = result2X + arcX1
                result2Y = result2Y + arcY1
            startPoint = N
            if(i == len(x)-2):
                e = self.normalize(self.vectorRoate(self.getVector((x[i], y[i]), (x[i+1], y[i+1])), math.pi/2.0))
                p = self.vectorCalc((e[0]*0.5, e[1]*0.5), (x[i+1], y[i+1]), '+')
                result1X.append(p[0])
                result1Y.append(p[1])
                q = self.vectorCalc((e[0]*0.5, e[1]*0.5), (x[i+1], y[i+1]), '-')
                result2X.append(q[0])
                result2Y.append(q[1])
        plt.plot(result1X, result1Y, color = 'skyblue')
        X, Y = self.getCentralAxis(result1X, result1Y, result2X, result2Y)
        plt.plot(X, Y, color = 'skyblue')
        plt.plot(result2X, result2Y, color = 'skyblue')
        
        '''
        plt.xlim(-5,40)
        plt.ylim(-3,20)
        plt.gca().set_aspect('equal', adjustable='box')
        plt.show()
        '''
        

def main():
    
    
    A = (-3,-3)
    B = (3,3)
    C = (6,3)
    D = (8,5)
    E = (13,2)
    F = (20,10)
    G = (10,17)
    H = (5.3, 10.2)
    '''
    A = (5, 5)
    B = (10, 6)
    C = (15, 5)
    D = (20, 6)
    E = (25, 5)
    F = (30, 6)
    G = (35, 5)
    H = (40, 6)
    '''
    

    x = [A[0],B[0],C[0],D[0],E[0],F[0], G[0], H[0]]
    y = [A[1],B[1],C[1],D[1],E[1],F[1], G[1], H[1]]
    '''
    plt.plot(x,y)
    plt.scatter(x,y,color='r')
    '''
    A = [0]*len(x)
    L = []
    f = []
    n = []
    init(x, y, L, f, n)
    alpha_assign(A, 0, len(x)-1, L, f)
    startPoint = (x[0], y[0])
    result1X = []
    result1Y = []
    result2X = []
    result2Y = []
    for i in range(1, len(x)-1):
        arcX1, arcY1, arcX2, arcY2, M, N = drawArc( (x[i-1], y[i-1]), (x[i], y[i]), (x[i+1], y[i+1]), A[i], 1)
        e = normalize(vectorRoate(getVector((x[i-1], y[i-1]), (x[i], y[i])), math.pi/2.0))
        p = vectorCalc(e, startPoint, '+')
        result1X.append(p[0])
        result1Y.append(p[1])
        q = vectorCalc(e, startPoint, '-')
        result2X.append(q[0])
        result2Y.append(q[1])
        if( Collinear(getVector(p, (arcX1[0], arcY1[0])), getVector(startPoint, (x[i], y[i]))) ):
            result1X = result1X + arcX1
            result1Y = result1Y + arcY1
            result2X = result2X + arcX2
            result2Y = result2Y + arcY2
        else:
            result1X = result1X + arcX2
            result1Y = result1Y + arcY2
            result2X = result2X + arcX1
            result2Y = result2Y + arcY1
        startPoint = N
        if(i == len(x)-2):
            e = normalize(vectorRoate(getVector((x[i], y[i]), (x[i+1], y[i+1])), math.pi/2.0))
            p = vectorCalc(e, (x[i+1], y[i+1]), '+')
            result1X.append(p[0])
            result1Y.append(p[1])
            q = vectorCalc(e, (x[i+1], y[i+1]), '-')
            result2X.append(q[0])
            result2Y.append(q[1])
    plt.plot(result1X, result1Y, color = '')
    plt.plot(result2X, result2Y, color = 'g')
    
    '''
    面片绘制
    for i in range(0, len(result1X)-1 ):
        plt.plot([result1X[i], result1X[i+1], result2X[i], result1X[i]], [result1Y[i], result1Y[i+1],  result2Y[i], result1Y[i]], color = 'r')
        plt.plot([result1X[i+1], result2X[i+1], result2X[i]], [result1Y[i+1], result2Y[i+1], result2Y[i]], color = 'r')
    '''
    '''
    plt.xlim(-5,40)
    plt.ylim(-3,20)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()
    '''
    #print(A)

#main()

'''
A = (-3,-3)
B = (3,3)
C = (6,3)
D = (8,5)
E = (13,2)
F = (20,10)
G = (10,17)
H = (5.3, 10.2)
    

x = [A[0],B[0],C[0],D[0],E[0],F[0], G[0], H[0]]
y = [A[1],B[1],C[1],D[1],E[1],F[1], G[1], H[1]]
road = Road(x, y)
road.plot()
'''