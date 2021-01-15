# *-* coding:utf-8 *-*
from cros_v1 import Intersection
import matplotlib.pyplot as plt

#普通四叉路
'''
ox = 5
oy = 4
x = [2, 7, 6, 11]
y = [1, 2, 7, 6]
'''

#T字型三叉路
'''
ox = 5
oy = 4
x = [1, 5, 9, 5]
y = [4, 1, 4, 7]
'''

'''
ox = 5
oy = 4
x = [2, 7, 6, 11, 1, 3]
y = [1, 2, 7, 6, 4, 7]
'''

ox = 5
oy = 4
x = [1, 2, 8, 9]
y = [4, 1, 7, 4]



for i in range(0, len(x) ):
    plt.plot( [ox, x[i]], [oy, y[i]], color = 'skyblue')


intersection = Intersection(ox, oy, x, y, False)
intersection.plot(0.5, 0.7) 
plt.xlim(0,15)
plt.ylim(0,15)
plt.gca().set_aspect('equal',adjustable = 'box')
plt.show()


