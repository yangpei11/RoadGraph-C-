# *-* coding:utf-8 *-*
import xml.dom.minidom as xmldom 
import os

class Node:
	def __init__(self, degree, isStart):
		self.degree = degree
		self.isStart = isStart

nodeMap = dict()
#j计算度
domObj = xmldom.parse("shenzhen1.osm")
elementObj = domObj.documentElement

ways = elementObj.getElementsByTagName("way")
for way in ways:
	nodes  = way.getElementsByTagName("nd")
	for index in range(0, len(nodes) ):
		nodeId = nodes[index].getAttribute("ref")
		if(index == 0 or index == len(nodes)-1):
			if(nodeMap.get(nodeId) == None):
				nd = Node(1,True)
				nodeMap[nodeId] = nd
			else:
				nodeMap[nodeId].degree += 1
				nodeMap[nodeId].isStart = True
		else:
			if(nodeMap.get(nodeId) == None):
				nd = Node(2 ,False)
				nodeMap[nodeId] = nd
			else:
				nodeMap[nodeId].degree += 2
	
#结论: 三岔路口由路的端点和另一条路的内部节点组成


for nodeId in nodeMap:
	if(nodeMap[nodeId].degree == 3 and nodeMap[nodeId].isStart == True):
		print(nodeId)




'''
#结论： 四岔路口由闭环路顶点和另一条路的中间节点形成 或者 两条路的内部节点相交形成
for nodeId in nodeMap:
	if(nodeMap[nodeId].degree == 4 and nodeMap[nodeId].isStart == True):
		print(nodeId)
'''



'''
#结论： 五岔路口为一段路顶点，和另外两条路的中间节点相交而成
for nodeId in nodeMap:
	if(nodeMap[nodeId].degree == 5):
		print(nodeId)
'''

'''
for nodeId in nodeMap:
	if(nodeMap[nodeId].degree == 12):
		print(nodeId)
'''
#print( way[0].getElementsByTagName("nd")[0].getAttribute("ref") )


