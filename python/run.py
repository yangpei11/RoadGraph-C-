from MPT import Road
from cros import Intersection, create_uid
from graph import Point, Edge
import xml.dom.minidom as xmldom 
import matplotlib.pyplot as plt


#id->point
pointMap = dict()
domObj = xmldom.parse("simple.osm")
elementObj = domObj.documentElement
nodes = elementObj.getElementsByTagName("node")
for node in nodes:
	nodeId = node.getAttribute("id")
	x = node.getAttribute("x")
	y = node.getAttribute("y")
	point = Point(0, nodeId, x, y)
	pointMap[nodeId] = point

ways = elementObj.getElementsByTagName("way")
for way in ways:
	points  = way.getElementsByTagName("nd")
	edge = Edge()
	xx =[]
	yy =[]
	for index in range(0, len(points)):
		pointId = points[index].getAttribute("ref")
		edge.addPoint(pointId)
		xx.append( float(pointMap[pointId].x))
		yy.append( float(pointMap[pointId].y) )
	#plt.plot(xx, yy, color = 'r')

	for i in range(0, len(edge.points)):
		if(i == 0):
			pointMap[ edge.points[i] ].adjVertex.append(edge.points[i+1])
			pointMap[ edge.points[i] ].degree += 1
		elif(i == len(edge.points) - 1):
			pointMap[ edge.points[i] ].adjVertex.append(edge.points[i-1])
			pointMap[ edge.points[i] ].degree += 1
		else:
			pointMap[ edge.points[i] ].adjVertex.append(edge.points[i-1])
			pointMap[ edge.points[i] ].adjVertex.append(edge.points[i+1])
			pointMap[ edge.points[i] ].degree += 2
#plt.show()

oneDegreePoints = []
for pointId in list(pointMap.keys()):
	if(pointMap[pointId].degree == 1):
		oneDegreePoints.append(pointId)
	elif(pointMap[pointId].degree > 2):
		xx = []
		yy = []
		pointsId = []
		for i in range(0, len(pointMap[pointId].adjVertex)):
			if( pointMap[pointMap[pointId].adjVertex[i] ].degree > 2):	
				x = (float(pointMap[pointMap[pointId].adjVertex[i] ].x) + float(pointMap[pointId].x) )/2
				y = (float(pointMap[pointMap[pointId].adjVertex[i] ].y) + float(pointMap[pointId].y) )/2
				intersectionPoint = Point(2, create_uid(), x, y)
				intersectionPoint.adjVertex.append( pointId )
				intersectionPoint.adjVertex.append( pointMap[pointId].adjVertex[i] )
				temp = pointMap[pointId].adjVertex[i]
				pointMap[pointId].adjVertex[i] = intersectionPoint.pointId
				for j in range(0, len(pointMap[temp].adjVertex)):
					if(pointId == pointMap[temp].adjVertex[j]):
						pointMap[temp].adjVertex[j] = intersectionPoint.pointId 
						break
				pointMap[intersectionPoint.pointId] = intersectionPoint
				xx.append( intersectionPoint.x )
				yy.append( intersectionPoint.y )
				pointsId.append(intersectionPoint.pointId)
			else:	
				xx.append( float(pointMap[pointMap[pointId].adjVertex[i] ].x) )
				yy.append( float(pointMap[pointMap[pointId].adjVertex[i] ].y) )
				pointsId.append(pointMap[pointId].adjVertex[i])
		'''
		print(xx)
		print(yy)
		print(pointsId)
		print(">>>>>>>>>>>>>>")
		'''
		intersection = Intersection( float(pointMap[pointId].x), float(pointMap[pointId].y), xx, yy, pointsId, pointMap, pointId, oneDegreePoints)
		intersection.plot(0.5,0.7)
def dfs(pointId, visited, xx, yy):
	if(visited.get(pointId) != None):
		return
	visited[pointId] = True
	xx.append( float(pointMap[pointId].x) )
	yy.append( float(pointMap[pointId].y) )
	for nextPointId in pointMap[pointId].adjVertex:
		if(visited.get(nextPointId) == None):
			dfs(nextPointId, visited, xx, yy)
			break

'''
for ID in oneDegreePoints:
	print( ID + " " + str(pointMap[ID].x) + " " + str(pointMap[ID].y))
print("\n\n\n\n")
print(1/0)
'''

visited = dict()
for pointId in oneDegreePoints:
	xx = []
	yy = []
	#print(pointId)

	dfs(pointId, visited, xx, yy)
	if(len(xx) < 2):
		continue
	plt.plot(xx, yy, color = "r")
	road = Road(xx, yy)
	road.plot()

plt.xlim(0,15)
plt.ylim(-3,15)
plt.gca().set_aspect('equal',adjustable = 'box')

plt.show()






