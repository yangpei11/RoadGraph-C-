class Point:
	def __init__(self, degree, pointId, x, y):
		self.x = x
		self.y = y
		self.degree = degree
		self.pointId = pointId
		self.adjVertex = []

class Edge:
	def __init__(self):
		self.points = []
	def addPoint(self, pointId):
		self.points.append(pointId)