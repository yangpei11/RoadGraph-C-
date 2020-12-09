var ArcRoad = require("./ArcRoad.class.js")
var CublicSpline = require("./CublicSpline.js")
a = [glm.vec2(0,0), glm.vec2(1,1), glm.vec2(3,1), glm.vec2(4,2)]
var Road = new ArcRoad(a)
var outPoints = Road.generatePoints()
var outPoints3D = CublicSpline.ConstuctOrdinaryBridge(outPoints, 0, 10, 0.1)