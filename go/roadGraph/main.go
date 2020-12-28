package main

import (
	"ArcRoad"
	"github.com/EngoEngine/glm"
	"fmt"
)

func main(){
	points := []glm.Vec2{glm.Vec2{0.0, 0.0}, glm.Vec2{1.0, 1.0}, glm.Vec2{3.0,1.0}, glm.Vec2{4.0, 2.0} }
	outPoints := ArcRoad.GeneratePoints(points)
	outPoints3D := ArcRoad.ConstuctOrdinaryBridge(outPoints, 0, 10 ,0.1)
	fmt.Printf("%d", len(outPoints3D) )

	
	for i:= 0; i < len(outPoints3D); i++{
		fmt.Printf("%s", outPoints3D[i].String())
	}

}