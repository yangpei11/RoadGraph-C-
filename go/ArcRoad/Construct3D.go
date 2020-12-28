package ArcRoad

import (
	//"math"
	"github.com/EngoEngine/glm"
	//"fmt"
)


func ConstuctOrdinaryBridge(points2d []glm.Vec2, h1 float64, h2 float64, step float64)([]glm.Vec3){
	outPoints3d := []glm.Vec3{}
	length := 0.0
	for i := 1; i < len(points2d); i++{
		length += distance(&points2d[i-1], &points2d[i])
	}

	x1 := 0.0
	y1 := h1
	x2 := length
	y2 := h2
	a := y1
	//b := 0.0
	c := 3*(y2-y1)/((x2-x1)*(x2-x1))
	d := -2*(y2-y1)/((x2 - x1) * (x2 - x1) * (x2 - x1))
	addLength := 0.0
	addStep := step
	for i := 1; i < len(points2d);i++{
		gapLength := distance(&points2d[i-1], &points2d[i])
		futureLength := addLength + float64(gapLength)
		tmp := (&points2d[i]).Sub(&points2d[i-1])
		n := (&tmp).Normalized()
		for dis := 0.0; dis < gapLength; dis += addStep{
			tmp1 := (&n).Mul(float32(dis))
			localPos :=  (&points2d[i-1]).Add(&tmp1)
			insertHeight := a + c * (addLength - x1) * (addLength - x1) + d * (addLength - x1) * (addLength - x1) * (addLength - x1)
			outPoints3d = append(outPoints3d, glm.Vec3{localPos[0], localPos[1], float32(insertHeight)} )
			addLength += addStep
		} 
		addLength = futureLength
	}

	outPoints3d = append(outPoints3d, glm.Vec3{points2d[len(points2d)-1][0], points2d[len(points2d)-1][1], float32(h2)})
	return outPoints3d
}

func ConstuctArchBridge(points2d []glm.Vec2, h1 float64, h2 float64, midPointHeight float64, step float64)([]glm.Vec3){
	outPoints3d := []glm.Vec3{}
	length := 0.0
	for i := 1; i < len(points2d); i++{
		length += distance(&points2d[i-1], &points2d[i])
	}
	half_len := length/2.0

	x1 := 0.0
	y1 := h1
	x2 := half_len
	y2 := midPointHeight
	a := y1
	//b := 0.0
	c := 3*(y2-y1)/((x2-x1)*(x2-x1))
	d := -2*(y2-y1)/((x2 - x1) * (x2 - x1) * (x2 - x1))

 	m1 := 0.0
 	n1 := midPointHeight
 	m2 := length/2.0
 	n2 := h2
 	a1 := n1
 	//b1 := 0.0
 	c1 := 3 * (n2 - n1) / ((m2 - m1) * (m2 - m1))
 	d1 := -2 * (n2 - n1) / ((m2 - m1) * (m2 - m1) * (m2 - m1))

 	is_first_half := true
 	addLength := 0.0
 	addStep := step

 	for i := 1; i < len(points2d); i++{
		gapLength := distance(&points2d[i-1], &points2d[i])
		futureLength := addLength + float64(gapLength)
		tmp := (&points2d[i]).Sub(&points2d[i-1])
		n := (&tmp).Normalized()
		for dis := 0.0; dis < gapLength; dis += addStep{
			if(addLength > half_len && is_first_half){
				is_first_half = false
			}
			tmp1 := (&n).Mul(float32(dis))
			localPos :=  (&points2d[i-1]).Add(&tmp1)
			var insertHeight float64
			if(is_first_half){
				insertHeight = a + c * (addLength - x1) * (addLength - x1) + d * (addLength - x1) * (addLength - x1) * (addLength - x1)
			}else{
				insertHeight = a1 + c1 * (addLength - half_len - m1) * (addLength - half_len - m1) +d1 * (addLength - half_len - m1) * (addLength - half_len - m1) *(addLength - half_len - m1)
			}
			outPoints3d = append(outPoints3d, glm.Vec3{localPos[0], localPos[1], float32(insertHeight)} )
			addLength += addStep
		} 
		addLength = futureLength
	}

	outPoints3d = append(outPoints3d, glm.Vec3{points2d[len(points2d)-1][0], points2d[len(points2d)-1][1], float32(h2)})
	return outPoints3d
}