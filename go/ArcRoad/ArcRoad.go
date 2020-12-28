package ArcRoad

import (
	"math"
	"github.com/EngoEngine/glm"
	//"fmt"
)


func alpha_assign(A []float64, s int, e int, L []float64, f []float64 ){
	r_min := float64(1<<30)
	i_min := e
	alpha_low := 0.0
	alpha_high := 0.0
	if(s+1 >= e){
		return;
	}
	alpha_b := math.Min(L[s]-A[s], L[s+1])
	r_current := math.Max(f[s] * A[s], f[s+1] * alpha_b)
	if(r_current < r_min){
		r_min = r_current
		i_min = s
		alpha_low = A[s]
		alpha_high = alpha_b
	}

	for i := s+1; i <= e-2; i++{
		alpha_a := math.Min(L[i-1], L[i] * f[i]/(f[i]+f[i+1]))
		alpha_b := math.Min(L[i+1], L[i] - alpha_a)
		r_current := math.Max(f[i]*alpha_a, f[i+1] * alpha_b)
		if(r_current < r_min){
			r_min = r_current
			i_min = i
			alpha_low = alpha_a
			alpha_high = alpha_b
		}
	} 

	alpha_a := math.Min(L[e-2], L[e-1]-A[e])
	r_current = math.Max(f[e-1]*alpha_a, f[e]*A[e])
	if(r_current < r_min){
		r_min = r_current
		i_min = e-1
		alpha_low = alpha_a
		alpha_high = A[e]
	}
	A[i_min] = alpha_low
	A[i_min+1] = alpha_high
	alpha_assign(A, s, i_min, L, f)
	alpha_assign(A, i_min+1, e, L, f)
}

func generateArcPoints(i int, dis float64, points []glm.Vec2)([]glm.Vec2){
	v1 := (&points[i]).Sub(&points[i-1])
	n1 := (&v1).Normalized()
	v2 := (&points[i+1]).Sub(&points[i])
	n2 := (&v2).Normalized()
	tmpB := (&n2).Sub(&n1)
	b := (&tmpB).Normalized()
	beishu := dis * math.Sqrt( float64((1+n1.Dot(&n2))/(1-n1.Dot(&n2))) + 1.0)
	//fmt.Println(beishu)
	//fmt.Println(dis)
	tmp1 := ((&b).Mul( float32(beishu)))
	tmp2 := n1.Mul( float32(-dis))
	tmp3 := n2.Mul( float32(dis))
	O := (&points[i]).Add( &tmp1 )
	M := (&points[i]).Add( &tmp2  )
	N := (&points[i]).Add( &tmp3 )
	OM := (&M).Sub(&O)
	ON := (&N).Sub(&O)
	//fmt.Println("OM: %s, ON: %s", OM.String(), ON.String())
	angle := math.Acos( float64((&OM).Dot(&ON)/(OM.Len()*ON.Len())) )
	newX := float64(OM[0])*math.Cos(angle) - float64(OM[1])*math.Sin(angle)
	newY := float64(OM[0])*math.Sin(angle) + float64(OM[1])*math.Cos(angle)
	var roateFlag bool
	if(math.Abs(newX- float64(ON[0])) < 0.01 && math.Abs(newY-float64(ON[1])) < 0.01){
		roateFlag = true
	}else{
		roateFlag = false
	}
	start := OM
	roateAngle := 0.0
	var tmp float64
	step := 0.01
	outPoints := []glm.Vec2{}
	//fmt.Println("angle %f", angle)
	for roateAngle <= angle{
		tmp = roateAngle
		if(roateFlag == false){
			roateAngle = -roateAngle
		}
		x1 := start[0]* float32(math.Cos(roateAngle)) - start[1]* float32(math.Sin(roateAngle))
		y1 := start[0]* float32(math.Sin(roateAngle)) + start[1]* float32(math.Cos(roateAngle))
		outPoints = append(outPoints, (&glm.Vec2{x1,y1}).Add(&O) )
		roateAngle = tmp
		roateAngle += step
	}

	return outPoints
}

func filterPoints(inputPoints []glm.Vec2) ([]glm.Vec2){
	cos_thres := math.Cos( float64(glm.DegToRad(179.0)))
	outPoints := []glm.Vec2{} 
	if( len(inputPoints) != 2){
		outPoints = append(outPoints, inputPoints[0])
		pre := 0
		for i := 1; i < len(inputPoints)-1; i++{
			l1 := glm.Vec2{inputPoints[pre][0]-inputPoints[i][0], inputPoints[pre][1]-inputPoints[i][1]}
			l2 := glm.Vec2{inputPoints[i+1][0]-inputPoints[i][0], inputPoints[i+1][1]-inputPoints[i][1]}
			if(l1.Len() < 0.01 || l2.Len() < 0.01){
				continue;
			}

			l1.Normalize()
			l2.Normalize()
			if( float64(l1.Dot(&l2)) > cos_thres){
				outPoints = append(outPoints, inputPoints[i])
				pre = i
			}
		}

		outPoints = append(outPoints, inputPoints[len(inputPoints)-1])
	}else{
		outPoints = append(outPoints, inputPoints[0])
		outPoints = append(outPoints, inputPoints[1])
	}

	return outPoints
}


func Init(L []float64, f []float64, points []glm.Vec2)([]float64, []float64){
	for i := 1; i < len(points) - 1; i++{
		if(i == 1){
			f = append(f, 0)
		}
		v1 := points[i].Sub( &points[i-1] )
		v2 := points[i+1].Sub( &points[i] )
		n1 := v1.Normalized()
		n2 := v2.Normalized()
		L = append(L, float64(v1.Len()) )
		if(i == len(points)-2){
			L = append(L, float64(v2.Len()) )
		}
		f = append(f, math.Sqrt( float64((1+n1.Dot(&n2))/(1-n1.Dot(&n2))) ))
		if(i == len(points)-2){
			f = append(f, 0.0)
		}
	}
	return L, f
}

func distance(A *glm.Vec2, B *glm.Vec2) float64 {
	C := A.Sub(B)
	return  float64( (&C).Len())
}


func GeneratePoints(points []glm.Vec2) ([]glm.Vec2){
	if(len(points) == 2){
		if( distance(&points[0], &points[1]) < 0.01){
			return points;
		}
	}
	points = filterPoints(points)

	n := len(points)
	A := make([]float64, n)
	L := []float64{}
	f := []float64{}
	L, f = Init(L, f, points)
	alpha_assign(A, 0, n-1, L, f)
	outPoints := []glm.Vec2{}

	outPoints = append(outPoints, points[0])
	for i := 1; i < n - 1; i++{
		x := generateArcPoints(i, A[i], points)
		outPoints = append(outPoints, x...)
	} 
	outPoints = append(outPoints, points[n-1])
	//fmt.Println(A)
	return outPoints
}