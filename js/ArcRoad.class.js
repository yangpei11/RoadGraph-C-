var glm = require("glm-js")


class ArcRoad
{
	#points = []
	#outPoints = []
	constructor(inputPoints)
	{
		let cos_thres = Math.cos(glm.radians(179.0)) //glm不提供cos，sin
		if(inputPoints.length != 2)
		{
			this.#points.push(inputPoints[0])
			let pre = 0;
			for(let i = 1; i < inputPoints.length-1; i++)
			{
				let l1 = inputPoints[pre].sub(inputPoints[i])
				let l2 = inputPoints[i+1].sub(inputPoints[i])
				if(glm.length(l1) < 0.01 || glm.length(l2) < 0.01)
				{
					continue;
				}
				l1 = glm.normalize(l1)
				l2 = glm.normalize(l2)

				//去掉共线的点
				if(glm.dot(l1, l2) > cos_thres)
				{
					this.#points.push(inputPoints[i])
					pre = i
				}
			}

			this.#points.push(inputPoints[inputPoints.length-1] )
		}
		else
		{
			this.#points.push(inputPoints[0])
			this.#points.push(inputPoints[inputPoints.length-1])
		}
	}

	getPointsSize()
	{
		return this.#points.length
	}

	Init(L, f)
	{
		for(let i = 1; i < this.#points.length-1; i++)
		{
			if(i == 1)
			{
				f.push(0)
			}
			let v1 = this.#points[i].sub(this.#points[i-1])
			let v2 = this.#points[i+1].sub(this.#points[i])
			let n1 = glm.normalize(v1)
			let n2 = glm.normalize(v2)

			L.push(glm.length(v1))
			if(i == this.#points.length - 2)
			{
				L.push(glm.length(v2))
			}
			f.push(Math.sqrt((1+glm.dot(n1,n2))/(1-glm.dot(n1,n2))))
			if(i == this.#points.length-2)
			{
				f.push(0)
			}
		}
	}

	alpha_assign(A, s, e, L, f)
	{
		let r_min = 1<<30
		let i_min = e
		let alpha_low = 0
		let alpha_high = 0
		if(s+1 >= e)
		{
			return;
		}
		let alpha_b = Math.min(L[s]-A[s], L[s+1])
		let r_current = Math.max(f[s] * A[s], f[s+1]*alpha_b)
		if(r_current < r_min)
		{
			r_min = r_current
			i_min = s 
			alpha_low = A[s]
			alpha_high = alpha_b
		}

		for(let i = s+1; i <= e-2; i++)
		{
			let alpha_a = Math.min(L[i-1], L[i]*f[i]/(f[i]+f[i+1]))
			let alpha_b = Math.min(L[i+1], L[i]-alpha_a)
			let r_current = Math.max(f[i]*alpha_a, f[i+1]*alpha_b)
			if(r_current < r_min)
			{
				r_min = r_current
				i_min = i
				alpha_low = alpha_a
				alpha_high = alpha_b
			}
		}

		let alpha_a = Math.min(L[e-2], L[e-1]-A[e])
		r_current = Math.max(f[e-1]*alpha_a, f[e]*A[e])
		if(r_current < r_min)
		{
			r_min = r_current
			i_min = e - 1
			alpha_low = alpha_a
			alpha_high = A[e]
		}

		A[i_min] = alpha_low
		A[i_min+1] = alpha_high
		this.alpha_assign(A, s, i_min, L, f)
		this.alpha_assign(A, i_min+1, e, L, f)
	}

	generateArcPoints(i, dis)
	{
		let n1 = glm.normalize(this.#points[i].sub(this.#points[i-1]))
		let n2 = glm.normalize(this.#points[i+1].sub(this.#points[i]))
		let b = glm.normalize(n2.sub(n1))
		let beishu = dis*Math.sqrt( (1+glm.dot(n1,n2))/(1-glm.dot(n1,n2))+1)
		let O = this.#points[i].add( glm.vec2(b.x*beishu, b.y*beishu) )
		let M = this.#points[i].add( glm.vec2(n1.x*(-dis), n1.y*(-dis)))
		let N = this.#points[i].add( glm.vec2(n2.x*dis, n2.y*dis) )
		let OM = M.sub(O)
		let ON = N.sub(O)
		let start = OM 
		let angle = Math.acos(glm.dot(OM, ON)/(glm.length(OM)*glm.length(ON)))
		let newX = OM.x*Math.cos(angle)-OM.y*Math.sin(angle)
		let newY = OM.x*Math.sin(angle)+OM.y*Math.cos(angle)
		let roateFlag
		if( Math.abs(newX-ON.x) < 0.01 && Math.abs(newY-ON.y) < 0.01)
		{
			roateFlag = true
		}
		else
		{
			roateFlag = false
		}
		let start1 = OM 
		let roateAngle = 0.0
		let tmp
		let step = 0.01 
		//double step = 5.0 / VPE::length(OM); 步长为5m
		while(roateAngle <= angle)
		{
			tmp = roateAngle
			if(roateFlag == 0)
			{
				roateAngle = -roateAngle
			}
			let x1 = start1.x*Math.cos(roateAngle) - start1.y*Math.sin(roateAngle)
			let y1 = start1.x*Math.sin(roateAngle) + start1.y*Math.cos(roateAngle)
			this.#outPoints.push( glm.vec2(x1,y1).add(O) )
			roateAngle = tmp 
			roateAngle += step 
		}
	}

	generatePoints()
	{
		if(this.#points.length == 2)
		{
			if(glm.distance(this.#points[0], this.#points[1]) <0.01)
			{
				return this.#outPoints
			}
			this.#outPoints.push(this.#points[0])
			this.#outPoints.push(this.#points[1])
		}
		else
		{
			let n = this.getPointsSize()
			let L = []
			let f = []
			let A = Array(n).fill(0)
			this.Init(L, f)
			this.alpha_assign(A, 0, n-1, L, f)
			this.#outPoints.push(this.#points[0])
			for(let i = 1; i < n-1; i++)
			{
				this.generateArcPoints(i, A[i])
			}
			this.#outPoints.push(this.#points[n-1])
		}
		return this.#outPoints
	}

}

module.exports = ArcRoad