var glm = require("glm-js")

class CublicSpline
{
	static ConstuctOrdinaryBridge(points2d, h1, h2, step)
	{
		let outPoints3d = []
		let length = 0
		for(let i = 1; i < points2d.length; i++)
		{
			length += glm.distance(points2d[i-1], points2d[i])
		}

		let x1 = 0, y1 = h1, x2 = length, y2 = h2
		let a = y1, b = 0, c = 3*(y2-y1)/((x2-x1)*(x2-x1))
		let d = -2 * (y2-y1)/( (x2-x1)*(x2-x1)*(x2-x1) )
		let addLength = 0

		let addStep = step 
		for(let i = 1; i < points2d.length; i++)
		{
			let gapLength = glm.distance(points2d[i - 1], points2d[i])
			let futureLength = addLength + gapLength
			let n = glm.normalize(points2d[i].sub(points2d[i - 1]))
			for(let dis = 0; dis < gapLength; dis += addStep)
			{
				let localPos = points2d[i - 1].add( glm.vec2(n.x*dis, n.y*dis) )
				let insertHeight = a + c * (addLength - x1) * (addLength - x1) + d * (addLength - x1) * (addLength - x1) * (addLength - x1)
				outPoints3d.push( glm.vec3(localPos.x, localPos.y, insertHeight) )
				addLength += addStep
			}
			addLength = futureLength
		}
		let lastIndex = points2d.length - 1
		outPoints3d.push( glm.vec3(points2d[lastIndex].x, points2d[lastIndex].y, h2))
		return outPoints3d
	}

	static ConstuctArchBridge(points2d, h1, h2, midPointHeight, step)
	{
		let outPoints3d = []
		let length = 0
		for(let i = 1; i < points2d.length; i++)
		{
			length += glm.distance(points2d[i-1], points2d[i])
		}
		let half_len = length / 2.0

		// 三次样条方程
	    //当addLength < length/2.0时
		let x1 = 0, y1 = h1, x2 = half_len, y2 = midPointHeight
		let a = y1, b = 0, c = 3 * (y2 - y1) / ((x2 - x1) * (x2 - x1))
		let d = -2 * (y2 - y1) / ((x2 - x1) * (x2 - x1) * (x2 - x1))

		//当addLength > length/2.0时
		let m1 = 0, n1 = midPointHeight, m2 = length / 2.0, n2 = h2
		let a1 = n1, b1 = 0, c1 = 3 * (n2 - n1) / ((m2 - m1) * (m2 - m1))
		let d1 = -2 * (n2 - n1) / ((m2 - m1) * (m2 - m1) * (m2 - m1))

		let is_first_half = true // 是否前半段
		let addLength = 0

		//迭代步长
		let addStep = step
		for (let i = 1; i < points2d.length; ++i) {
			let gapLength = glm.distance(points2d[i - 1], points2d[i])
			let futureLength = addLength + gapLength
			let  n = glm.normalize(points2d[i].sub(points2d[i - 1]))
			for (let dis= 0; dis < gapLength; dis += addStep) {
				if (addLength > half_len && is_first_half) {
					is_first_half = false //改为后半段
				}
				let localPos = points2d[i - 1].add(glm.vec2(n.x*dis, n.y*dis))

				let insertHeight
				if (is_first_half) {
					insertHeight = a + c * (addLength - x1) * (addLength - x1) +
						d * (addLength - x1) * (addLength - x1) * (addLength - x1)
				}
				else {
					insertHeight = a1 +
						c1 * (addLength - half_len - m1) * (addLength - half_len - m1) +
						d1 * (addLength - half_len - m1) * (addLength - half_len - m1) *
						(addLength - half_len - m1)
				}
				outPoints3d.push( glm.vec3(localPos.x, localPos.y, insertHeight) )
				addLength += addStep;
			}
			addLength = futureLength;
		}
		let lastIndex = points2d.length - 1
		outPoints3d.push( glm.vec3(points2d[lastIndex].x, points2d[lastIndex].y, h2))
		return outPoints3d
	}
}

module.exports = CublicSpline