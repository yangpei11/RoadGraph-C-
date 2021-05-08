//三次样条技术求三维道路的高度
extern crate nalgebra_glm as glm;

pub struct CCublicSpline
{

}

impl CCublicSpline
{
    pub fn New() -> CCublicSpline
    {
        CCublicSpline{
        }
    }
    /* 构造普通的上升或者下降的桥梁
	 * @param  points2d 二维化的点
	 * @param  h1，h2为两端的高度
	 * @param  step 迭代需求，根据自己算法选择合适的步长。
	 * @return 三维化的点
	 */
    pub fn ConstuctOrdinaryBridge(points2d:&Vec<glm::Vec2>, h1:f32, h2:f32, step:f32) -> Vec<glm::Vec3>
    {
        let mut outPoints3d:Vec<glm::Vec3> = vec![];
        let mut length:f32 = 0.0;

        for i in 1..points2d.len()
        {
            length = length + glm::distance(&points2d[i-1], &points2d[i]);
        }
        let x1:f32 = 0.0;
        let y1 = h1;
        let x2 = length;
        let y2 = h2;

        let a = y1;
        let b:f32 = 0.0;
        let c:f32 = 3.0 * (y2-y1)/((x2-x1)*(x2-x1));
        let d:f32 = -2.0 * (y2-y1)/((x2-x1)*(x2-x1)*(x2-x1));
        let mut addLength:f32 = 0.0;

        let addStep = step;
        for i in 1..points2d.len()
        {
            let mut gapLength = glm::distance(&points2d[i-1], &points2d[i]);
            let mut futureLength = addLength + gapLength;
            let n = glm::normalize(&(points2d[i]-points2d[i-1]));
            let mut dis:f32 = 0.0;
            while dis < gapLength
            {
                let pos = points2d[i-1] + n * dis;
                let insertHeight = a + c * (addLength-x1)*(addLength-x1) + d * (addLength-x1)*(addLength-x1)*(addLength-x1);
                outPoints3d.push(glm::vec3(pos[0], pos[1], insertHeight));
                addLength += addStep;
                dis += addStep;
            }
            addLength = futureLength;
        }
        
        outPoints3d.push(glm::vec3(points2d[points2d.len()-1][0], points2d[points2d.len()-1][1], h2));
        return outPoints3d;
    }

    /* 构造拱桥
	 * @param  points2d 二维化的点
	 * @param  h1，h2为两端的高度， midPointHeight为拱桥最高点高度
	 * @param  step 迭代需求，根据自己算法选择合适的步长。
	 * @return 三维化的点
	 */
     pub fn ConstuctArchBridge(points2d:&Vec<glm::Vec2>, h1:f32, h2:f32, midPointHeight:f32, step:f32) -> Vec<glm::Vec3>
     {
        let mut outPoints3d:Vec<glm::Vec3> = vec![];
        let mut length:f32 = 0.0;

        for i in 1..points2d.len()
        {
            length = length + glm::distance(&points2d[i-1], &points2d[i]);
        }

        let half_len:f32 = length/2.0;

        //三次样条方程
        //前半段 addLength < length/2.0时
        let x1:f32 = 0.0;
        let y1 = h1;
        let x2 = half_len;
        let y2 = midPointHeight;
        let a = y1;
        let b:f32 = 0.0;
        let c:f32 = 3.0 * (y2-y1)/((x2-x1)*(x2-x1));
        let d:f32 = -2.0 * (y2-y1)/((x2-x1)*(x2-x1)*(x2-x1));

        //当addLength > length/2.0时
        let m1:f32 = 0.0;
        let n1 = midPointHeight;
        let m2 = length/2.0;
        let n2 = h2;
        let a1 = n1;
        let b1:f32 = 0.0;
        let c1 = 3.0 * (n2 - n1) / ((m2 - m1) * (m2 - m1));
		let d1 = -2.0 * (n2 - n1) / ((m2 - m1) * (m2 - m1) * (m2 - m1));

        let mut is_first_half:bool = true;
        let mut addLength:f32 = 0.0;
        //迭代步长
        let addStep:f32 = step;

        for i in 1..points2d.len()
        {
            let gapLength = glm::distance(&points2d[i-1], &points2d[i]);
            let futureLength = addLength + gapLength;
            let n = glm::normalize(&(points2d[i]-points2d[i-1]));
            let mut dis:f32 = 0.0;
            while dis < gapLength
            {
                if(addLength > half_len && is_first_half)
                {
                    is_first_half = false;
                }
                let pos = points2d[i-1] + n * dis;
                let insertHeight:f32;
                if(is_first_half)
                {
                    insertHeight = a + c * (addLength-x1)*(addLength-x1) + d * (addLength-x1)*(addLength-x1)*(addLength-x1);
                }
                else
                {
                    insertHeight = a1 + c1 * (addLength-half_len-m1)*(addLength-half_len-m1) + d1 * (addLength-half_len-m1)*(addLength-half_len-m1)*(addLength-half_len-m1);
                }
                //insertHeight = a + c * (addLength-x1)*(addLength-x1) + d * (addLength-x1)*(addLength-x1)*(addLength-x1);
                outPoints3d.push(glm::vec3(pos[0], pos[1], insertHeight));
                addLength += addStep;
                dis += addStep;
            }
            addLength = futureLength;
        }
        outPoints3d.push(glm::vec3(points2d[points2d.len()-1][0], points2d[points2d.len()-1][1], h2));
        return outPoints3d;
     }
}