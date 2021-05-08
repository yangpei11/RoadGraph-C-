use glm::{abs, int_bits_to_float_vec};

extern crate nalgebra_glm as glm;
pub struct CArcRoad
{
    pub points: Vec<glm::Vec2>,
    pub out_points:Vec<glm::Vec2>,
}

impl CArcRoad
{
    pub fn New() -> CArcRoad
    {
        CArcRoad{
            points:Vec::new(),
            out_points:Vec::new(),
        }
    }
    //过滤三点一线的中间点
    pub fn filter_points(&mut self, inputPoins:&Vec<glm::Vec2>)
    {
        let angle  = glm::radians(&glm::vec1(179.0));
        let cos_thres = glm::cos(&angle);
        let pointsLen = inputPoins.len();
        if pointsLen != 2
        {
            self.points.push(inputPoins[0]);
            let mut pre = 0;
            let mut i = 1;
            while i < pointsLen-1
            {
                let l1 = inputPoins[pre]-inputPoins[i];
                let l2 = inputPoins[i+1]-inputPoins[i];
                if glm::length(&l1) < 0.01 || glm::length(&l2) < 0.01
                {
                    i += 1;
                    continue;
                }

                let l1 = glm::normalize(&l1);
                let l2 = glm::normalize(&l2);
                if glm::dot(&l1, &l2) > cos_thres[0]
                {
                    self.points.push(inputPoins[i]);
                    pre = i;
                }
                i += 1;
            }

            self.points.push(inputPoins[pointsLen-1]);
        }
        else
        {
            self.points.push(inputPoins[0]);
            self.points.push(inputPoins[pointsLen-1]);
        }
    }

    //ArcRoad算法初始化L和f，传引用
    pub fn Init(&mut self, L:&mut Vec<f32>, f:&mut Vec<f32>)
    {
        let mut i = 1;
        let pointsLen = self.points.len();
        while i < pointsLen
        {
            if i == 1
            {
                f.push(0.0);
            }
            let v1 = self.points[i] - self.points[i-1];
            let v2 = self.points[i+1] - self.points[i];
            let n1 = glm::normalize(&v1);
            let n2 = glm::normalize(&v2);
            L.push(glm::length(&v1));
            if i == pointsLen - 2
            {
                L.push(glm::length(&v2));
            }
            let x = 1.0+glm::dot(&n1,&n2);
            let y = 1.0-glm::dot(&n1,&n2);
            let val = glm::sqrt(&glm::vec1(x/y));
            f.push(val[0]);

            if i == pointsLen -2
            {
                f.push(0.0);
            }
            i += 1;
        }
    }

    pub fn TestRef(&mut self, L:&mut Vec<f32>)
    {
        L.push(1.0);
    }

    //Unit Test
    pub fn TestCalc(&mut self)
    {
        //TestCase1{ glm::vec2 相减, result = [4, 3] }
        let subPoint = glm::vec2(5.0,6.0) - glm::vec2(1.0,3.0);
        println!("point is {}", subPoint);

        //TestCase2{ glm::sqrt result = 2}
        let sqrtNum = glm::sqrt(&subPoint)[0];
        println!("sqrtNum is {}", sqrtNum);

        //TestCase3{ glm::vec2 相乘, result = [8, 6] }
        let mulPoint = subPoint*2.0;
        println!("mulPoint is {}", mulPoint);

        //TestCase4{ glm::sin, result = 4.9, newX = {4.9} }
        let pi:f32 = glm::pi();
        let cosNum = glm::cos(&glm::vec1(pi));
        let OM = glm::vec2(3.0, 4.0);
        let newX = OM[0] * glm::cos(&cosNum) - OM[1] * glm::sin(&cosNum);
        //let newY = OM[0] * glm::sin(&angle) + OM[1] * glm::cos(&angle); 
        println!("cosNum is {}", newX[0]);

        //TestCase5{ abs result = {3.3} }
        let absNum = glm::abs(&glm::vec1(-3.3));
        println!("absNum is {}", absNum);

        //TestCase6{ 测试动态数组固定长度，默认值为0 }
        let arr:Vec<f32> = vec![0.0;10];
        println!("arr begin is {}, end is {}", arr[0], arr[9]);

        //TestCase7{ 测试for }
        for i in 1..10
        {
            print!("{}", i);
        }   

        //TestCase8{ f32 + i32 }
        let num1:f32 = 1.2;
        let num2:i32 = 1;
        let num3 = num1 +  num2 as f32;
    }

    //ArcRoad的核心算法
    pub fn alpha_assign(&mut self, A:&mut Vec<f32>, s:usize, e:usize, L:&mut Vec<f32>, f:&mut Vec<f32>)
    {
        let mut r_min:f32 = (1<<30) as f32;
        let mut i_min = e;
        let mut alpha_low = 0.0;
        let mut alpha_high = 0.0;
        if s + 1 >= e
        {
            return;
        }

        let alpha_b = f32::min(L[s]-A[s], L[s+1]);
        let mut r_current = f32::min(f[s]*A[s], f[s+1]*alpha_b);
        if r_current < r_min 
        {
            r_min = r_current;
            i_min = s;
            alpha_low = A[s];
            alpha_high = alpha_b;
        }

        let i = s + 1;
        while i <= e-2
        {
            let alpha_a = f32::min(L[i-1], L[i]*f[i]/(f[i]+f[i+1]));
            let alpha_b = f32::min(L[i+1], L[i]-alpha_a);
            let r_current = f32::max(f[i]*alpha_a, f[i+1]*alpha_b);
            if(r_current < r_min)
            {
                r_min = r_current;
                i_min = i;
                alpha_low = alpha_a;
                alpha_high = alpha_b;
            }
        }
        let alpha_a = f32::min(L[e-2], L[e-1]-A[e]);
        r_current = f32::max(f[e-1]*alpha_a, f[e]*A[e]);

        if(r_current < r_min)
        {
            r_min = r_current;
            i_min = e-1;
            alpha_low = alpha_a;
            alpha_high = A[e];
        }

        //找到两个最优解
        A[i_min] = alpha_low;
        A[i_min+1] = alpha_high;
        //一分为二，递归求解最优解
        self.alpha_assign(A, s, i_min, L, f);
        self.alpha_assign(A, i_min+1, e, L, f);
    }
    
    //生成每个局部区域的圆弧
    pub fn generateArcPoints(&mut self, i:usize, dis:f32)
    {
        let n1 = glm::normalize( &(self.points[i] - self.points[i-1]) );
        let n2 = glm::normalize( &(self.points[i+1] - self.points[i]) );
        let b = glm::normalize(&(n2-n1));
        let x = 1.0+glm::dot(&n1,&n2);
        let y = 1.0-glm::dot(&n1,&n2);
        let val = glm::sqrt(&glm::vec1(x/y));
        let beishu = dis*val[0];

        let O = self.points[i] + b*beishu;
        let M = self.points[i] + (-dis)*n1;
        let N = self.points[i] + dis * n2;
        let OM = M - O;
        let ON = N - O;
        let start = OM;
        let angle = glm::cos(&glm::vec1(glm::dot(&OM,&ON)/(glm::length(&OM)*glm::length(&ON))));
        let newX = OM[0] * glm::cos(&angle) - OM[1] * glm::sin(&angle);
        let newY = OM[0] * glm::sin(&angle) + OM[1] * glm::cos(&angle); 

        let roateFlag:bool;
        if glm::abs( &glm::vec1(newX[0]-ON[0]) )[0] < 0.01 && glm::abs( &glm::vec1(newY[1]-ON[1]) )[0] < 0.01
        {
            roateFlag = true;
        } 
        else
        {
            roateFlag = false;
        }
        let start1 = OM;
        let mut roateAngle:f32 = 0.0;
        let mut tmp:f32;
        let step:f32 = 0.01;
        while roateAngle <= angle[0]
        {
            tmp = roateAngle;
            if(roateFlag)
            {
                roateAngle = -roateAngle;
            }
            let x1 = start1[0]*glm::cos(&glm::vec1(roateAngle)) - start1[1]*glm::sin(&glm::vec1(roateAngle));
            let y1 = start1[0]*glm::sin(&glm::vec1(roateAngle)) + start1[1]*glm::cos(&glm::vec1(roateAngle));
            self.out_points.push(glm::vec2(x1[0], y1[0]) + O);
            roateAngle = tmp;
            roateAngle += 0.1;
        }
    }

    //生成ArcRoad样条
    pub fn generatePoints(&mut self, inputPoints:&Vec<glm::Vec2>) -> &Vec<glm::Vec2> 
    {
        self.filter_points(&inputPoints);
        if inputPoints.len() == 2
        {
            //let tmp = inputPoints;
            return &self.points;
        }
        else
        {
            let n = inputPoints.len();
            let mut A:Vec<f32> = vec![0.0;n];
            let mut L:Vec<f32> = vec![];
            let mut f:Vec<f32> = vec![];
            self.Init(&mut L, &mut f);
            self.alpha_assign(&mut A, 0, n-1, &mut L, &mut f);
            self.out_points.push(inputPoints[0]);
            for i in 1..n-1
            {
                self.generateArcPoints(i, A[i]);
            }
            self.out_points.push(inputPoints[n-1]);
            return &self.out_points;
        }
    }
}