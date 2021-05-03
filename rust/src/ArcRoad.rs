use glm::int_bits_to_float_vec;
use std::cmp;

extern crate nalgebra_glm as glm;
pub struct CArcRoad
{
    pub points: Vec<glm::Vec2>,
}

impl CArcRoad
{
    pub fn New() -> CArcRoad
    {
        CArcRoad{
            points:Vec::new(),
        }
    }
    //过滤三点一线的中间点
    pub fn filter_points(&mut self, inputPoins:Vec<glm::Vec2>)
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
    
}