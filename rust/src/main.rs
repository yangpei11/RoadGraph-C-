use std::io;

mod ArcRoad;
use ArcRoad::CArcRoad;
mod CublicSpline;
use CublicSpline::CCublicSpline;

extern crate nalgebra_glm as glm;

fn main() {
    let mut object = CArcRoad::New();
    let mut arr:Vec<glm::Vec2> = vec![];
    //arr.push(1);
    //arr.push(2);
    //glm::vec2是函数而非类型
    arr.push(glm::vec2(0.0, 0.0));
    arr.push(glm::vec2(1.0, 1.0));
    arr.push(glm::vec2(3.0, 1.0));
    arr.push(glm::vec2(4.0, 2.0));
    let points2d = object.generatePoints(&arr);
    //创建高度为10m的拱桥
    let points3d  = CCublicSpline::ConstuctArchBridge(points2d, 0.0, 0.0, 10.0, 0.1);
    //object.filter_points(arr);
    
    //UnitTest
    let mut arr1:Vec<f32> = vec![];
    //println!("{}", arr1.len());
    object.TestRef(&mut arr1);
    //println!("{}", arr1.len());
    object.TestCalc();
}


