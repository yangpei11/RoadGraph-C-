use std::io;

mod ArcRoad;
use ArcRoad::CArcRoad;
extern crate nalgebra_glm as glm;

fn main() {
    let mut object = CArcRoad::New();
    let mut arr:Vec<glm::Vec2> = vec![];
    //arr.push(1);
    //arr.push(2);
    arr.push(glm::vec2(2.0, 1.0));
    object.filter_points(arr);
    
    //
    let mut arr1:Vec<f32> = vec![];
    //println!("{}", arr1.len());
    object.TestRef(&mut arr1);
    println!("{}", arr1.len());
}


