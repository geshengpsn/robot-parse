use liealg::{se3, SE3};
use nalgebra::Matrix6;
use petgraph::prelude::*;

mod bfs;
mod spatial_inertial;
mod urdf;
mod utils;

#[derive(Debug)]
pub struct Joint {
    pub urdf_joint: Option<urdf_rs::Joint>,
}

#[derive(Debug)]
pub struct Link {
    pub space_spatial_screw: se3<f64>,
    pub local_spatial_screw: se3<f64>,

    pub global_zero_pose: SE3<f64>,

    // zero pose relative to parent link
    pub parent_zero_pose: SE3<f64>,

    // link frame spatial inertia
    pub local_spatial_inertial: Matrix6<f64>,

    // joint
    pub joint: Joint,
    // original urdf link
    // urdf_link: urdf_rs::Link,
}

#[derive(Debug)]
pub struct Model {
    // rigid body tree
    pub links: Vec<Link>,
    pub link_graph: DiGraphMap<usize, ()>,
    pub bfs: Vec<usize>,
}
