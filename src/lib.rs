use std::{
    collections::{HashMap},
    path::Path,
};

use liealg::{Algebra, SE3, SO3};
use petgraph::visit::Bfs;
use urdf_rs::{read_file, read_from_string};

pub use urdf_rs;

#[derive(Debug)]
pub struct Link {
    pub joint: Option<urdf_rs::Joint>,
    pub joint_screw: Option<liealg::se3<f64>>,
    pub joint_relative_pose: liealg::SE3<f64>,
    pub joint_value: f64,
    pub global_pose: liealg::SE3<f64>,
    pub urdf_link: urdf_rs::Link,
}

#[derive(Debug)]
pub struct MultiBodyGraph {
    graph: petgraph::graphmap::DiGraphMap<usize, ()>,
    pub link_map: HashMap<usize, Link>,
    pub root_index: usize,
    pub leafs_index: Vec<usize>,
    pub name: String,
    pub material: Vec<urdf_rs::Material>,
}

impl MultiBodyGraph {
    pub fn bfs(&self, start: usize) -> Vec<usize> {
        let bfs = petgraph::visit::Bfs::new(&self.graph, start);
        let iter = BfsIter {
            graph: &self.graph,
            bfs,
        };
        iter.collect()
    }

    pub fn get_link(&self, index: usize) -> Option<&Link> {
        self.link_map.get(&index)
    }

    pub fn get_mut_link(&mut self, index: usize) -> &mut Link {
        self.link_map.get_mut(&index).unwrap()
    }

    pub fn parent(&self, index: usize) -> Option<usize> {
        self.graph
            .neighbors_directed(index, petgraph::Direction::Incoming)
            .next()
    }

    pub fn children(&self, index: usize) -> Vec<usize> {
        self.graph
            .neighbors_directed(index, petgraph::Direction::Outgoing)
            .collect()
    }

    pub fn set_joint_value(&mut self, joint_values: HashMap<String, f64>) {
        for (name, value) in joint_values {
            let link = self
                .link_map
                .values_mut()
                .filter(|link| link.joint.is_some())
                .find(|link| link.joint.as_ref().unwrap().name == name)
                .unwrap();
            link.joint_value = value;
        }
    }

    pub fn move_joint(&mut self) {
        let bfs = self.bfs(self.root_index);

        for link in bfs {
            let parent = self.parent(link);
            if let Some(parent_index) = parent {
                let parent_global_pose = self.get_link(parent_index).unwrap().global_pose.clone();
                let joint_value = self.get_link(link).unwrap().joint_value;
                let relative_pose = relative_pose(
                    self.get_link(link).unwrap().joint.as_ref().unwrap(),
                    joint_value,
                );
                self.get_mut_link(link).global_pose = parent_global_pose * relative_pose;
            }
        }
    }
}

pub struct BfsIter<'a> {
    graph: &'a petgraph::graphmap::DiGraphMap<usize, ()>,
    bfs: Bfs<usize, hashbrown::hash_set::HashSet<usize>>,
}

impl<'a> Iterator for BfsIter<'a> {
    type Item = usize;

    fn next(&mut self) -> Option<Self::Item> {
        self.bfs.next(self.graph)
    }
}

pub fn parse_urdf_string(string: &str) -> Result<MultiBodyGraph, urdf_rs::UrdfError> {
    let robot = read_from_string(string)?;
    Ok(parse_robot(robot))
}

pub fn parse_urdf_file(path: impl AsRef<Path>) -> Result<MultiBodyGraph, urdf_rs::UrdfError> {
    let robot = read_file(path)?;
    Ok(parse_robot(robot))
}

fn parse_robot(robot: urdf_rs::Robot) -> MultiBodyGraph {
    // provide an index for each link
    let link_name_index_map = robot
        .links
        .iter()
        .enumerate()
        .map(|(index, link)| (link.name.clone(), index))
        .collect::<HashMap<_, _>>();
    
    // create joint link
    let joint_links = robot
        .links
        .into_iter()
        .map(|l| {
            let j = robot
                .joints
                .iter()
                .find(|joint| joint.child.link == l.name);
            (j.cloned(), l)
        })
        .collect::<Vec<_>>();

    let root = *joint_links
        .iter()
        .find(|(joint, _)| joint.is_none())
        .map(|(_,link)| link_name_index_map.get(&link.name).unwrap())
        .expect("no root link");

    let leafs = joint_links
        .iter()
        .filter(|(_, link)| {
            robot
                .joints
                .iter()
                .all(|joint| joint.parent.link != link.name.clone())
        })
        .map(|(_, link)| *link_name_index_map.get(&link.name).unwrap())
        .collect::<Vec<_>>();

    let mut graph = petgraph::graphmap::DiGraphMap::new();

    // store all links index
    link_name_index_map.iter().for_each(|(_, index)| {
        graph.add_node(*index);
    });

    joint_links.iter().for_each(|(j, _)| {
        if let Some(joint) = j {
            let parent_index = link_name_index_map.get(&joint.parent.link).unwrap();
            let child_index = link_name_index_map.get(&joint.child.link).unwrap();
            graph.add_edge(*parent_index, *child_index, ());
        }
    });

    let link_map = joint_links
        .into_iter()
        .map(|(j,l)| {
            (
                *link_name_index_map.get(&l.name).unwrap(),
                Link {
                    joint_value: 0.,
                    joint_screw: j.as_ref().and_then(joint_screw),
                    joint_relative_pose: j
                        .as_ref()
                        .map(joint_relative_pose)
                        .unwrap_or(liealg::SE3::<f64>::identity()),
                    global_pose: liealg::SE3::<f64>::identity(),
                    urdf_link: l,
                    joint: j,
                },
            )
        })
        .collect::<HashMap<_, _>>();

    let mut multi_body = MultiBodyGraph {
        graph,
        link_map,
        name: robot.name,
        material: robot.materials,
        root_index: root,
        leafs_index: leafs,
    };

    multi_body.move_joint();
    multi_body
}

fn relative_pose(joint: &urdf_rs::Joint, joint_value: f64) -> liealg::SE3<f64> {
    let pose = joint_relative_pose(joint);
    let screw = joint_screw(joint);
    if let Some(screw) = screw {
        pose * (screw * joint_value).exp()
    } else {
        pose
    }
}

fn joint_relative_pose(joint: &urdf_rs::Joint) -> liealg::SE3<f64> {
    origin_to_se3(&joint.origin)
}

pub fn origin_to_se3(origin: &urdf_rs::Pose) -> liealg::SE3<f64> {
    let rpy = origin.rpy.0;
    let xyz = origin.xyz.0;
    SE3::new(&SO3::from_euler_angles(rpy[0], rpy[1], rpy[2]), xyz)
}

fn joint_screw(joint: &urdf_rs::Joint) -> Option<liealg::se3<f64>> {
    match joint.joint_type {
        urdf_rs::JointType::Revolute | urdf_rs::JointType::Continuous => {
            let norm =
                (joint.axis.xyz[0].powi(2) + joint.axis.xyz[1].powi(2) + joint.axis.xyz[2].powi(2))
                    .sqrt();
            let norm_x = joint.axis.xyz[0] / norm;
            let norm_y = joint.axis.xyz[1] / norm;
            let norm_z = joint.axis.xyz[2] / norm;
            Some(liealg::se3::<f64>::new(
                [norm_x, norm_y, norm_z],
                [0., 0., 0.],
            ))
        }
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_urdf_file() {
        let robot = parse_urdf_file("urdf/kuka_allegro_description/kuka.urdf").unwrap();
        robot.bfs(robot.root_index).iter().for_each(|j| {
            println!("{:?}", j);
        });

        println!("root index: {:?}", robot.root_index);
        let children = robot.children(robot.root_index);
        println!("root children: {:?}", children);

        println!("leaf index: {:?}", robot.leafs_index);
        let parent = robot.parent(robot.leafs_index[0]);
        println!("leaf parent: {:?}", parent);
    }
}
