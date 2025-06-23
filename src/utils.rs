use std::{collections::HashMap, io::Read};

use liealg::{se3, Adjoint, Group, SE3, SO3};
use nalgebra::{Matrix3, Matrix6};

use super::{spatial_inertial::to_local_spatial_inertial, Joint, Link};

pub(super) fn url_is_urdf_file(url: &str) -> bool {
    std::path::Path::new(url).exists() && (url.ends_with(".urdf") || url.ends_with(".URDF"))
}

pub(super) fn url_is_weburl(url: &str) -> bool {
    // check if url is a web url
    // if url starts with http:// or https://
    // and ends with .urdf
    (url.starts_with("http://") || url.starts_with("https://"))
        && (url.ends_with(".urdf") || url.ends_with(".URDF"))
}

pub(super) fn read_file(url: &str) -> std::io::Result<String> {
    // read file from url
    // if url is a file, read the file
    let path = std::path::Path::new(url);
    let mut file = std::fs::File::open(path)?;
    let mut contents = String::new();
    file.read_to_string(&mut contents)?;
    Ok(contents)
}

pub(super) fn read_web(url: &str) -> std::io::Result<String> {
    // read web from url
    // if url is a web url, read the url
    let mut response = reqwest::blocking::get(url)
        .map_err(std::io::Error::other)?;
    let mut contents = String::new();
    response.read_to_string(&mut contents)?;
    Ok(contents)
}

pub(super) fn temp_link_map(
    robot: urdf_rs::Robot,
) -> HashMap<Option<String>, (usize, Option<urdf_rs::Joint>, urdf_rs::Link)> {
    // parent_link_name -> (link_id, joint, link)
    robot
        .links
        .into_iter()
        .enumerate()
        .map(|(index, link)| {
            let j = robot
                .joints
                .iter()
                .find(|joint| joint.child.link == link.name);
            (j.map(|j| j.parent.link.clone()), (index, j.cloned(), link))
        })
        .collect::<HashMap<_, _>>()
}

pub(super) fn construct_link_graph(
    temp_map: &HashMap<Option<String>, (usize, Option<urdf_rs::Joint>, urdf_rs::Link)>,
) -> petgraph::graphmap::DiGraphMap<usize, ()> {
    // empty graph
    let mut graph = petgraph::graphmap::DiGraphMap::<usize, ()>::new();

    // add all nodes to graph
    temp_map.iter().for_each(|(_, (self_index, _, _))| {
        graph.add_node(*self_index);
    });

    // add all edges to graph
    temp_map.iter().for_each(|(_, (self_index, _, link))| {
        if let Some((child_index, _, _)) = temp_map.get(&Some(link.name.clone())) {
            graph.add_edge(*self_index, *child_index, ());
        }
    });

    graph
}

pub(super) fn construct_link_map(
    temp_map: HashMap<Option<String>, (usize, Option<urdf_rs::Joint>, urdf_rs::Link)>,
) -> HashMap<usize, Link> {
    temp_map
        .into_iter()
        .map(|(_, (i, j, l))| {
            (
                i,
                Link {
                    // need pre build the link graph
                    space_spatial_screw: se3::identity(),

                    local_spatial_screw: se3::identity(),

                    // construct from joint
                    // local_spatial_twist: None,
                    global_zero_pose: SE3::identity(),
                    parent_zero_pose: SE3::identity(),

                    // use Steiner’s theorem
                    local_spatial_inertial: spatial_inertia(&l),

                    // urdf_link: l,
                    joint: Joint { urdf_joint: j },
                },
            )
        })
        .collect::<HashMap<_, _>>()
}

pub(super) fn spatial_inertia(link: &urdf_rs::Link) -> Matrix6<f64> {
    let i = &link.inertial.inertia;
    let inertia = Matrix3::new(
        i.ixx, i.ixy, i.ixz, i.ixy, i.iyy, i.iyz, i.ixz, i.iyz, i.izz,
    );
    let mass = link.inertial.mass.value;
    to_local_spatial_inertial(&pose_to_se3(&link.inertial.origin), &inertia, mass)
}

pub(super) fn pose_to_se3(pose: &urdf_rs::Pose) -> SE3<f64> {
    let rpy = pose.rpy;
    SE3::new(&SO3::from_euler_angles(rpy[0], rpy[1], rpy[2]), pose.xyz.0)
}

pub(super) fn fullfill_link_map(
    link_map: &mut HashMap<usize, Link>,
    graph: &petgraph::graphmap::DiGraphMap<usize, ()>,
    bfs: &[usize],
) {
    for link in bfs {
        // get the parent link
        let parent_index = graph
            .neighbors_directed(*link, petgraph::Direction::Incoming)
            .next();
        if let Some(parent_index) = parent_index {
            let parent_global_pose = link_map
                .get(&parent_index)
                .unwrap()
                .global_zero_pose
                .clone();
            let relative_pose = pose_to_se3(
                &link_map
                    .get(link)
                    .unwrap()
                    .joint
                    .urdf_joint
                    .as_ref()
                    .unwrap()
                    .origin,
            );
            let twist = joint_twist(
                link_map
                    .get(link)
                    .unwrap()
                    .joint
                    .urdf_joint
                    .as_ref()
                    .unwrap(),
            );
            let link_mut = link_map.get_mut(link).unwrap();

            let global_pose = parent_global_pose * relative_pose.clone();
            link_mut.parent_zero_pose = relative_pose;

            let global_screw = global_pose.adjoint().act(&twist);
            link_mut.global_zero_pose = global_pose;

            link_mut.local_spatial_screw = twist;
            link_mut.space_spatial_screw = global_screw;
        }
    }
}

pub(super) fn joint_twist(joint: &urdf_rs::Joint) -> liealg::se3<f64> {
    match joint.joint_type {
        urdf_rs::JointType::Revolute | urdf_rs::JointType::Continuous => {
            let norm =
                (joint.axis.xyz[0].powi(2) + joint.axis.xyz[1].powi(2) + joint.axis.xyz[2].powi(2))
                    .sqrt();
            let norm_x = joint.axis.xyz[0] / norm;
            let norm_y = joint.axis.xyz[1] / norm;
            let norm_z = joint.axis.xyz[2] / norm;
            liealg::se3::<f64>::new([norm_x, norm_y, norm_z], [0., 0., 0.])
        }

        urdf_rs::JointType::Prismatic => {
            let norm =
                (joint.axis.xyz[0].powi(2) + joint.axis.xyz[1].powi(2) + joint.axis.xyz[2].powi(2))
                    .sqrt();
            let norm_x = joint.axis.xyz[0] / norm;
            let norm_y = joint.axis.xyz[1] / norm;
            let norm_z = joint.axis.xyz[2] / norm;
            liealg::se3::<f64>::new([0., 0., 0.], [norm_x, norm_y, norm_z])
        }

        urdf_rs::JointType::Fixed => liealg::se3::<f64>::identity(),

        _ => panic!("joint type not supported"),
    }
}

#[cfg(test)]
mod tests {
    use liealg::Algebra;

    use crate::bfs::bfs;

    use super::*;

    #[test]
    fn url_is_file_test() {
        assert!(url_is_urdf_file(
            "./urdf/rm_75_6fb_description/urdf/RM75-6F.urdf"
        ));
        assert!(!url_is_urdf_file("src/model/urdf/test.txt"));
    }

    #[test]
    fn url_is_weburl_test() {
        assert!(url_is_weburl("http://www.example.com/urdf/test.urdf"));
        assert!(url_is_weburl("https://www.example.com/urdf/test.urdf"));
        assert!(!url_is_weburl("http://www.example.com/urdf/test.txt"));
        assert!(!url_is_weburl("https://www.example.com/urdf/test.txt"));
        assert!(!url_is_weburl("http://www.example.com/urdf/test"));
        assert!(!url_is_weburl("https://www.example.com/urdf/test"));
    }

    #[test]
    fn read_file_test() {
        let path = std::path::Path::new("./urdf/rm_75_6fb_description/urdf/RM75-6F.urdf");
        let contents = read_file(path.to_str().unwrap()).unwrap();
        assert!(!contents.is_empty());
    }

    #[test]
    fn read_web_test() {
        let url = "https://gitee.com/RealManRobot/rm_models/raw/main/RM75/urdf/rm_75_6fb_description/urdf/RM75-6F.urdf";
        let web_contents = read_web(url).unwrap();
        let path = std::path::Path::new("./urdf/rm_75_6fb_description/urdf/RM75-6F.urdf");
        let file_contents = read_file(path.to_str().unwrap()).unwrap();
        assert!(web_contents == file_contents);
    }

    #[test]
    fn temp_link_map_test() {
        let robot = urdf_rs::read_file("./urdf/rm_75_6fb_description/urdf/RM75-6F.urdf").unwrap();
        let map = temp_link_map(robot);
        map.iter()
            .map(|(p, (id, j, l))| (p, (id, j.as_ref().map(|j| j.name.clone()), l.name.clone())))
            .for_each(|(p, (id, j, l))| {
                println!(
                    "{} -> {} {} {}",
                    p.as_ref().unwrap_or(&"None".to_string()),
                    id,
                    j.as_ref().unwrap_or(&"None".to_string()),
                    l
                );
            });
        // works fine
        // println!("{:#?}", map);
    }

    #[test]
    fn pose_to_se3_test() {
        let robot = urdf_rs::read_file("./urdf/rm_75_6fb_description/urdf/RM75-6F.urdf").unwrap();
        let map = temp_link_map(robot);
        let (_, _, link) = map.get(&None).unwrap();
        let pose = pose_to_se3(&link.inertial.origin);
        println!("name: {}", link.name);
        println!("{:.4}", pose);
    }

    #[test]
    fn construct_link_graph_test() {
        let robot = urdf_rs::read_file("./urdf/rm_75_6fb_description/urdf/RM75-6F.urdf").unwrap();
        let map = temp_link_map(robot);
        let graph = construct_link_graph(&map);
        // looks works fine
        println!("{:?}", graph);
    }

    #[test]
    fn spatial_inertia_test() {
        let robot = urdf_rs::read_file("./urdf/rm_75_6fb_description/urdf/RM75-6F.urdf").unwrap();
        let map = temp_link_map(robot);
        let (_, _, link) = map.get(&None).unwrap();
        let inertia = spatial_inertia(link);
        println!("name: {}", link.name);
        println!("{:.2e}", inertia);
    }

    #[test]
    fn construct_link_map_test() {
        let robot = urdf_rs::read_file("./urdf/rm_75_6fb_description/urdf/RM75-6F.urdf").unwrap();
        let map = temp_link_map(robot);
        let link_map = construct_link_map(map);
        // looks works fine
        println!("{:#?}", link_map);

        // link_map.iter().for_each(|(i, l)| {
        //     println!("index{}: {}", i, l.urdf_link.name);
        // });
    }

    #[test]
    fn joint_twist_test() {
        let robot = urdf_rs::read_file("./urdf/rm_75_6fb_description/urdf/RM75-6F.urdf").unwrap();
        let map = temp_link_map(robot);
        let link_map = construct_link_map(map);

        link_map.iter().for_each(|(i, l)| {
            if let Some(joint) = &l.joint.urdf_joint {
                println!("{}: {:.3}", i, joint_twist(joint).vee());
            }
        });
    }

    #[test]
    fn fullfill_link_map_test() {
        let robot = urdf_rs::read_file("./urdf/rm_75_6fb_description/urdf/RM75-6F.urdf").unwrap();
        let map = temp_link_map(robot);
        let graph = construct_link_graph(&map);
        let mut link_map = construct_link_map(map);
        fullfill_link_map(&mut link_map, &graph, &bfs(&graph, 0));
        // looks works fine
        link_map.iter().for_each(|(i, l)| {
            println!(
                "{}: {:.3} {:.3} {:.3}",
                i,
                l.global_zero_pose,
                l.space_spatial_screw.vee(),
                l.local_spatial_screw.vee()
            );
        });
    }
}
