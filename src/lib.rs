use std::{collections::HashMap, path::Path};

// use sdformat_rs::read_from_string as sdf_read_from_string;
// use slab_tree::TreeBuilder;
use urdf_rs::{read_file, read_from_string};

pub struct MultiBody {}

pub fn parse_urdf_string(string: &str) -> Result<MultiBody, urdf_rs::UrdfError> {
    let robot = read_from_string(string)?;
    Ok(parse_robot(robot))
}

pub fn parse_urdf_file(path: impl AsRef<Path>) -> Result<MultiBody, urdf_rs::UrdfError> {
    let robot = read_file(path)?;
    Ok(parse_robot(robot))
}

fn parse_robot(robot: urdf_rs::Robot) -> MultiBody {
    let links_map = robot
        .links
        .into_iter()
        .map(|link| (link.name.clone(), link))
        .collect::<HashMap<_, _>>();
    let joints_map = robot
        .joints
        .into_iter()
        .map(|joint| (joint.name.clone(), joint))
        .collect::<HashMap<_, _>>();

    // find base link
    let (base_link_name, base_link) = links_map
        .iter()
        .find(|(link_name, link)| {
            !joints_map
                .iter()
                .any(|(_joint_name, joint)| &joint.child.link == *link_name)
        })
        .unwrap();

    // build tree from base link

    todo!()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_urdf_file() {
        let robot = parse_urdf_file("test_data/urdf/robot.urdf").unwrap();
    }
}
