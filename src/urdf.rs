use super::bfs::bfs;
use super::utils::*;
use super::Link;
use super::Model;
use urdf_rs::read_from_string;

impl Model {
    pub fn from_urdf_string(str: &str) -> std::io::Result<Self> {
        // construct a Model from robot
        let robot =
            read_from_string(str).map_err(std::io::Error::other)?;

        // parent_link_name -> (link_id, joint, link)
        // put joint and link together
        let temp_map = temp_link_map(robot);

        // find start that has no parent link
        let start = temp_map
            .iter()
            .find(|(k, _)| k.is_none())
            .map(|(_, v)| v.0)
            .unwrap_or(0);

        // construct a robot link_id digraph from robot
        // link_id -> link_id
        let link_graph = construct_link_graph(&temp_map);

        // construct a map: link_id -> link(empty)
        let mut link_map = construct_link_map(temp_map);

        let bfs = bfs(&link_graph, start);

        // fullfill the link in link_map(space_spatial_twist & global_zero_pose)
        fullfill_link_map(&mut link_map, &link_graph, &bfs);

        let mut v = link_map.into_iter().collect::<Vec<(usize, Link)>>();
        v.sort_by(|(a, _), (b, _)| a.cmp(b));

        Ok(Self {
            links: v.into_iter().map(|(_, l)| l).collect::<Vec<_>>(),
            link_graph,
            bfs,
        })
    }

    pub fn from_urdf(url: &str) -> std::io::Result<Self> {
        // check url is a file or a web url
        // if url is a file, read the file
        // if url is a web url, read the url
        let str = if url_is_urdf_file(url) {
            read_file(url)?
        } else if url_is_weburl(url) {
            read_web(url)?
        } else {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                "url is not a file or a web url",
            ));
        };
        Self::from_urdf_string(&str)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn from_urdf_test() {
        let file_path = "./urdf/rm_75_6fb_description/urdf/RM75-6F.urdf";
        let model = Model::from_urdf(file_path).unwrap();
        assert_eq!(model.links.len(), 8);
    }
}
