use hashbrown::HashSet;

use petgraph::visit::Bfs;

struct BfsIter<'a> {
    graph: &'a petgraph::graphmap::DiGraphMap<usize, ()>,
    bfs: Bfs<usize, HashSet<usize>>,
}

impl Iterator for BfsIter<'_> {
    type Item = usize;

    fn next(&mut self) -> Option<Self::Item> {
        self.bfs.next(self.graph)
    }
}

// BFS traversal of a directed graph
// Returns a vector of node ids in the order they were visited
// make sure every parent node is visited before child node
pub(super) fn bfs(graph: &petgraph::graphmap::DiGraphMap<usize, ()>, start: usize) -> Vec<usize> {
    let bfs = petgraph::visit::Bfs::new(graph, start);
    let iter = BfsIter { graph, bfs };
    iter.collect()
}
