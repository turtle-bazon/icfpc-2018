use super::coord::{
    Region,
    Coord,
};

#[derive(Clone, PartialEq, Eq, Debug)]
pub enum Error {
    OutsideOfBoundingBox { bounding_box: Region, point: Coord, },
}

#[derive(Clone, PartialEq, Eq, Debug)]
pub enum InsertStatus {
    Inserted,
    Kept,
}

pub struct Octree {
    nodes: Vec<Node>,
}


#[derive(Debug)]
struct Node {
    bounds: Region,
    content: Content,
}

#[derive(Debug)]
enum Content {
    Empty,
    Leaf(Coord),
    Tree([usize; 8]),
}

impl Octree {
    pub fn new(dimension: Region) -> Octree {
        Octree {
            nodes: vec![Node {
                bounds: dimension,
                content: Content::Empty,
            }],
        }
    }

    pub fn insert(&mut self, point: Coord) -> Result<InsertStatus, Error> {
        loop {
            let mut node_ref = 0;
            let (mid, leaf) = loop {
                let node = &mut self.nodes[node_ref];
                if !node.bounds.contains(&point) {
                    return Err(Error::OutsideOfBoundingBox {
                        bounding_box: node.bounds,
                        point,
                    });
                }
                let mid = bounds_mid(&node.bounds);

                match node.content {
                    Content::Empty => {
                        node.content = Content::Leaf(point);
                        return Ok(InsertStatus::Inserted);
                    },
                    Content::Leaf(leaf) if leaf == point =>
                        return Ok(InsertStatus::Kept),
                    Content::Leaf(leaf) =>
                        break (mid, leaf),
                    Content::Tree(ref nodes) =>
                        node_ref = nodes[leaf_index(&mid, &point)],
                }
            };
            let bounds = self.nodes[node_ref].bounds;
            let node_root = self.nodes.len();
            self.nodes.push(Node { // 0
                bounds: Region {
                    min: Coord {
                        x: bounds.min.x,
                        y: bounds.min.y,
                        z: bounds.min.z,
                    },
                    max: Coord {
                        x: mid.x,
                        y: mid.y,
                        z: mid.z,
                    },
                },
                content: Content::Empty,
            });
            self.nodes.push(Node { // 1
                bounds: Region {
                    min: Coord {
                        x: bounds.min.x,
                        y: bounds.min.y,
                        z: mid.z + 1,
                    },
                    max: Coord {
                        x: mid.x,
                        y: mid.y,
                        z: bounds.max.z,
                    },
                },
                content: Content::Empty,
            });
            self.nodes.push(Node { // 2
                bounds: Region {
                    min: Coord {
                        x: bounds.min.x,
                        y: mid.y + 1,
                        z: bounds.min.z,
                    },
                    max: Coord {
                        x: mid.x,
                        y: bounds.max.y,
                        z: mid.z,
                    },
                },
                content: Content::Empty,
            });
            self.nodes.push(Node { // 3
                bounds: Region {
                    min: Coord {
                        x: bounds.min.x,
                        y: mid.y + 1,
                        z: mid.z + 1,
                    },
                    max: Coord {
                        x: mid.x,
                        y: bounds.max.y,
                        z: bounds.max.z,
                    },
                },
                content: Content::Empty,
            });
            self.nodes.push(Node { // 4
                bounds: Region {
                    min: Coord {
                        x: mid.x + 1,
                        y: bounds.min.y,
                        z: bounds.min.z,
                    },
                    max: Coord {
                        x: bounds.max.x,
                        y: mid.y,
                        z: mid.z,
                    },
                },
                content: Content::Empty,
            });
            self.nodes.push(Node { // 5
                bounds: Region {
                    min: Coord {
                        x: mid.x + 1,
                        y: bounds.min.y,
                        z: mid.z + 1,
                    },
                    max: Coord {
                        x: bounds.max.x,
                        y: mid.y,
                        z: bounds.max.z,
                    },
                },
                content: Content::Empty,
            });
            self.nodes.push(Node { // 6
                bounds: Region {
                    min: Coord {
                        x: mid.x + 1,
                        y: mid.y + 1,
                        z: bounds.min.z,
                    },
                    max: Coord {
                        x: bounds.max.x,
                        y: bounds.max.y,
                        z: mid.z,
                    },
                },
                content: Content::Empty,
            });
            self.nodes.push(Node { // 7
                bounds: Region {
                    min: Coord {
                        x: mid.x + 1,
                        y: mid.y + 1,
                        z: mid.z + 1,
                    },
                    max: Coord {
                        x: bounds.max.x,
                        y: bounds.max.y,
                        z: bounds.max.z,
                    },
                },
                content: Content::Empty,
            });
            self.nodes[node_ref].content = Content::Tree([
                node_root + 0,
                node_root + 1,
                node_root + 2,
                node_root + 3,
                node_root + 4,
                node_root + 5,
                node_root + 6,
                node_root + 7,
                ]);
            let leaf_ref = node_root + leaf_index(&mid, &leaf);
            let point_ref = node_root + leaf_index(&mid, &point);
            self.nodes[leaf_ref].content = Content::Leaf(leaf);
            if leaf_ref != point_ref {
                self.nodes[point_ref].content = Content::Leaf(point);
                return Ok(InsertStatus::Inserted);
            }
        }
    }

    pub fn contains(&self, point: &Coord) -> bool {
        let mut node_ref = 0;
        loop {
            let node = &self.nodes[node_ref];
            if !node.bounds.contains(&point) {
                return false;
            }
            let mid = bounds_mid(&node.bounds);

            match node.content {
                Content::Empty =>
                    return false,
                Content::Leaf(ref leaf) if leaf == point =>
                    return true,
                Content::Leaf(..) =>
                    return false,
                Content::Tree(ref nodes) =>
                    node_ref = nodes[leaf_index(&mid, &point)],
            }
        }
    }
}

fn bounds_mid(bounds: &Region) -> Coord {
    Coord {
        x: (bounds.min.x + bounds.max.x) / 2,
        y: (bounds.min.y + bounds.max.y) / 2,
        z: (bounds.min.z + bounds.max.z) / 2,
    }
}

fn leaf_index(mid: &Coord, point: &Coord) -> usize {
    match (point.x > mid.x, point.y > mid.y, point.z > mid.z) {
        (false, false, false) => 0,
        (false, false, true) => 1,
        (false, true, false) => 2,
        (false, true, true) => 3,
        (true, false, false) => 4,
        (true, false, true) => 5,
        (true, true, false) => 6,
        (true, true, true) => 7,
    }
}

#[cfg(test)]
mod test {
    use rand::{self, Rng};
    use super::Octree;
    use super::super::coord::{Coord, Region};

    #[test]
    fn contains_1000() {
        let mut tree =
            Octree::new(Region { min: Coord { x: 0, y: 0, z: 0, }, max: Coord { x: 250, y: 250, z: 250, }, });

        let mut rng = rand::thread_rng();
        let samples: Vec<_> = (0 .. 1000)
            .map(|_| Coord {
                x: rng.gen_range(0, 251),
                y: rng.gen_range(0, 251),
                z: rng.gen_range(0, 251),
            })
            .collect();
        for point in samples.iter().cloned() {
            tree.insert(point).unwrap();
        }
        for point in samples.iter() {
            if !tree.contains(point) {
                panic!("could not find sample point {:?} in octree", point);
            }
        }
    }
}
