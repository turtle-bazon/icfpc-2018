use std::mem;
use std::collections::BinaryHeap;

use super::coord::{
    Region,
    Coord,
    CoordDiff,
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

pub struct Octree<T> {
    nodes: Vec<Node<T>>,
    queue: BinaryHeap<(isize, usize)>,
}


#[derive(Debug)]
struct Node<T> {
    bounds: Region,
    content: Content<T>,
}

#[derive(Debug)]
enum Content<T> {
    Empty,
    Leaf { point: Coord, value: T, },
    Tree([usize; 8]),
}

impl<T> Octree<T> {
    pub fn new(dimension: Region) -> Octree<T> {
        Octree {
            nodes: vec![Node {
                bounds: dimension,
                content: Content::Empty,
            }],
            queue: BinaryHeap::new(),
        }
    }

    pub fn insert(&mut self, mut point: Coord, mut value: T) -> Result<InsertStatus, Error> {
        let mut node_ref = 0;
        loop {
            let mid = loop {
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
                        node.content = Content::Leaf { point, value, };
                        return Ok(InsertStatus::Inserted);
                    },
                    Content::Leaf { point: leaf_point, .. } if leaf_point == point =>
                        return Ok(InsertStatus::Kept),
                    Content::Leaf { .. } =>
                        break mid,
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
            let point_ref = node_root + leaf_index(&mid, &point);
            self.nodes[point_ref].content = Content::Leaf { point, value, };
            let content_tree = Content::Tree([
                node_root + 0,
                node_root + 1,
                node_root + 2,
                node_root + 3,
                node_root + 4,
                node_root + 5,
                node_root + 6,
                node_root + 7,
                ]);
            if let Content::Leaf { point: leaf_point, value: leaf_value, } = mem::replace(&mut self.nodes[node_ref].content, content_tree) {
                point = leaf_point;
                value = leaf_value;
            } else {
                unreachable!()
            }
        }
    }

    pub fn get(&self, point: &Coord) -> Option<&T> {
        let mut node_ref = 0;
        loop {
            let node = &self.nodes[node_ref];
            if !node.bounds.contains(&point) {
                return None;
            }

            match node.content {
                Content::Empty =>
                    return None,
                Content::Leaf { point: ref leaf_point, ref value, } if leaf_point == point =>
                    return Some(value),
                Content::Leaf { .. } =>
                    return None,
                Content::Tree(ref nodes) => {
                    let mid = bounds_mid(&node.bounds);
                    node_ref = nodes[leaf_index(&mid, &point)];
                },
            }
        }
    }

    pub fn nearest<'a>(&'a mut self, needle: &Coord) -> Option<(usize, &'a Coord, &'a T)> where T: ::std::fmt::Debug {

        self.queue.clear();
        self.queue.push((0, 0));

        let mut best_found: Option<(usize, &'a Coord, &'a T)> = None;

        while let Some((node_distance_neg, node_ref)) = self.queue.pop() {
            let node_distance = -node_distance_neg as usize;
            if best_found.as_ref().map(|best| best.0 < node_distance).unwrap_or(false) {
                break;
            }

            let node = &self.nodes[node_ref];
            match node.content {
                Content::Empty =>
                    (),
                Content::Leaf { ref point, ref value, } if point == needle =>
                    return Some((0, point, value)),
                Content::Leaf { ref point, ref value, } => {
                    let distance = needle.diff(point).l_1_norm();
                    if best_found.as_ref().map(|best| distance < best.0).unwrap_or(true) {
                        best_found = Some((distance, point, value));
                    }
                },
                Content::Tree(ref children) => {
                    let nodes = &self.nodes;
                    self.queue.extend(children.iter().map(|&node_ref| {
                        let node = &nodes[node_ref];
                        let proj_diff = CoordDiff(Coord {
                            x: if needle.x < node.bounds.min.x {
                                node.bounds.min.x - needle.x
                            } else if needle.x > node.bounds.max.x {
                                needle.x - node.bounds.max.x
                            } else {
                                0
                            },
                            y: if needle.y < node.bounds.min.y {
                                node.bounds.min.y - needle.y
                            } else if needle.y > node.bounds.max.y {
                                needle.y - node.bounds.max.y
                            } else {
                                0
                            },
                            z: if needle.z < node.bounds.min.z {
                                node.bounds.min.z - needle.z
                            } else if needle.z > node.bounds.max.z {
                                needle.z - node.bounds.max.z
                            } else {
                                0
                            },
                        });
                        let node_distance_neg = -(proj_diff.l_1_norm() as isize);
                        (node_distance_neg, node_ref)
                    }));
                },
            }
        }

        best_found
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
            .map(|_| (Coord {
                x: rng.gen_range(1, 251),
                y: rng.gen_range(1, 251),
                z: rng.gen_range(1, 251),
            }, rng.gen_range(-1000, 1000)))
            .collect();
        for &(point, value) in samples.iter() {
            tree.insert(point, value).unwrap();
        }
        for &(ref point, ref value) in samples.iter() {
            assert_eq!(tree.get(point), Some(value));
        }
        assert_eq!(tree.get(&Coord { x: 0, y: 0, z: 0, }), None);
    }

    #[test]
    fn nearest_1000() {
        use std::collections::HashMap;

        let mut tree =
            Octree::new(Region { min: Coord { x: 0, y: 0, z: 0, }, max: Coord { x: 250, y: 250, z: 250, }, });

        let mut rng = rand::thread_rng();
        let samples: HashMap<_, _> = (0 .. 1000)
            .map(|_| (Coord {
                x: rng.gen_range(0, 251),
                y: rng.gen_range(0, 251),
                z: rng.gen_range(0, 251),
            }, rng.gen_range(-1000, 1000)))
            .collect();
        for (&point, &value) in samples.iter() {
            tree.insert(point, value).unwrap();
        }

        for _ in 0 .. 1000 {
            let needle = Coord {
                x: rng.gen_range(0, 251),
                y: rng.gen_range(0, 251),
                z: rng.gen_range(0, 251),
            };

            let expected = samples
                .iter()
                .map(|(p, v)| (needle.diff(p).l_1_norm(), p, v))
                .min_by_key(|r| r.0);
            let result = tree.nearest(&needle);
            assert_eq!(result.map(|r| r.0), expected.map(|r| r.0));
        }
    }
}
