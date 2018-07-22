use kdvtree;

use super::{
    coord::{
        Axis,
        Coord,
        Region,
    },
};

pub type KdTree =
    kdvtree::KdvTree<Axis, Coord, BoundingBox, Vec<Coord>>;

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub struct BoundingBox(Region);
