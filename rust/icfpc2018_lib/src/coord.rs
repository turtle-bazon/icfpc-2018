use std::cmp;
use bit_vec::BitVec;

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub enum Axis { X, Y, Z, }

pub type M = isize;

#[derive(Debug)]
pub struct Resolution(pub M);

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub struct Coord {
    pub x: M,
    pub y: M,
    pub z: M,
}

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub struct CoordDiff(pub Coord);

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub enum LinearCoordDiff {
    Short { axis: Axis, value: M, },
    Long { axis: Axis, value: M, },
}

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub struct Region {
    pub min: Coord,
    pub max: Coord,
}

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub enum RegionDim {
    Point,
    Line,
    Plane,
    Box,
}

pub struct Matrix {
    dim: usize,
    field: BitVec,
}

impl Coord {
    pub fn add(&self, diff: CoordDiff) -> Coord {
        Coord {
            x: self.x + diff.0.x,
            y: self.y + diff.0.y,
            z: self.z + diff.0.z,
        }
    }

    pub fn diff(&self, other: &Coord) -> CoordDiff {
        CoordDiff(Coord {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
        })
    }

    pub fn is_adjacent(&self, other: &Coord) -> bool {
        self.diff(other).l_1_norm() == 1
    }
}

impl CoordDiff {
    pub fn l_1_norm(&self) -> usize {
        (self.0.x.abs() + self.0.y.abs() + self.0.z.abs()) as usize
    }

    pub fn l_inf_norm(&self) -> usize {
        cmp::max(
            cmp::max(
                self.0.x.abs(),
                self.0.y.abs(),
            ),
            self.0.z.abs(),
        ) as usize
    }

    pub fn is_near(&self) -> bool {
        self.l_inf_norm() == 1 && self.l_1_norm() <= 2
    }
}

impl Region {
    pub fn dimension(&self) -> RegionDim {
        match (self.min.x == self.max.x, self.min.y == self.max.y, self.min.z == self.max.z) {
            (true, true, true) =>
                RegionDim::Point,
            (false, true, true) | (true, false, true) | (true, true, false) =>
                RegionDim::Line,
            (false, false, true) | (false, true, false) | (true, false, false) =>
                RegionDim::Plane,
            (false, false, false) =>
                RegionDim::Box,
        }
    }
}

impl Matrix {
    pub fn new(Resolution(dimension): Resolution) -> Matrix {
        let dim = dimension as usize;
        let total_size = dim * dim * dim;
        Matrix { dim, field: BitVec::with_capacity(total_size), }
    }

    pub fn is_filled(&self, coord: &Coord) -> bool {
        let offset = (coord.x as usize * self.dim * self.dim) + (coord.y as usize * self.dim) + coord.z as usize;
        self.field[offset]
    }
}
