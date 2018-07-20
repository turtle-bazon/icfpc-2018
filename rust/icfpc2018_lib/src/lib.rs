use std::cmp;

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

// dummy

pub fn dummy() {
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
