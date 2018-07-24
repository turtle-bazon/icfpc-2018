use std::{cmp, iter};
use std::collections::HashSet;
use bit_vec::BitVec;

pub const LOWER_LIMIT: isize = 0;
pub const UPPER_LIMIT: isize = 250;

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub enum Axis { X, Y, Z, }

pub type M = isize;

#[derive(Debug)]
pub struct Resolution(pub M);

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Debug)]
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
impl LinearCoordDiff {
    pub fn get_axis(&self) -> Axis {
        match &self {
            LinearCoordDiff::Long{ axis, .. } | LinearCoordDiff::Short{ axis, .. } => *axis,
        }
    }
    pub fn get_value(&self) -> M {
        match &self {
            LinearCoordDiff::Long{ value, .. } | LinearCoordDiff::Short{ value, .. } => *value,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Debug)]
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

#[derive(Clone)]
pub struct Matrix {
    dim: usize,
    field: BitVec,
    filled: HashSet<Coord>,
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

    pub fn near_neighbours(&self) -> impl Iterator<Item = Coord> {
        iter::once(Coord { x: self.x - 1, y: self.y, z: self.z })
            .chain(iter::once(Coord { x: self.x + 1, y: self.y, z: self.z }))
            .chain(iter::once(Coord { x: self.x, y: self.y - 1, z: self.z }))
            .chain(iter::once(Coord { x: self.x, y: self.y + 1, z: self.z }))
            .chain(iter::once(Coord { x: self.x, y: self.y, z: self.z - 1 }))
            .chain(iter::once(Coord { x: self.x, y: self.y, z: self.z + 1 }))
            .filter(|c| c.x >= LOWER_LIMIT && c.x <= UPPER_LIMIT)
            .filter(|c| c.y >= LOWER_LIMIT && c.y <= UPPER_LIMIT)
            .filter(|c| c.z >= LOWER_LIMIT && c.z <= UPPER_LIMIT)
    }

    pub fn get_neighbours(&self) -> impl Iterator<Item = Coord> {
        let slf = self.clone();
        iproduct!(-1..2, -1..2, -1..2)
            .filter_map(move |(dx,dy,dz)| {
                let dc = CoordDiff(Coord{ x: dx, y: dy, z: dz });
                match dc.is_near() {
                    true => Some(slf.add(dc)),
                    false => None,
                }
            })
            .filter(|c| c.x >= LOWER_LIMIT && c.x <= UPPER_LIMIT)
            .filter(|c| c.y >= LOWER_LIMIT && c.y <= UPPER_LIMIT)
            .filter(|c| c.z >= LOWER_LIMIT && c.z <= UPPER_LIMIT)
    }

    pub fn get_neighbours_limit(&self, limit: isize) -> impl Iterator<Item = Coord> {
        self.get_neighbours()
            .filter(move |c| c.x < limit && c.y < limit && c.z < limit)
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
    pub fn is_far(&self) -> bool {
        self.l_inf_norm() > 0 && self.l_inf_norm() <= 30
    }
}

impl LinearCoordDiff {
    pub fn axis(&self) -> Axis {
        match self {
            LinearCoordDiff::Short{axis, value: _} => *axis,
            LinearCoordDiff::Long{axis, value: _} => *axis,
        }
    }

    pub fn value(&self) -> M {
        match self {
            LinearCoordDiff::Short{axis: _, value} => *value,
            LinearCoordDiff::Long{axis: _, value} => *value,
        }
    }

    pub fn to_coord_diff(&self) -> CoordDiff {
        match self.axis() {
            Axis::X => CoordDiff(Coord{ x: self.value(), y: 0, z: 0 }),
            Axis::Y => CoordDiff(Coord{ x: 0, y: self.value(), z: 0 }),
            Axis::Z => CoordDiff(Coord{ x: 0, y: 0, z: self.value() }),
        }
    }
}

impl Region {
    pub fn from_corners(l: &Coord, r: &Coord) -> Region {
        Region {
            min: Coord { x: cmp::min(l.x, r.x), y: cmp::min(l.y, r.y), z: cmp::min(l.z, r.z) },
            max: Coord { x: cmp::max(l.x, r.x), y: cmp::max(l.y, r.y), z: cmp::max(l.z, r.z) },
        }
    }

    pub fn contains(&self, coord: &Coord) -> bool {
        coord.x >= self.min.x && coord.x <= self.max.x &&
            coord.y >= self.min.y && coord.y <= self.max.y &&
            coord.z >= self.min.z && coord.z <= self.max.z
    }

    pub fn intersects(&self, other: &Region) -> bool {
        let not_x_proj =
            self.max.z < other.min.z || other.max.z < self.min.z || self.max.y < other.min.y || other.max.y < self.min.y;
        let not_y_proj =
            self.max.z < other.min.z || other.max.z < self.min.z || self.max.x < other.min.x || other.max.x < self.min.x;
        let not_z_proj =
            self.max.y < other.min.y || other.max.y < self.min.y || self.max.x < other.min.x || other.max.x < self.min.x;
        !(not_x_proj || not_y_proj || not_z_proj)
    }

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

    pub fn contents(&self) -> impl Iterator<Item = Coord> {
        let reg = *self;
        (reg.min.x ..= reg.max.x)
            .flat_map(move |x| {
                (reg.min.y ..= reg.max.y).flat_map(move |y| {
                    (reg.min.z ..= reg.max.z)
                        .map(move |z| Coord { x, y, z, })
                })
            })
    }

    pub fn coord_set(&self) -> HashSet<Coord> {
        let mut set = HashSet::new();

        for x in self.min.x..self.max.x+1 {
            for y in self.min.y..self.max.y+1 {
                for z in self.min.z..self.max.z+1 {
                    set.insert(Coord { x, y, z });
                }
            }
        }

        set
    }
}

impl Matrix {
    pub fn new(Resolution(dimension): Resolution) -> Matrix {
        let dim = dimension as usize;
        let total_size = dim * dim * dim;
        Matrix {
            dim,
            field: BitVec::from_elem(total_size, false),
            filled: HashSet::new(),
        }
    }

    pub fn new_empty_of_same_size(&self) -> Matrix {
        Matrix::new(Resolution(self.dim() as isize))
    }

    pub fn from_iter<I>(dim: Resolution, filled_coords: I) -> Matrix where I: IntoIterator<Item = Coord> {
        let mut matrix = Matrix::new(dim);
        for coord in filled_coords {
            matrix.set_filled(&coord);
        }
        matrix
    }

    pub fn dim(&self) -> usize {
        self.dim
    }

    pub fn set_filled(&mut self, &coord: &Coord) {
        let offset = (coord.x as usize * self.dim * self.dim) + (coord.y as usize * self.dim) + coord.z as usize;
        self.field.set(offset, true);
        self.filled.insert(coord);
    }

    pub fn set_void(&mut self, &coord: &Coord) {
        let offset = (coord.x as usize * self.dim * self.dim) + (coord.y as usize * self.dim) + coord.z as usize;
        self.field.set(offset, false);
        self.filled.remove(&coord);
    }

    pub fn is_filled(&self, coord: &Coord) -> bool {
        let offset = (coord.x as usize * self.dim * self.dim) + (coord.y as usize * self.dim) + coord.z as usize;
        if offset >= self.field.len() {
            panic!("out of bounds `Matrix::is_filled` check for {:?} size {} x {} x {}", coord, self.dim, self.dim, self.dim);
        }
        self.field[offset]
    }

    pub fn contains_filled(&self, region: &Region) -> bool {
        let mut coord = region.min;
        loop {
            if self.is_filled(&coord) {
                return true;
            }
            coord.z += 1;
            if coord.z > region.max.z || coord.z >= self.dim() as isize {
                coord.z = region.min.z;
                coord.y += 1;
            }
            if coord.y > region.max.y || coord.y >= self.dim() as isize {
                coord.y = region.min.y;
                coord.x += 1;
            }
            if coord.x > region.max.x || coord.x >= self.dim() as isize {
                return false;
            }
        }
    }

    pub fn filled_near_neighbours<'a>(&'a self, coord: &Coord) -> impl Iterator<Item = Coord> + 'a {
        coord.near_neighbours()
            .filter(move |c| c.x < self.dim as isize)
            .filter(move |c| c.y < self.dim as isize)
            .filter(move |c| c.z < self.dim as isize)
            .filter(move |c| self.is_filled(c))
    }

    pub fn will_be_grounded(&self, coord: &Coord) -> bool {
        use pathfinding::directed::astar;

        astar::astar(
            coord,
            |coord| self.filled_near_neighbours(coord)
                .map(|neighbour| (neighbour, neighbour.y)),
            |coord| coord.y,
            |coord| coord.y == 0,
        ).is_some()
    }

    pub fn is_grounded(&self, coord: &Coord) -> bool {
        if !self.is_filled(coord) {
            return false;
        }
        self.will_be_grounded(coord)
    }

    pub fn filled_voxels(&self) -> impl Iterator<Item = &Coord> {
        self.filled.iter()
    }

    pub fn all_voxels_are_grounded(&self) -> bool {
        all_voxels_are_grounded(self.filled.clone())
    }

    pub fn first_ungrounded_voxel(&self) -> Option<Coord> {
        first_ungrounded_voxel(self.filled.clone())
    }

    pub fn is_valid_coord(&self, c: &Coord) -> bool {
        c.x >= 0 && c.y >= 0 && c.z >= 0
            && (c.x as usize) < self.dim()
            && (c.y as usize) < self.dim()
            && (c.z as usize) < self.dim()
    }

    pub fn equals(&self, other: &Matrix) -> bool {
        &self.field == &other.field
    }
}

pub fn all_voxels_are_grounded(voxels_pending: HashSet<Coord>) -> bool {
    first_ungrounded_voxel(voxels_pending).is_none()
}

pub fn first_ungrounded_voxel(mut voxels_pending: HashSet<Coord>) -> Option<Coord> {
    let mut queue = Vec::with_capacity(voxels_pending.len());
    while let Some(&voxel) = voxels_pending.iter().next() {
        queue.push(voxel);
        let mut grounded = false;
        while let Some(voxel) = queue.pop() {
            if !voxels_pending.remove(&voxel) {
                continue;
            }
            if voxel.y == 0 {
                grounded = true;
            }
            queue.extend(voxel.near_neighbours().filter(|c| voxels_pending.contains(c)))
        }
        if !grounded {
            return Some(voxel);
        }
    }
    None
}

use std::fmt;

impl fmt::Debug for Matrix {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        f.debug_struct("Matrix")
            .field("dimension", &self.dim)
            .finish()
    }
}


#[cfg(test)]
mod tests {
    use super::{Coord, Resolution, Matrix, LinearCoordDiff, Axis, Region};

    #[test]
    fn is_grounded_single_empty() {
        let matrix = Matrix::from_iter(Resolution(3), vec![]);
        assert!(!matrix.is_grounded(&Coord { x: 1, y: 0, z: 1, }));
    }

    #[test]
    fn is_grounded_single_on_floor() {
        let matrix = Matrix::from_iter(Resolution(3), vec![Coord { x: 1, y: 0, z: 1, }]);
        assert!(matrix.is_grounded(&Coord { x: 1, y: 0, z: 1, }));
    }

    #[test]
    fn is_grounded_single_flying() {
        let matrix = Matrix::from_iter(Resolution(3), vec![Coord { x: 1, y: 1, z: 1, }]);
        assert!(!matrix.is_grounded(&Coord { x: 1, y: 1, z: 1, }));
    }

    #[test]
    fn is_grounded_double() {
        let matrix = Matrix::from_iter(
            Resolution(3),
            vec![
                Coord { x: 1, y: 1, z: 1, },
                Coord { x: 1, y: 0, z: 1, },
            ]);
        assert!(matrix.is_grounded(&Coord { x: 1, y: 1, z: 1, }));
        assert!(matrix.is_grounded(&Coord { x: 1, y: 0, z: 1, }));
        assert!(!matrix.is_grounded(&Coord { x: 1, y: 2, z: 1, }));
    }

    #[test]
    fn is_grounded_cross() {
        let matrix = Matrix::from_iter(
            Resolution(3),
            vec![
                Coord { x: 1, y: 0, z: 1, },
                Coord { x: 0, y: 1, z: 1, },
                Coord { x: 1, y: 1, z: 0, },
                Coord { x: 1, y: 1, z: 2, },
                Coord { x: 1, y: 1, z: 1, },
                Coord { x: 2, y: 1, z: 1, },
                Coord { x: 1, y: 2, z: 1, },
            ]);
        assert!(matrix.is_grounded(&Coord { x: 1, y: 0, z: 1, }));
        assert!(matrix.is_grounded(&Coord { x: 0, y: 1, z: 1, }));
        assert!(matrix.is_grounded(&Coord { x: 1, y: 1, z: 0, }));
        assert!(matrix.is_grounded(&Coord { x: 1, y: 1, z: 2, }));
        assert!(matrix.is_grounded(&Coord { x: 1, y: 1, z: 1, }));
        assert!(matrix.is_grounded(&Coord { x: 2, y: 1, z: 1, }));
        assert!(matrix.is_grounded(&Coord { x: 1, y: 2, z: 1, }));
    }

    #[test]
    fn is_all_grounded_cross() {
        let matrix = Matrix::from_iter(
            Resolution(3),
            vec![
                Coord { x: 1, y: 0, z: 1, },
                Coord { x: 0, y: 1, z: 1, },
                Coord { x: 1, y: 1, z: 0, },
                Coord { x: 1, y: 1, z: 2, },
                Coord { x: 1, y: 1, z: 1, },
                Coord { x: 2, y: 1, z: 1, },
                Coord { x: 1, y: 2, z: 1, },
            ]);
        assert!(matrix.all_voxels_are_grounded());
    }

    #[test]
    fn is_all_grounded_corrupt_cross() {
        let matrix = Matrix::from_iter(
            Resolution(3),
            vec![
                Coord { x: 1, y: 0, z: 1, },
                Coord { x: 0, y: 1, z: 1, },
                Coord { x: 1, y: 1, z: 0, },
                Coord { x: 1, y: 1, z: 2, },
                Coord { x: 2, y: 1, z: 1, },
                Coord { x: 1, y: 2, z: 1, },
            ]);
        assert!(!matrix.all_voxels_are_grounded());
    }

    #[test]
    fn is_coord_valid() {
        let matrix = Matrix::from_iter(Resolution(3), vec![]);
        assert!(matrix.is_valid_coord(&Coord { x: 1, y: 0, z: 0}));
        assert!(!matrix.is_valid_coord(&Coord { x: 3, y: 1, z: 0}));
        assert!(!matrix.is_valid_coord(&Coord { x: 1, y: 3, z: 0}));
        assert!(!matrix.is_valid_coord(&Coord { x: 1, y: 2, z: 3}));
    }

    #[test]
    fn is_linear_to_coord_diff_works_fine() {
        let lin = LinearCoordDiff::Short {
            axis: Axis::X, value: 2,
        };
        let diff = lin.to_coord_diff();
        assert_eq!(diff.0.x, 2);
        assert_eq!(diff.0.y, 0);
        assert_eq!(diff.0.z, 0);

        let lin = LinearCoordDiff::Short {
            axis: Axis::Y, value: 3,
        };
        let diff = lin.to_coord_diff();
        assert_eq!(diff.0.x, 0);
        assert_eq!(diff.0.y, 3);
        assert_eq!(diff.0.z, 0);

        let lin = LinearCoordDiff::Short {
            axis: Axis::Z, value: 4,
        };
        let diff = lin.to_coord_diff();
        assert_eq!(diff.0.x, 0);
        assert_eq!(diff.0.y, 0);
        assert_eq!(diff.0.z, 4);

        let lin = LinearCoordDiff::Long {
            axis: Axis::X, value: 10,
        };
        let diff = lin.to_coord_diff();
        assert_eq!(diff.0.x, 10);
        assert_eq!(diff.0.y, 0);
        assert_eq!(diff.0.z, 0);

        let lin = LinearCoordDiff::Long {
            axis: Axis::Y, value: 11,
        };
        let diff = lin.to_coord_diff();
        assert_eq!(diff.0.x, 0);
        assert_eq!(diff.0.y, 11);
        assert_eq!(diff.0.z, 0);

        let lin = LinearCoordDiff::Long {
            axis: Axis::Z, value: 12,
        };
        let diff = lin.to_coord_diff();
        assert_eq!(diff.0.x, 0);
        assert_eq!(diff.0.y, 0);
        assert_eq!(diff.0.z, 12);
    }

    #[test]
    fn contains_filled() {
        let matrix = Matrix::from_iter(
            Resolution(3),
            vec![
                Coord { x: 2, y: 2, z: 2, },
           ]);

        assert!(matrix.contains_filled(&Region::from_corners(&Coord { x: 0, y: 0, z: 0, },
                                                             &Coord { x: 2, y: 2, z: 2, })));
        assert!(!matrix.contains_filled(&Region::from_corners(&Coord { x: 0, y: 0, z: 0, },
                                                              &Coord { x: 0, y: 0, z: 0, })));
        assert!(matrix.contains_filled(&Region::from_corners(&Coord { x: 2, y: 2, z: 2, },
                                                             &Coord { x: 2, y: 2, z: 2, })));
        assert!(matrix.contains_filled(&Region::from_corners(&Coord { x: 1, y: 1, z: 1, },
                                                             &Coord { x: 2, y: 2, z: 2, })));
    }

    #[test]
    fn regions_intersect() {
        let region = Region {
            min: Coord { x: 0, y: 0, z: 0, },
            max: Coord { x: 1, y: 1, z: 1, },
        };
        assert!(region.intersects(&Region {
            min: Coord { x: 1, y: 1, z: 1, },
            max: Coord { x: 2, y: 2, z: 2, },
        }));
        assert!(region.intersects(&Region {
            min: Coord { x: 1, y: 1, z: 0, },
            max: Coord { x: 2, y: 1, z: 2, },
        }));
        assert!(!region.intersects(&Region {
            min: Coord { x: 0, y: 2, z: 0, },
            max: Coord { x: 2, y: 2, z: 2, },
        }));
    }

    #[test]
    fn will_be_grounded() {
        let matrix = Matrix::from_iter(Resolution(3), vec![Coord { x: 1, y: 0, z: 1, },]);
        assert!(matrix.will_be_grounded(&Coord { x: 1, y: 1, z: 1, }));
        assert!(matrix.will_be_grounded(&Coord { x: 0, y: 0, z: 0, }));
        assert!(!matrix.will_be_grounded(&Coord { x: 1, y: 2, z: 1, }));
        assert!(!matrix.will_be_grounded(&Coord { x: 0, y: 1, z: 1, }));

        let matrix = Matrix::from_iter(Resolution(3), vec![
            Coord { x: 1, y: 0, z: 1, },
            Coord { x: 1, y: 1, z: 1, },
            Coord { x: 1, y: 2, z: 1, },
        ]);
        assert!(!matrix.will_be_grounded(&Coord { x: 2, y: 1, z: 0, }));
    }

    #[test]
    fn tower_all_voxels_are_grounded() {
        let matrix = Matrix::from_iter(Resolution(3), vec![
            Coord { x: 1, y: 0, z: 1, },
            Coord { x: 1, y: 1, z: 1, },
            Coord { x: 1, y: 2, z: 1, },
        ]);
        assert!(super::all_voxels_are_grounded(matrix.filled_voxels().cloned().collect()));
        assert!(super::all_voxels_are_grounded(matrix.filled_voxels().cloned().filter(|v| v.y != 2).collect()));
        assert!(!super::all_voxels_are_grounded(matrix.filled_voxels().cloned().filter(|v| v.y != 1).collect()));
        assert!(!super::all_voxels_are_grounded(matrix.filled_voxels().cloned().filter(|v| v.y != 0).collect()));
    }
}
