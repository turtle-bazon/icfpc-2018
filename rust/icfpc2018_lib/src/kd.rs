use std::{iter, cmp::Ordering};
use kdvtree;

use super::{
    coord::{
        Axis,
        Coord,
    },
};

pub struct KdTree {
    kd: kdvtree::KdvTree<Axis, Coord, BoundingBox, Coord>,
}

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub struct BoundingBox(Coord);

impl kdvtree::BoundingVolume<Coord> for BoundingBox {
    fn min_corner(&self) -> Coord {
        self.0
    }

    fn max_corner(&self) -> Coord {
        self.0
    }
}

fn cmp_points(axis: &Axis, a: &Coord, b: &Coord) -> Ordering {
    match axis {
        &Axis::X => a.x.cmp(&b.x),
        &Axis::Y => a.y.cmp(&b.y),
        &Axis::Z => a.z.cmp(&b.z),
    }
}

fn get_bounding_volume(&voxel: &Coord) -> BoundingBox {
    BoundingBox(voxel)
}

struct CutPoint {
    buffer: Vec<Coord>,
}

impl CutPoint {
    fn new() -> CutPoint {
        CutPoint {
            buffer: Vec::new(),
        }
    }
}

impl kdvtree::GetCutPoint<Axis, Coord> for CutPoint {
    fn cut_point<I>(&mut self, cut_axis: &Axis, points: I) -> Option<Coord> where I: Iterator<Item = Coord> {
        self.buffer.clear();
        self.buffer.extend(points);
        self.buffer.sort_by_key(|p| match cut_axis {
            &Axis::X => p.x,
            &Axis::Y => p.y,
            &Axis::Z => p.z,
        });
        let mid = self.buffer.len() / 2;
        self.buffer.get(mid).cloned()
    }
}

fn empty_cutter(_shape: &Coord, _fragment: &BoundingBox, _cut_axis: &Axis, _cut_point: &Coord) -> Result<Option<(BoundingBox, BoundingBox)>, ()> {
    Ok(None)
}

fn bv_to_cut_point_sq_dist(axis: &Axis, bounding_volume: &BoundingBox, cut_point: &Coord) -> usize {
    let diff = bounding_volume.0.diff(cut_point);
    match axis {
        &Axis::X => diff.0.x,
        &Axis::Y => diff.0.y,
        &Axis::Z => diff.0.z,
    }.abs() as usize
}

fn bv_to_bv_sq_dist(bounding_volume_a: &BoundingBox, bounding_volume_b: &BoundingBox) -> usize {
    bounding_volume_a.0.diff(&bounding_volume_b.0).l_1_norm()
}

impl KdTree {
    pub fn build<I>(voxels: I) -> KdTree where I: IntoIterator<Item = Coord> {
        let kd = kdvtree::KdvTree::build(
            iter::once(Axis::X).chain(iter::once(Axis::Y)).chain(iter::once(Axis::Z)),
            voxels,
            cmp_points,
            get_bounding_volume,
            CutPoint::new(),
            empty_cutter,
        ).unwrap_or_else(|()| unreachable!());
        KdTree { kd, }
    }

    pub fn nearest<'a>(&'a self, voxel: &'a Coord) -> impl Iterator<Item = (Coord, usize)> + 'a {
        self.kd.nearest(
            voxel,
            cmp_points,
            get_bounding_volume,
            empty_cutter,
            bv_to_cut_point_sq_dist,
            bv_to_bv_sq_dist,
        )
            .map(|maybe_result| maybe_result.unwrap_or_else(|()| unreachable!()))
            .map(|nearest_shape| (*nearest_shape.shape, nearest_shape.dist))
    }
}

#[cfg(test)]
mod test {
    use super::super::coord::Coord;
    use super::KdTree;

    #[test]
    fn build_empty() {
        let kd = KdTree::build(None);
        assert_eq!(
            kd.nearest(&Coord { x: 0, y: 0, z: 0, }).collect::<Vec<_>>(),
            vec![],
        );
    }

    #[test]
    fn build_1() {
        let kd = KdTree::build(vec![
            Coord { x: 1, y: 1, z: 1, },
        ]);

        assert_eq!(
            kd.nearest(&Coord { x: 0, y: 0, z: 0, }).collect::<Vec<_>>(),
            vec![(Coord { x: 1, y: 1, z: 1 }, 3)],
        );
    }

    #[test]
    fn build_5() {
        let kd = KdTree::build(vec![
            Coord { x: 2, y: 2, z: 2, },
            Coord { x: 2, y: 0, z: 2, },
            Coord { x: 1, y: 1, z: 1, },
            Coord { x: 0, y: 1, z: 1, },
            Coord { x: 1, y: 0, z: 0, },
        ]);

        assert_eq!(
            kd.nearest(&Coord { x: 0, y: 0, z: 0, }).collect::<Vec<_>>(),
            vec![
                (Coord { x: 1, y: 0, z: 0 }, 1),
                (Coord { x: 0, y: 1, z: 1 }, 2),
                (Coord { x: 1, y: 1, z: 1 }, 3),
                (Coord { x: 2, y: 0, z: 2 }, 4),
                (Coord { x: 2, y: 2, z: 2 }, 6),
            ],
        );
    }

    #[test]
    fn la008_tgt_mdl_nearest() {
        use super::super::junk::LA008_TGT_MDL;
        let matrix = super::super::model::read_model(LA008_TGT_MDL).unwrap();
        let kd = KdTree::build(matrix.filled_voxels().cloned());
        let mut neighbours: Vec<_> =
            kd.nearest(&Coord { x: 0, y: 0, z: 0, }).collect();
        neighbours.sort_by_key(|&(coord, dist)| (dist, coord));
        // assert_eq!(neighbours.len(), matrix.filled_voxels().count());
        assert_eq!(
            &neighbours[0 .. 10],
            &[
                (Coord { x: 8, y: 0, z: 3 }, 11),
                (Coord { x: 5, y: 3, z: 4 }, 12),
                (Coord { x: 5, y: 4, z: 3 }, 12),
                (Coord { x: 6, y: 0, z: 6 }, 12),
                (Coord { x: 6, y: 3, z: 3 }, 12),
                (Coord { x: 7, y: 0, z: 5 }, 12),
                (Coord { x: 7, y: 1, z: 4 }, 12),
                (Coord { x: 7, y: 2, z: 3 }, 12),
                (Coord { x: 8, y: 0, z: 4 }, 12),
                (Coord { x: 8, y: 1, z: 3 }, 12),
            ],
        );
    }
}
