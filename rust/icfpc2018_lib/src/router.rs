use std::iter;
use super::{
    coord::{
        M,
        Axis,
        Coord,
        Region,
        Matrix,
        CoordDiff,
        LinearCoordDiff,
    },
    state::State,
    cmd::BotCommand,
};
use pathfinding::directed::astar;

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub struct Move {
    pub coord: Coord,
    pub cmd_performed: Option<BotCommand>,
}

pub fn plan_route(bot_start: &Coord, bot_finish: &Coord, state: &State, volatile: &[Region]) -> Option<Vec<BotCommand>> {

    // astar::astar(
    //     coord,
    //     |coord| self.filled_near_neighbours(coord)
    //         .map(|neighbour| (neighbour, neighbour.y)),
    //     |coord| coord.y,
    //     |coord| coord.y == 0,
    // )

    unimplemented!()
}

impl Move {
    fn moves_allowed<'a, VI>(&self, matrix: &'a Matrix, volatile: VI) -> impl Iterator<Item = Move> + 'a where
        VI: Iterator<Item = Region> + Clone + 'a
    {
        let coord = self.coord;
        let dim = matrix.dim() as isize;
        smove_iter(coord, 1, Axis::X, -15 .. -1)
            .chain(smove_iter(coord, -1, Axis::X, 1 .. 15))
            .chain(smove_iter(coord, 1, Axis::Y, -15 .. -1))
            .chain(smove_iter(coord, -1, Axis::Y, 1 .. 15))
            .chain(smove_iter(coord, 1, Axis::Z, -15 .. -1))
            .chain(smove_iter(coord, -1, Axis::Z, 1 .. 15))
            .map(|(cmd, coord, more_volatiles)| (Move { coord, cmd_performed: Some(cmd), }, more_volatiles))
            .filter(move |mv| mv.0.coord.x >= 0 && mv.0.coord.x < dim)
            .filter(move |mv| mv.0.coord.y >= 0 && mv.0.coord.y < dim)
            .filter(move |mv| mv.0.coord.z >= 0 && mv.0.coord.z < dim)
            .filter(move |mv| !matrix.is_filled(&mv.0.coord))
            .filter(move |&(ref mv, ref more_volatiles)| {
                !volatile.clone().chain(more_volatiles.clone())
                    .any(|region| region.contains(&mv.coord) || matrix.contains_filled(&region))
            })
            .map(|mv| mv.0)
    }
}

fn smove_iter<I>(
    coord: Coord,
    shift: isize,
    axis: Axis,
    offsets: I
)
    -> impl Iterator<Item = (BotCommand, Coord, impl Iterator<Item = Region> + Clone)> where
    I: Iterator<Item = M>
{
    offsets.map(move |value| {
        let mk_coord = |value| coord.add(CoordDiff(match axis {
            Axis::X => Coord { x: value, y: 0, z: 0, },
            Axis::Y => Coord { x: 0, y: value, z: 0, },
            Axis::Z => Coord { x: 0, y: 0, z: value, },
        }));
        (
            BotCommand::SMove {
                long: LinearCoordDiff::Long { axis, value, },
            },
            mk_coord(value),
            iter::once(Region::from_corners(&coord, &mk_coord(value + shift))),
        )
    })
}

#[cfg(test)]
mod tests {
    use super::super::{
        coord::{
            Matrix,
            Resolution,
            Coord,
            LinearCoordDiff,
            Axis,
            Region,
        },
        cmd::BotCommand,
    };
    use super::Move;

    #[test]
    fn moves_allowed_empty() {
        let matrix = Matrix::from_iter(Resolution(3), vec![]);
        let start = Move { coord: Coord { x: 0, y: 0, z: 0, }, cmd_performed: None, };
        let moves: Vec<_> = start.moves_allowed(&matrix, None.into_iter()).collect();
        assert_eq!(moves, vec![
            Move {
                coord: Coord { x: 1, y: 0, z: 0 },
                cmd_performed: Some(BotCommand::SMove { long: LinearCoordDiff::Long { axis: Axis::X, value: 1 } }),
            },
            Move {
                coord: Coord { x: 2, y: 0, z: 0 },
                cmd_performed: Some(BotCommand::SMove { long: LinearCoordDiff::Long { axis: Axis::X, value: 2 } }),
            },
            Move {
                coord: Coord { x: 0, y: 1, z: 0 },
                cmd_performed: Some(BotCommand::SMove { long: LinearCoordDiff::Long { axis: Axis::Y, value: 1 } }),
            },
            Move {
                coord: Coord { x: 0, y: 2, z: 0 },
                cmd_performed: Some(BotCommand::SMove { long: LinearCoordDiff::Long { axis: Axis::Y, value: 2 } }),
            },
            Move {
                coord: Coord { x: 0, y: 0, z: 1 },
                cmd_performed: Some(BotCommand::SMove { long: LinearCoordDiff::Long { axis: Axis::Z, value: 1 } }),
            },
            Move {
                coord: Coord { x: 0, y: 0, z: 2 },
                cmd_performed: Some(BotCommand::SMove { long: LinearCoordDiff::Long { axis: Axis::Z, value: 2 } }),
            },
        ]);
    }

    #[test]
    fn moves_allowed_1_filled() {
        let matrix = Matrix::from_iter(Resolution(3), vec![Coord { x: 1, y: 0, z: 0, }]);
        let start = Move { coord: Coord { x: 0, y: 0, z: 0, }, cmd_performed: None, };
        let moves: Vec<_> = start.moves_allowed(&matrix, None.into_iter()).collect();
        assert_eq!(moves, vec![
            Move {
                coord: Coord { x: 0, y: 1, z: 0 },
                cmd_performed: Some(BotCommand::SMove { long: LinearCoordDiff::Long { axis: Axis::Y, value: 1 } }),
            },
            Move {
                coord: Coord { x: 0, y: 2, z: 0 },
                cmd_performed: Some(BotCommand::SMove { long: LinearCoordDiff::Long { axis: Axis::Y, value: 2 } }),
            },
            Move {
                coord: Coord { x: 0, y: 0, z: 1 },
                cmd_performed: Some(BotCommand::SMove { long: LinearCoordDiff::Long { axis: Axis::Z, value: 1 } }),
            },
            Move {
                coord: Coord { x: 0, y: 0, z: 2 },
                cmd_performed: Some(BotCommand::SMove { long: LinearCoordDiff::Long { axis: Axis::Z, value: 2 } }),
            },
        ]);
    }

    #[test]
    fn moves_allowed_1_filled_1_volatile() {
        let matrix = Matrix::from_iter(Resolution(3), vec![Coord { x: 1, y: 0, z: 0, }]);
        let start = Move { coord: Coord { x: 0, y: 0, z: 0, }, cmd_performed: None, };
        let moves: Vec<_> = start.moves_allowed(
            &matrix,
            Some(Region { min: Coord { x: 0, y: 1, z: 0, }, max: Coord { x: 0, y: 1, z: 0, }, }).into_iter(),
        ).collect();
        assert_eq!(moves, vec![
            Move {
                coord: Coord { x: 0, y: 0, z: 1 },
                cmd_performed: Some(BotCommand::SMove { long: LinearCoordDiff::Long { axis: Axis::Z, value: 1 } }),
            },
            Move {
                coord: Coord { x: 0, y: 0, z: 2 },
                cmd_performed: Some(BotCommand::SMove { long: LinearCoordDiff::Long { axis: Axis::Z, value: 2 } }),
            },
        ]);
    }
}
