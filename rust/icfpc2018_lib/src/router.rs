use super::{
    coord::{
        M,
        Axis,
        Coord,
        CoordDiff,
        Region,
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
    fn moves_allowed<'a>(&self, state: &State, volatile: &'a [Region]) -> impl Iterator<Item = Move> + 'a {
        let coord = self.coord;
        let dim = state.matrix.dim() as isize;
        smove_iter(coord, Axis::X, -15 .. -1)
            .chain(smove_iter(coord, Axis::X, 1 .. 15))
            .chain(smove_iter(coord, Axis::Y, -15 .. -1))
            .chain(smove_iter(coord, Axis::Y, 1 .. 15))
            .chain(smove_iter(coord, Axis::Z, -15 .. -1))
            .chain(smove_iter(coord, Axis::Z, 1 .. 15))
            .map(|(cmd, coord)| Move { coord, cmd_performed: Some(cmd), })
            .filter(move |mv| mv.coord.x >= 0 && mv.coord.x <= dim)
            .filter(move |mv| mv.coord.y >= 0 && mv.coord.y <= dim)
            .filter(move |mv| mv.coord.z >= 0 && mv.coord.z <= dim)
            .filter(move |mv| !volatile.iter().any(move |region| region.contains(&mv.coord)))
    }
}

fn smove_iter<I>(coord: Coord, axis: Axis, offsets: I) -> impl Iterator<Item = (BotCommand, Coord)> where I: Iterator<Item = M> {
    offsets.map(move |value| (
        BotCommand::SMove {
            long: LinearCoordDiff::Long { axis, value, },
        },
        coord.add(CoordDiff(match axis {
            Axis::X => Coord { x: value, y: 0, z: 0, },
            Axis::Y => Coord { x: 0, y: value, z: 0, },
            Axis::Z => Coord { x: 0, y: 0, z: value, },
        })),
    ))
}
