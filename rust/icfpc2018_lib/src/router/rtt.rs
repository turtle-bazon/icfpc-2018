use super::super::{
    coord::{
        Coord,
        Region,
        Matrix,
    },
};

pub struct Move;

pub fn plan_route<VI>(&bot_start: &Coord, bot_finish: &Coord, matrix: &Matrix, volatile: VI) -> Option<(Vec<Move>, usize)> where
    VI: Iterator<Item = Region> + Clone
{
    unimplemented!()
}
