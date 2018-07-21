use super::{
    coord::Coord,
    state::State,
    cmd::BotCommand,
};
use pathfinding::directed::astar;

pub fn plan_route(bot_start: &Coord, bot_finish: &Coord, state: &State) -> Option<Vec<BotCommand>> {


    unimplemented!()
}
