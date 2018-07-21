use std::collections::HashSet;
use rand;

use rtt::{
    self,
    util::rtt::vec_slist::{
        EmptyRandomTree,
        RandomTree,
        NodeRef,
    },
};

use super::super::{
    coord::{
        Coord,
        Region,
        Matrix,
    },
    cmd::BotCommand,
};

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub struct Move {
    pub coord: Coord,
    pub cmd_performed: Option<BotCommand>,
}

pub fn plan_route<VI>(
    &bot_start: &Coord,
    bot_finish: &Coord,
    matrix: &Matrix,
    volatile: VI,
    max_iters: usize,
)
    -> Option<(Vec<Move>, usize)> where
    VI: Iterator<Item = Region> + Clone
{
    let start = Move { coord: bot_start, cmd_performed: None, };

    let mut rng = rand::thread_rng();
    let mut visited = HashSet::new();
    let mut iters = 0;

    let planner = rtt::PlannerInit::new(EmptyRandomTree::new());
    let planner = planner.add_root_ok(|empty_rtt: EmptyRandomTree<Move>| Ok(empty_rtt.add_root(start)));
    let mut planner_node = planner.root_node_ok(|rtt: &mut RandomTree<Move>| {
        let root_ref = rtt.root();
        visited.insert(start);
        Ok(RttNodeFocus { node_ref: root_ref, goal_reached: false, })
    });

    let rev_path = loop {
        if planner_node.node_ref().goal_reached {
            break planner_node.into_path_ok(
                |rtt: RandomTree<_>, focus: RttNodeFocus| Ok(rtt.into_path(focus.node_ref))
            );
        }
        let mut planner_ready_to_sample = planner_node.prepare_sample_ok(|_rtt: &mut _, _focus| Ok(()));

        loop {
            if iters >= max_iters {
                return None;
            }
            iters += 1;

            // let planner_sample = planner_ready_to_sample.sample_ok(|_rtt: &mut _| {
            //     Ok((rng.gen_range(0, height), rng.gen_range(0, width)))
            // });
            // let planner_closest = planner_sample.closest_to_sample_ok(locate_closest);

            // let route = {
            //     let rtt = planner_closest.rtt();
            //     let node_ref = &planner_closest.node_ref().node_ref;
            //     let dst = planner_closest.sample();
            //     StraightPathIter::new(rtt.get_state(node_ref), dst)
            // };

            // if let Some(path_iter) = route {
            //     let blocked = path_iter.clone().any(|coord| {
            //         maze[coord.0][coord.1] == b'#' || visited.contains(&coord)
            //     });
            //     if !blocked {
            //         planner_node =
            //             planner_closest.has_transition_ok(
            //                 |rtt: &mut _, focus: RttNodeFocus, _sample| perform_move(rtt, focus.node_ref, path_iter, &finish, &mut visited)
            //             );
            //         break;
            //     }
            // }
            // planner_ready_to_sample =
            //     planner_closest.no_transition_ok(|_rtt: &mut _, _node_ref| Ok(()));
        }
    };

    unimplemented!()
}

struct RttNodeFocus {
    node_ref: NodeRef,
    goal_reached: bool,
}
