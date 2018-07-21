use std::collections::HashSet;
use rand::{self, Rng};

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

            let planner_sample = planner_ready_to_sample.sample_ok(|_rtt: &mut _| {
                let dim = matrix.dim() as isize;
                Ok(Coord {
                    x: rng.gen_range(0, dim),
                    y: rng.gen_range(0, dim),
                    z: rng.gen_range(0, dim),
                })
            });
            let planner_closest = planner_sample.closest_to_sample_ok(|rtt: &mut RandomTree<Move>, sample: &_| {
                let mut closest;
                {
                    let states = rtt.states();
                    closest = (states.root.0, states.root.1.coord.diff(sample).l_1_norm());
                    for (node_ref, mv) in states.children {
                        let dist = mv.coord.diff(sample).l_1_norm();
                        if dist < closest.1 {
                            closest = (node_ref, dist);
                        }
                    }
                }
                Ok(RttNodeFocus { node_ref: closest.0, goal_reached: false, })
            });

            // let route = {
            let rtt = planner_closest.rtt();
            let node_ref = &planner_closest.node_ref().node_ref;
            let dst = planner_closest.sample();
            let src = rtt.get_state(node_ref).coord;
            unimplemented!()
                //StraightPathIter::new(rtt.get_state(node_ref), dst)

            //};

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

#[derive(PartialEq, Eq, Debug)]
struct EdgesJump {
    start: Coord,
    mid_a: Coord,
    mid_b: Coord,
    finish: Coord,
}

fn random_edge_paths<R>(start: Coord, finish: Coord, rng: &mut R) -> impl Iterator<Item = EdgesJump> where R: Rng {
    let table = [
        ((finish.x, start.y, start.z), (finish.x, finish.y, start.z)),
        ((finish.x, start.y, start.z), (finish.x, start.y, finish.z)),
        ((start.x, finish.y, start.z), (finish.x, finish.y, start.z)),
        ((start.x, finish.y, start.z), (start.x, finish.y, finish.z)),
        ((start.x, start.y, finish.z), (finish.x, start.y, finish.z)),
        ((start.x, start.y, finish.z), (start.x, finish.y, finish.z)),
    ];

    let mut picks = [0, 1, 2, 3, 4, 5];
    rng.shuffle(&mut picks);
    (0 .. 6)
        .map(move |index| picks[index])
        .map(move |choice| EdgesJump {
            start, finish,
            mid_a: Coord { x: (table[choice].0).0, y: (table[choice].0).1, z: (table[choice].0).2, },
            mid_b: Coord { x: (table[choice].1).0, y: (table[choice].1).1, z: (table[choice].1).2, },
        })
}

#[cfg(test)]
mod test {
    use rand;
    use super::super::super::coord::Coord;
    use super::EdgesJump;

    #[test]
    fn random_edge_paths() {
        let paths: Vec<_> = super::random_edge_paths(
            Coord { x: 0, y: 0, z: 0, },
            Coord { x: 2, y: 2, z: 2, },
            &mut rand::thread_rng(),
        ).collect();
        assert!(paths.contains(&EdgesJump {
            start: Coord { x: 0, y: 0, z: 0, },
            finish: Coord { x: 2, y: 2, z: 2, },
            mid_a: Coord { x: 2, y: 0, z: 0, },
            mid_b: Coord { x: 2, y: 2, z: 0, },
        }));
        assert!(paths.contains(&EdgesJump {
            start: Coord { x: 0, y: 0, z: 0, },
            finish: Coord { x: 2, y: 2, z: 2, },
            mid_a: Coord { x: 2, y: 0, z: 0, },
            mid_b: Coord { x: 2, y: 0, z: 2, },
        }));
        assert!(paths.contains(&EdgesJump {
            start: Coord { x: 0, y: 0, z: 0, },
            finish: Coord { x: 2, y: 2, z: 2, },
            mid_a: Coord { x: 0, y: 2, z: 0, },
            mid_b: Coord { x: 2, y: 2, z: 0, },
        }));
        assert!(paths.contains(&EdgesJump {
            start: Coord { x: 0, y: 0, z: 0, },
            finish: Coord { x: 2, y: 2, z: 2, },
            mid_a: Coord { x: 0, y: 2, z: 0, },
            mid_b: Coord { x: 0, y: 2, z: 2, },
        }));
        assert!(paths.contains(&EdgesJump {
            start: Coord { x: 0, y: 0, z: 0, },
            finish: Coord { x: 2, y: 2, z: 2, },
            mid_a: Coord { x: 0, y: 0, z: 2, },
            mid_b: Coord { x: 2, y: 0, z: 2, },
        }));
        assert!(paths.contains(&EdgesJump {
            start: Coord { x: 0, y: 0, z: 0, },
            finish: Coord { x: 2, y: 2, z: 2, },
            mid_a: Coord { x: 0, y: 0, z: 2, },
            mid_b: Coord { x: 0, y: 2, z: 2, },
        }));
    }
}
