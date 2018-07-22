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
        CoordDiff,
        Region,
        Matrix,
    },
    // cmd::BotCommand,
};

pub fn plan_route<VI>(
    &bot_start: &Coord,
    &bot_finish: &Coord,
    matrix: &Matrix,
    volatile: VI,
    max_iters: usize,
)
    -> Option<(Vec<()>, usize)> where
    VI: Iterator<Item = Region> + Clone
{
    let mut rng = rand::thread_rng();
    let mut visited_regions = Vec::new();
    let mut iters = 0;

    let planner = rtt::PlannerInit::new(EmptyRandomTree::new());
    let planner = planner.add_root_ok(|empty_rtt: EmptyRandomTree<Coord>| Ok(empty_rtt.add_root(bot_start)));
    let mut planner_node = planner.root_node_ok(|rtt: &mut RandomTree<Coord>| {
        let root_ref = rtt.root();
        Ok(RttNodeFocus { node_ref: root_ref, goal_reached: false, })
    });

    let rev_path = loop {
        if planner_node.node_ref().goal_reached {
            break planner_node.into_path_ok(
                |rtt: RandomTree<_>, focus: RttNodeFocus| Ok(rtt.into_path(focus.node_ref))
            );
        }
        let mut planner_ready_to_sample = planner_node.prepare_sample_ok(|_rtt: &mut _, _focus| Ok(()));

        let mut first_time = true;
        loop {
            if iters >= max_iters {
                return None;
            }
            iters += 1;

            let planner_sample = planner_ready_to_sample.sample_ok(|_rtt: &mut _| if first_time {
                Ok(bot_finish)
            } else {
                let dim = matrix.dim() as isize;
                Ok(Coord {
                    x: rng.gen_range(0, dim),
                    y: rng.gen_range(0, dim),
                    z: rng.gen_range(0, dim),
                })
            });
            first_time = false;
            let planner_closest = planner_sample.closest_to_sample_ok(|rtt: &mut RandomTree<Coord>, sample: &_| {
                let mut closest;
                {
                    let states = rtt.states();
                    closest = (states.root.0, states.root.1.diff(sample).l_1_norm());
                    for (node_ref, mv) in states.children {
                        let dist = mv.diff(sample).l_1_norm();
                        if dist < closest.1 {
                            closest = (node_ref, dist);
                        }
                    }
                }
                Ok(RttNodeFocus { node_ref: closest.0, goal_reached: false, })
            });

            let maybe_route = {
                let rtt = planner_closest.rtt();
                let node_ref = &planner_closest.node_ref().node_ref;
                let &dst = planner_closest.sample();
                let &src = rtt.get_state(node_ref);
                random_valid_edge_path(
                    src, dst, matrix,
                    visited_regions.iter().cloned().chain(volatile.clone()),
                    &mut rng,
                )
            };

            if let Some((jump, ra, rb, rc)) = maybe_route {
                planner_node =
                    planner_closest.has_transition_ok(|rtt: &mut RandomTree<Coord>, focus: RttNodeFocus, _dst| {
                        visited_regions.push(ra);
                        visited_regions.push(rb);
                        visited_regions.push(rc);
                        let mut node_ref = focus.node_ref;
                        node_ref = rtt.expand(node_ref, jump.mid_a);
                        node_ref = rtt.expand(node_ref, jump.mid_b);
                        node_ref = rtt.expand(node_ref, jump.finish);
                        Ok(RttNodeFocus {
                            node_ref,
                            goal_reached: jump.finish == bot_finish,
                        })
                    });
                break;
            }
            planner_ready_to_sample =
                planner_closest.no_transition_ok(|_rtt: &mut _, _node_ref| Ok(()));
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

fn random_valid_edge_path<VI, R>(
    start: Coord,
    finish: Coord,
    matrix: &Matrix,
    volatile: VI,
    rng: &mut R,
)
    -> Option<(EdgesJump, Region, Region, Region)> where
    VI: Iterator<Item = Region> + Clone,
    R: Rng,
{
    random_edge_paths(start, finish, rng)
        .map(|jump| {
            let CoordDiff(delta) = jump.mid_b.diff(&jump.finish);
            let fix_finish = CoordDiff(Coord {
                x: if delta.x < 0 { -1 } else if delta.x > 0 { 1 } else { 0 },
                y: if delta.y < 0 { -1 } else if delta.y > 0 { 1 } else { 0 },
                z: if delta.z < 0 { -1 } else if delta.z > 0 { 1 } else { 0 },
            });
            let ra = Region::from_corners(&jump.start, &jump.mid_a);
            let rb = Region::from_corners(&jump.mid_a, &jump.mid_b);
            let rc = Region::from_corners(&jump.mid_b, &jump.finish.add(fix_finish));
            (jump, ra, rb, rc)
        })
        .filter(move |&(_, ref ra, ref rb, ref rc)| {
            !volatile
                .clone()
                .any(|reg| reg.intersects(&ra) || reg.intersects(&rb) || reg.intersects(rc))
                && !matrix.contains_filled(&ra)
                && !matrix.contains_filled(&rb)
                && !matrix.contains_filled(&rc)
        })
        .next()
}

#[cfg(test)]
mod test {
    use rand;
    use super::super::super::coord::{Coord, Matrix, Region, Resolution};
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

    #[test]
    fn random_valid_edge_path() {
        let matrix = Matrix::from_iter(Resolution(3), vec![
            Coord { x: 1, y: 0, z: 0, },
            Coord { x: 0, y: 0, z: 1, },
        ]);
        let path = super::random_valid_edge_path(
            Coord { x: 0, y: 0, z: 0, },
            Coord { x: 2, y: 2, z: 2, },
            &matrix,
            Some(Region::from_corners(
                &Coord { x: 1, y: 2, z: 0, },
                &Coord { x: 2, y: 2, z: 0, },
            )).into_iter(),
            &mut rand::thread_rng(),
        );
        assert_eq!(path, Some((
            EdgesJump {
                start: Coord { x: 0, y: 0, z: 0 },
                mid_a: Coord { x: 0, y: 2, z: 0 },
                mid_b: Coord { x: 0, y: 2, z: 2 },
                finish: Coord { x: 2, y: 2, z: 2 },
            },
            Region { min: Coord { x: 0, y: 0, z: 0 }, max: Coord { x: 0, y: 2, z: 0 } },
            Region { min: Coord { x: 0, y: 2, z: 0 }, max: Coord { x: 0, y: 2, z: 2 } },
            Region { min: Coord { x: 0, y: 2, z: 2 }, max: Coord { x: 1, y: 2, z: 2 } },
        )));
    }
}
