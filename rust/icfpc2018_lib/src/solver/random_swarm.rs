use rand::{self, Rng};

use super::super::{
    coord::{
        Coord,
        Matrix,
        Region,
    },
    cmd::BotCommand,
    state::{
        Bid,
        Bot,
    },
    kd,
    router::rtt,
};

const INIT_POS: Coord = Coord { x: 0, y: 0, z: 0, };

#[derive(Clone, PartialEq, Eq, Debug)]
pub enum Error {
    ModelsDimMismatch { source_dim: usize, target_dim: usize, },
    EmptyCommandsBufferForRoute { route: Vec<Coord>, },
    RouteAttempsLimitExceeded(usize),
    GlobalTicksLimitExceeded(usize),
}

pub struct Config {
    init_bots: Vec<(Bid, Bot)>,
    rtt_limit: usize,
    route_attempts_limit: usize,
    global_ticks_limit: usize,
}

pub fn solve(source_model: Matrix, target_model: Matrix, config: Config) -> Result<Vec<BotCommand>, Error> {
    solve_rng(source_model, target_model, config, &mut rand::thread_rng())
}

pub fn solve_rng<R>(source_model: Matrix, target_model: Matrix, config: Config, rng: &mut R) -> Result<Vec<BotCommand>, Error> where R: Rng {
    let source_dim = source_model.dim();
    let target_dim = target_model.dim();
    if source_dim != target_dim {
        return Err(Error::ModelsDimMismatch { source_dim, target_dim, });
    }
    let env = Env::new(source_model, target_model, config);
    let mut current_model = env.source_model.clone();
    let mut commands_buf: Vec<(Coord, BotCommand)> = Vec::new();
    let mut script: Vec<BotCommand> = Vec::new();
    let mut volatiles: Vec<Region> = Vec::new();
    let mut positions: Vec<Coord> = Vec::new();

    let mut nanobots = if env.config.init_bots.is_empty() {
        let (init_bid, init_bot) = Nanobot::init_bot();
        vec![
            Nanobot {
                bid: init_bid,
                bot: init_bot,
                plan: Plan::Init,
            },
        ]
    } else {
        env.config.init_bots.iter()
            .map(|&(bid, ref bot)| Nanobot { bid, bot: bot.clone(), plan: Plan::Init, })
            .collect()
    };

    let mut ticks_count = 0;
    let mut work_complete = false;
    loop {
        ticks_count += 1;
        if ticks_count >= env.config.global_ticks_limit {
            return Err(Error::GlobalTicksLimitExceeded(ticks_count));
        }
        // check for stop condition
        let work_state = if work_complete || current_model.equals(&env.target_model) {
            work_complete = true;
            if nanobots.is_empty() {
                return Ok(script);
            }

            let (mut master, mut slave) = (None, None);
            for nanobot in nanobots.iter() {
                let pos = nanobot.bot.pos;
                if pos == INIT_POS {
                    master = Some(pos);
                } else if pos.diff(&INIT_POS).is_near() {
                    slave = Some(pos);
                }
            }
            WorkState::Completed {
                nanobots_left: nanobots.len(),
                slave_pick: master.and_then(|_| slave),
            }
        } else {
            WorkState::InProgress
        };

        // println!("| loop with bots: {:?}", nanobots);

        volatiles.clear();
        positions.clear();
        positions.extend(nanobots.iter().map(|nanobot| nanobot.bot.pos));

        let mut next_nanobots =
            Vec::with_capacity(nanobots.len());
        for nanobot in nanobots {
            let nanobot_pos = nanobot.bot.pos;
            let implement_result =
                nanobot.implement_plan(
                    &env,
                    &current_model,
                    work_state,
                    |region| if current_model.contains_filled(region) {
                        false
                    } else if volatiles.iter().any(|reg| reg.intersects(region)) {
                        false
                    } else if positions.iter().filter(|&pos| pos != &nanobot_pos).any(|pos| region.contains(pos)) {
                        false
                    } else {
                        true
                    },
                    &mut commands_buf,
                    rng,
                );

            let mut interpret = |nanobot: &mut Nanobot, cmd: &BotCommand| match cmd {
                &BotCommand::Halt |
                &BotCommand::Wait |
                &BotCommand::Flip |
                &BotCommand::Fission { .. } |
                &BotCommand::FusionP{ .. } |
                &BotCommand::FusionS{ .. } |
                &BotCommand::GFill { .. } |
                &BotCommand::GVoid { .. } =>
                    (),
                &BotCommand::SMove { ref long } => {
                    let move_diff = long.to_coord_diff();
                    let move_coord = nanobot.bot.pos.add(move_diff);
                    volatiles.push(Region::from_corners(
                        &move_coord,
                        &nanobot.bot.pos,
                    ));
                    nanobot.bot.pos = move_coord;
                },
                &BotCommand::LMove { ref short1, ref short2, } => {
                    let move_diff_a = short1.to_coord_diff();
                    let move_coord_a = nanobot.bot.pos.add(move_diff_a);
                    volatiles.push(Region::from_corners(
                        &move_coord_a,
                        &nanobot.bot.pos,
                    ));
                    let move_diff_b = short2.to_coord_diff();
                    let move_coord_b = move_coord_a.add(move_diff_b);
                    volatiles.push(Region::from_corners(
                        &move_coord_b,
                        &move_coord_a,
                    ));
                    nanobot.bot.pos = move_coord_b;
                },
                &BotCommand::Fill { near, } => {
                    let fill_coord = nanobot.bot.pos.add(near);
                    current_model.set_filled(&fill_coord);
                },
                &BotCommand::Void{ near, } => {
                    let void_coord = nanobot.bot.pos.add(near);
                    current_model.set_void(&void_coord);
                },
            };

            match implement_result {
                PlanResult::DoAndPerish(cmd) =>
                    script.push(cmd),
                PlanResult::Regular { mut nanobot, cmd, } => {
                    interpret(&mut nanobot, &cmd);
                    script.push(cmd);
                    next_nanobots.push(nanobot);
                },
                PlanResult::Spawn { mut parent, child, cmd, } => {
                    interpret(&mut parent, &cmd);
                    script.push(cmd);
                    next_nanobots.push(parent);
                    next_nanobots.push(child);
                },
                PlanResult::Error(error) =>
                    return Err(error),
            }
        }
        nanobots = next_nanobots;
        nanobots.sort_by_key(|nanobot| nanobot.bid);
    }
}

struct Env {
    source_model: Matrix,
    target_model: Matrix,
    source_kd: kd::KdTree,
    target_kd: kd::KdTree,
    config: Config,
}

impl Env {
    fn new(source_model: Matrix, target_model: Matrix, config: Config) -> Env {
        let source_kd = kd::KdTree::build(source_model.filled_voxels().cloned());
        let target_kd = kd::KdTree::build(target_model.filled_voxels().cloned());
        Env {
            source_model,
            target_model,
            source_kd,
            target_kd,
            config,
        }
    }
}

#[derive(Clone, PartialEq, Eq, Debug)]
struct Nanobot {
    bid: Bid,
    bot: Bot,
    plan: Plan,
}

#[derive(Clone, PartialEq, Eq, Debug)]
enum Plan {
    Init,
    HeadingFor { target: Coord, attempts: usize, },
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum WorkState {
    InProgress,
    Completed {
        nanobots_left: usize,
        slave_pick: Option<Coord>,
    },
}

enum PlanResult {
    DoAndPerish(BotCommand),
    Regular { nanobot: Nanobot, cmd: BotCommand, },
    Spawn { parent: Nanobot, child: Nanobot, cmd: BotCommand, },
    Error(Error),
}

impl Nanobot {
    fn init_bot() -> (Bid, Bot) {
        (1, Bot {
            pos: INIT_POS,
            seeds: (2 ..= 40).collect(),
        })
    }

    fn implement_plan<FP, R>(
        mut self,
        env: &Env,
        current_model: &Matrix,
        work_state: WorkState,
        is_passable: FP,
        commands_buf: &mut Vec<(Coord, BotCommand)>,
        rng: &mut R,
    )
        -> PlanResult where FP: Fn(&Region) -> bool, R: Rng,
    {
        match work_state {
            WorkState::InProgress =>
                (),
            WorkState::Completed { nanobots_left, slave_pick, } =>
                if self.bot.pos == INIT_POS {
                    // i am the master
                    return if nanobots_left == 1 {
                        PlanResult::DoAndPerish(BotCommand::Halt)
                    } else if let Some(slave_coord) = slave_pick {
                        let fusion_cmd = BotCommand::FusionP {
                            near: slave_coord.diff(&self.bot.pos),
                        };
                        PlanResult::Regular { nanobot: self, cmd: fusion_cmd, }
                    } else {
                        PlanResult::Regular { nanobot: self, cmd: BotCommand::Wait, }
                    };
                } else {
                    match slave_pick {
                        Some(coord) if coord == self.bot.pos =>
                            return PlanResult::DoAndPerish(BotCommand::FusionS {
                                near: INIT_POS.diff(&self.bot.pos),
                            }),
                        _ =>
                            self.plan = Plan::HeadingFor { target: INIT_POS, attempts: 0, },
                    };
                },
        }
        loop {
            match self.plan {
                Plan::Init => {
                    // go somewhere
                    let target = pick_random_coord(current_model.dim() as isize, rng);
                    self.plan = Plan::HeadingFor { target, attempts: 0, };
                },
                Plan::HeadingFor { ref target, .. } if target == &self.bot.pos => {
                    // reached a target
                    unimplemented!()
                },
                Plan::HeadingFor { attempts, .. } if attempts > env.config.route_attempts_limit =>
                    return PlanResult::Error(Error::RouteAttempsLimitExceeded(attempts)),
                Plan::HeadingFor { target, attempts, } => {
                    // still moving to target
                    let route_result =
                        route_and_step(&self.bot.pos, &target, current_model, &is_passable, commands_buf, env.config.rtt_limit, rng);
                    match route_result {
                        Ok(Some(moving_cmd)) => {
                            // can continue moving
                            return PlanResult::Regular { nanobot: self, cmd: moving_cmd, };
                        },
                        Ok(None) => {
                            // can not move there
                            match work_state {
                                WorkState::InProgress => {
                                    // pick another wandering target
                                    let target = pick_random_coord(current_model.dim() as isize, rng);
                                    self.plan = Plan::HeadingFor { target, attempts: attempts + 1, };
                                },
                                WorkState::Completed { .. } => {
                                    // try to find a free position nearby
                                    let next_attempts = attempts + 1;
                                    let offset = (next_attempts / 3) as isize;
                                    let axis = next_attempts % 3;
                                    self.plan = Plan::HeadingFor {
                                        target: Coord {
                                            x: target.x + if axis == 0 { offset } else { 0 },
                                            y: target.y + if axis == 1 { offset } else { 0 },
                                            z: target.z + if axis == 2 { offset } else { 0 },
                                        },
                                        attempts: next_attempts,
                                    };
                                },
                            }
                        },
                        Err(error) =>
                            return PlanResult::Error(error),
                    }
                },
            }
        }
    }
}

fn pick_random_coord<R>(dim: isize, rng: &mut R) -> Coord where R: Rng {
    Coord {
        x: rng.gen_range(0, dim),
        y: rng.gen_range(0, dim),
        z: rng.gen_range(0, dim),
    }
}

fn route_and_step<FP, R>(
    start: &Coord,
    finish: &Coord,
    current_model: &Matrix,
    is_passable: FP,
    commands_buf: &mut Vec<(Coord, BotCommand)>,
    rtt_limit: usize,
    rng: &mut R,
)
    -> Result<Option<BotCommand>, Error> where
    FP: Fn(&Region) -> bool,
    R: Rng,
{
    let maybe_route = rtt::plan_route_rng(
        start,
        finish,
        current_model.dim(),
        is_passable,
        rtt_limit,
        rng,
    );
    Ok(if let Some(route) = maybe_route {
        rtt::plan_route_commands(&route, commands_buf);
        if commands_buf.is_empty() {
            return Err(Error::EmptyCommandsBufferForRoute { route, });
        }
        let (_move_coord, move_command) =
            commands_buf.swap_remove(0);
        Some(move_command)
    } else {
        None
    })
}

#[cfg(test)]
mod test {
    use super::super::super::{
        coord::{
            Axis,
            Coord,
            Matrix,
            CoordDiff,
            Resolution,
            LinearCoordDiff,
        },
        state::Bot,
        cmd::BotCommand,
    };
    use super::{
        Nanobot,
    };

    #[test]
    fn nanobot_init_bot() {
        assert_eq!(
            Nanobot::init_bot(),
            (1, Bot {
                pos: Coord {
                    x: 0,
                    y: 0,
                    z: 0,
                },
                seeds: vec![
                    2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,
                    13, 14, 15, 16, 17, 18, 19, 20, 21, 22,
                    23, 24, 25, 26, 27, 28, 29, 30, 31, 32,
                    33, 34, 35, 36, 37, 38, 39, 40,
                ],
            })
        );
    }

    #[test]
    fn solve_empty() {
        let source_model = Matrix::from_iter(Resolution(3), vec![]);
        let target_model = Matrix::from_iter(Resolution(3), vec![]);
        let script = super::solve(source_model, target_model, super::Config {
            init_bots: vec![],
            rtt_limit: 64,
            route_attempts_limit: 16,
            global_ticks_limit: 100,
        }).unwrap();
        assert_eq!(script, vec![BotCommand::Halt]);
    }

    #[test]
    fn solve_move_and_halt() {
        use rand::{SeedableRng, prng::XorShiftRng};
        let mut rng: XorShiftRng =
            SeedableRng::from_seed([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
        let source_model = Matrix::from_iter(Resolution(3), vec![]);
        let target_model = Matrix::from_iter(Resolution(3), vec![]);
        let script = super::solve_rng(
            source_model,
            target_model,
            super::Config {
                init_bots: vec![(1, Bot { pos: Coord { x: 1, y: 0, z: 0, }, seeds: vec![], })],
                rtt_limit: 64,
                route_attempts_limit: 16,
                global_ticks_limit: 100,
            },
            &mut rng,
        ).unwrap();
        assert_eq!(
            script,
            vec![
                BotCommand::SMove { long: LinearCoordDiff::Long { axis: Axis::X, value: -1 } },
                BotCommand::Halt
            ],
        );
    }

    #[test]
    fn solve_fusion_2_and_halt() {
        use rand::{SeedableRng, prng::XorShiftRng};
        let mut rng: XorShiftRng =
            SeedableRng::from_seed([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
        let source_model = Matrix::from_iter(Resolution(3), vec![]);
        let target_model = Matrix::from_iter(Resolution(3), vec![]);
        let script = super::solve_rng(
            source_model,
            target_model,
            super::Config {
                init_bots: vec![
                    (1, Bot { pos: Coord { x: 0, y: 1, z: 1, }, seeds: vec![], }),
                    (2, Bot { pos: Coord { x: 2, y: 2, z: 2, }, seeds: vec![], }),
                    ],
                rtt_limit: 64,
                route_attempts_limit: 16,
                global_ticks_limit: 100,
            },
            &mut rng,
        ).unwrap();
        assert_eq!(
            script,
            vec![
                BotCommand::LMove {
                    short1: LinearCoordDiff::Short { axis: Axis::Z, value: -1 },
                    short2: LinearCoordDiff::Short { axis: Axis::Y, value: -1 },
                },
                BotCommand::LMove {
                    short1: LinearCoordDiff::Short { axis: Axis::Y, value: -2 },
                    short2: LinearCoordDiff::Short { axis: Axis::Z, value: -2 },
                },

                BotCommand::Wait,
                BotCommand::SMove { long: LinearCoordDiff::Long { axis: Axis::X, value: -1 } },

                BotCommand::FusionP { near: CoordDiff(Coord { x: 1, y: 0, z: 0 }) },
                BotCommand::FusionS { near: CoordDiff(Coord { x: -1, y: 0, z: 0 }) },

                BotCommand::Halt
            ],
        );
    }

    #[test]
    fn solve_fusion_3_and_halt() {
        use rand::{SeedableRng, prng::XorShiftRng};
        let mut rng: XorShiftRng =
            SeedableRng::from_seed([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
        let source_model = Matrix::from_iter(Resolution(3), vec![]);
        let target_model = Matrix::from_iter(Resolution(3), vec![]);
        let script = super::solve_rng(
            source_model,
            target_model,
            super::Config {
                init_bots: vec![
                    (1, Bot { pos: Coord { x: 0, y: 1, z: 1, }, seeds: vec![], }),
                    (2, Bot { pos: Coord { x: 2, y: 2, z: 2, }, seeds: vec![], }),
                    (3, Bot { pos: Coord { x: 0, y: 2, z: 0, }, seeds: vec![], }),
                    ],
                rtt_limit: 64,
                route_attempts_limit: 16,
                global_ticks_limit: 100,
            },
            &mut rng,
        ).unwrap();
        assert_eq!(
            script,
            vec![
                BotCommand::LMove {
                    short1: LinearCoordDiff::Short { axis: Axis::Z, value: -1 },
                    short2: LinearCoordDiff::Short { axis: Axis::Y, value: -1 },
                },
                BotCommand::LMove {
                    short1: LinearCoordDiff::Short { axis: Axis::Y, value: -2 },
                    short2: LinearCoordDiff::Short { axis: Axis::Z, value: -2 },
                },
                BotCommand::LMove {
                    short1: LinearCoordDiff::Short { axis: Axis::X, value: 1 },
                    short2: LinearCoordDiff::Short { axis: Axis::Y, value: -2 },
                },

                BotCommand::FusionP { near: CoordDiff(Coord { x: 1, y: 0, z: 0 }) },
                BotCommand::LMove {
                    short1: LinearCoordDiff::Short { axis: Axis::Y, value: 1 },
                    short2: LinearCoordDiff::Short { axis: Axis::X, value: -1 },
                },
                BotCommand::FusionS { near: CoordDiff(Coord { x: -1, y: 0, z: 0 }) },

                BotCommand::FusionP { near: CoordDiff(Coord { x: 1, y: 1, z: 0 }) },
                BotCommand::FusionS { near: CoordDiff(Coord { x: -1, y: -1, z: 0 }) },

                BotCommand::Halt
            ],
        );
    }
}
