use std::collections::HashSet;
use rand::{self, Rng};

use super::super::{
    coord::{
        self,
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
    RouteAttempsLimitExceeded { source: Coord, target: Coord, attempts: usize, },
    GlobalTicksLimitExceeded { ticks: usize, script_so_far: Vec<BotCommand>, voxels_to_do: usize, },
}

pub struct Config {
    pub init_bots: Vec<(Bid, Bot)>,
    pub rtt_limit: usize,
    pub route_attempts_limit: usize,
    pub global_ticks_limit: usize,
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
            let mut voxels_to_do = 0;

            println!(" ;; source model filled voxels = {}", env.source_model.filled_voxels().count());
            println!(" ;; target model filled voxels = {}", env.target_model.filled_voxels().count());
            println!(" ;; current model filled voxels = {}", current_model.filled_voxels().count());

            for voxel in env.source_model.filled_voxels() {
                if current_model.is_filled(voxel) && !env.target_model.is_filled(voxel) {
                    voxels_to_do += 1;
                }
            }
            for voxel in env.target_model.filled_voxels() {
                if !current_model.is_filled(voxel) {
                    voxels_to_do += 1;
                }
            }
            return Err(Error::GlobalTicksLimitExceeded {
                ticks: ticks_count,
                script_so_far: script,
                voxels_to_do,
            });
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

#[allow(dead_code)]
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
                Plan::HeadingFor { target, .. } if target == self.bot.pos => {
                    // reached a target
                    // println!(" ;; successfully reached {:?}", target);

                    // check if there is a job nearby
                    for neighbour_coord in target.get_neighbours_limit(current_model.dim() as isize) {
                        if let Some(cmd) = try_perform_job(&target, &neighbour_coord, env, current_model, &is_passable, rng) {
                            // println!("  == performing job : {:?}", cmd);
                            return PlanResult::Regular { nanobot: self, cmd, };
                        }
                    }

                    // locate and move to a job to perform
                    let mut nearest_voids = env.source_kd.nearest(&target);
                    let mut nearest_fills = env.target_kd.nearest(&target);
                    let mut maybe_void = nearest_voids.next();
                    let mut maybe_fill = nearest_fills.next();
                    let mut shuffle_coords = Vec::new();
                    loop {
                        let nearest_job = loop {
                            match (maybe_void.take(), maybe_fill.take()) {
                                (None, None) =>
                                    break None,
                                (Some((job, _)), None) => {
                                    maybe_void = nearest_voids.next();
                                    break Some(job);
                                },
                                (None, Some((job, _))) => {
                                    maybe_fill = nearest_fills.next();
                                    break Some(job);
                                },
                                (Some((job_void, dist_void)), Some((job_fill, dist_fill))) =>
                                    if dist_void < dist_fill {
                                        maybe_void = nearest_voids.next();
                                        break Some(job_void);
                                    } else {
                                        maybe_fill = nearest_fills.next();
                                        break Some(job_fill);
                                    },
                            }
                        };
                        if let Some(job) = nearest_job {
                            if try_perform_job(&target, &job, env, current_model, &is_passable, rng).is_none() {
                                continue;
                            }
                            shuffle_coords.extend(job.get_neighbours_limit(current_model.dim() as isize));
                            rng.shuffle(&mut shuffle_coords);
                            for possible_target in shuffle_coords.drain(..) {
                                let route_result =
                                    route_and_step(&target, &possible_target, current_model, &is_passable, commands_buf, env.config.rtt_limit, rng);
                                match route_result {
                                    Ok(Some(moving_cmd)) => {

                                        // println!(" ;; found new job at {:?} moving from {:?}, performing {:?}",
                                        //          possible_target, target, moving_cmd);

                                        self.plan = Plan::HeadingFor { target: possible_target, attempts: 0, };
                                        return PlanResult::Regular { nanobot: self, cmd: moving_cmd, };
                                    },
                                    Ok(None) =>
                                        (),
                                    Err(error) =>
                                        return PlanResult::Error(error),
                                }
                            }
                        } else {
                            break;
                        }
                    }
                    // no job could be found, go somewhere
                    let wandering_target = pick_random_coord(current_model.dim() as isize, rng);
                    // println!(" ;; no job could be found, wandering from {:?} to {:?}", target, wandering_target);
                    // println!("  == original is {}", if is_passable(&Region { min: target, max: target, }) { "passable" } else { "blocked" });
                    self.plan = Plan::HeadingFor {
                        target: wandering_target,
                        attempts: 0,
                    };
                },
                Plan::HeadingFor { target, attempts, } if attempts > env.config.route_attempts_limit =>
                    return PlanResult::Error(Error::RouteAttempsLimitExceeded {
                        source: self.bot.pos,
                        target,
                        attempts,
                    }),
                Plan::HeadingFor { target, attempts, } => {
                    // still moving to target
                    let route_result =
                        route_and_step(&self.bot.pos, &target, current_model, &is_passable, commands_buf, env.config.rtt_limit, rng);
                    match route_result {
                        Ok(Some(moving_cmd)) => {
                            // can continue moving
                            // println!("   -- can continue moving from {:?} to {:?} with {:?}", self.bot.pos, target, moving_cmd);

                            self.plan = Plan::HeadingFor { target, attempts: 0, };
                            return PlanResult::Regular { nanobot: self, cmd: moving_cmd, };
                        },
                        Ok(None) => {
                            // can not move there
                            // println!("   -- can not move from {:?} to {:?}", self.bot.pos, target);

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

fn try_perform_job<FP, R>(
    bot_coord: &Coord,
    job_coord: &Coord,
    env: &Env,
    current_model: &Matrix,
    is_passable: FP,
    rng: &mut R,
)
    -> Option<BotCommand> where
    FP: Fn(&Region) -> bool,
    R: Rng,
{
    let current_filled = current_model.is_filled(job_coord);
    let source_filled = env.source_model.is_filled(job_coord);
    let target_filled = env.target_model.is_filled(job_coord);
    if current_filled && source_filled && !target_filled {
        // "void" job
        let voxels: HashSet<_> = current_model
            .filled_voxels()
            .cloned()
            .filter(|c| c != job_coord)
            .collect();
        if coord::all_voxels_are_grounded(voxels) {
            return Some(BotCommand::Void { near: job_coord.diff(bot_coord), });
        }
    }
    if !current_filled && target_filled {
        // "fill" job
        if current_model.will_be_grounded(job_coord) {
            let maybe_route = rtt::plan_route_rng(
                bot_coord,
                &INIT_POS,
                current_model.dim(),
                |region| !region.contains(job_coord) && is_passable(region),
                env.config.rtt_limit,
                rng,
            );
            if maybe_route.is_some() {
                return Some(BotCommand::Fill { near: job_coord.diff(bot_coord), });
            }
        }
    }
    None
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

    #[test]
    fn solve_void_and_halt() {
        use rand::{SeedableRng, prng::XorShiftRng};
        let mut rng: XorShiftRng =
            SeedableRng::from_seed([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
        let source_model = Matrix::from_iter(Resolution(3), vec![Coord { x: 1, y: 0, z: 1, }]);
        let target_model = Matrix::from_iter(Resolution(3), vec![]);
        let script = super::solve_rng(
            source_model,
            target_model,
            super::Config {
                init_bots: vec![],
                rtt_limit: 64,
                route_attempts_limit: 16,
                global_ticks_limit: 100,
            },
            &mut rng,
        ).unwrap();
        assert_eq!(
            script,
            vec![
                BotCommand::SMove { long: LinearCoordDiff::Long { axis: Axis::Z, value: 2 } },
                BotCommand::Void { near: CoordDiff(Coord { x: 1, y: 0, z: -1 }) },
                BotCommand::SMove { long: LinearCoordDiff::Long { axis: Axis::Z, value: -2 } },
                BotCommand::Halt
            ],
        );
    }

    #[test]
    fn solve_void_tower_and_halt() {
        use rand::{SeedableRng, prng::XorShiftRng};
        let mut rng: XorShiftRng =
            SeedableRng::from_seed([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
        let source_model = Matrix::from_iter(Resolution(3), vec![
            Coord { x: 1, y: 0, z: 1, },
            Coord { x: 1, y: 1, z: 1, },
            Coord { x: 1, y: 2, z: 1, },
        ]);
        let target_model = Matrix::from_iter(Resolution(3), vec![]);
        let script = super::solve_rng(
            source_model,
            target_model,
            super::Config {
                init_bots: vec![],
                rtt_limit: 64,
                route_attempts_limit: 16,
                global_ticks_limit: 100,
            },
            &mut rng,
        ).unwrap();
        assert_eq!(
            script,
            vec![
                BotCommand::SMove { long: LinearCoordDiff::Long { axis: Axis::Z, value: 2 } },
                BotCommand::LMove {
                    short1: LinearCoordDiff::Short { axis: Axis::Y, value: 1 },
                    short2: LinearCoordDiff::Short { axis: Axis::Z, value: -2 },
                },
                BotCommand::SMove { long: LinearCoordDiff::Long { axis: Axis::X, value: 1 } },
                BotCommand::Void { near: CoordDiff(Coord { x: 0, y: 1, z: 1 }) },
                BotCommand::Void { near: CoordDiff(Coord { x: 0, y: 0, z: 1 }) },
                BotCommand::Void { near: CoordDiff(Coord { x: 0, y: -1, z: 1 }) },
                BotCommand::LMove {
                    short1: LinearCoordDiff::Short { axis: Axis::X, value: -1 },
                    short2: LinearCoordDiff::Short { axis: Axis::Y, value: -1 },
                },
                BotCommand::Halt,
            ],
        );
    }

    #[test]
    fn solve_fill_tower_and_halt() {
        use rand::{SeedableRng, prng::XorShiftRng};
        let mut rng: XorShiftRng =
            SeedableRng::from_seed([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
        let source_model = Matrix::from_iter(Resolution(3), vec![]);
        let target_model = Matrix::from_iter(Resolution(3), vec![
            Coord { x: 1, y: 0, z: 1, },
            Coord { x: 1, y: 1, z: 1, },
            Coord { x: 1, y: 2, z: 1, },
        ]);
        let script = super::solve_rng(
            source_model,
            target_model,
            super::Config {
                init_bots: vec![],
                rtt_limit: 64,
                route_attempts_limit: 16,
                global_ticks_limit: 100,
            },
            &mut rng,
        ).unwrap();
        assert_eq!(
            script,
            vec![
                BotCommand::SMove { long: LinearCoordDiff::Long { axis: Axis::Z, value: 2 } },
                BotCommand::Fill { near: CoordDiff(Coord { x: 1, y: 0, z: -1 }) },
                BotCommand::LMove {
                    short1: LinearCoordDiff::Short { axis: Axis::Z, value: -2 },
                    short2: LinearCoordDiff::Short { axis: Axis::Y, value: 1 },
                },
                BotCommand::Fill { near: CoordDiff(Coord { x: 1, y: 0, z: 1 }) },
                BotCommand::LMove {
                    short1: LinearCoordDiff::Short { axis: Axis::Y, value: 1 },
                    short2: LinearCoordDiff::Short { axis: Axis::X, value: 2 },
                },
                BotCommand::Fill { near: CoordDiff(Coord { x: -1, y: 0, z: 1 }) },
                BotCommand::LMove {
                    short1: LinearCoordDiff::Short { axis: Axis::Y, value: -2 },
                    short2: LinearCoordDiff::Short { axis: Axis::X, value: -2 },
                },
                BotCommand::Halt,
            ],
        );
    }

    #[test]
    fn solve_rebuild_tower_and_halt() {
        use rand::{SeedableRng, prng::XorShiftRng};
        let mut rng: XorShiftRng =
            SeedableRng::from_seed([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
        let source_model = Matrix::from_iter(Resolution(3), vec![
            Coord { x: 1, y: 0, z: 1, },
            Coord { x: 1, y: 1, z: 1, },
            Coord { x: 1, y: 2, z: 1, },
        ]);
        let target_model = Matrix::from_iter(Resolution(3), vec![
            Coord { x: 1, y: 0, z: 1, },
            Coord { x: 1, y: 1, z: 1, },
            Coord { x: 0, y: 1, z: 1, },
            Coord { x: 2, y: 1, z: 1, },
            Coord { x: 1, y: 1, z: 0, },
            Coord { x: 1, y: 1, z: 2, },
        ]);
        let script = super::solve_rng(
            source_model,
            target_model,
            super::Config {
                init_bots: vec![],
                rtt_limit: 64,
                route_attempts_limit: 16,
                global_ticks_limit: 100,
            },
            &mut rng,
        ).unwrap();
        assert_eq!(script.iter().filter(|cmd| if let BotCommand::Fill { .. } = cmd { true } else { false }).count(), 4);
        assert_eq!(script.iter().filter(|cmd| if let BotCommand::Void { .. } = cmd { true } else { false }).count(), 1);
        assert_eq!(script.last(), Some(&BotCommand::Halt));
    }

    // #[test]
    // fn solve_la008_tgt_mdl() {
    //     use rand::{SeedableRng, prng::XorShiftRng};
    //     use super::super::super::junk::LA008_TGT_MDL;
    //     let target_model = super::super::super::model::read_model(LA008_TGT_MDL).unwrap();
    //     let mut rng: XorShiftRng =
    //         SeedableRng::from_seed([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
    //     let source_model = Matrix::new(Resolution(target_model.dim() as isize));
    //     let _script = super::solve_rng(
    //         source_model,
    //         target_model,
    //         super::Config {
    //             init_bots: vec![],
    //             rtt_limit: 64,
    //             route_attempts_limit: 64,
    //             global_ticks_limit: 4096,
    //         },
    //         &mut rng,
    //     ).unwrap();

    // }
}
