use std::collections::HashSet;
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
        Harmonics,
    },
    router::rtt,
};

const INIT_POS: Coord = Coord { x: 0, y: 0, z: 0, };

#[derive(Clone, PartialEq, Eq, Debug)]
pub enum Error {
    ModelsDimMismatch { source_dim: usize, target_dim: usize, },
    EmptyCommandsBufferForRoute { route: Vec<Coord>, },
    RouteAttempsLimitExceeded { source: Coord, target: Coord, attempts: usize, },
    GlobalTicksLimitExceeded { ticks: usize, voxels_to_do: usize, },
    NoRouteToVoidDest { start: Coord, finish: Coord, region: Region, },
    NoRouteToFillDest { start: Coord, finish: Coord, region: Region, },
}

pub struct Config {
    pub init_bots: Vec<(Bid, Bot)>,
    pub rtt_limit: usize,
    pub rtt_wander_limit: usize,
    pub route_attempts_limit: usize,
    pub global_ticks_limit: usize,
    pub max_spawns: usize,
    pub max_seeds: usize,
}

pub fn solve(source_model: Matrix, target_model: Matrix, config: Config) -> Result<Vec<BotCommand>, (Error, Vec<BotCommand>)> {
    solve_rng(source_model, target_model, config, &mut rand::thread_rng())
}

pub fn solve_rng<R>(
    source_model: Matrix,
    target_model: Matrix,
    config: Config,
    rng: &mut R,
)
    -> Result<Vec<BotCommand>, (Error, Vec<BotCommand>)> where
    R: Rng
{
    let source_dim = source_model.dim();
    let target_dim = target_model.dim();
    if source_dim != target_dim {
        return Err((Error::ModelsDimMismatch { source_dim, target_dim, }, vec![]));
    }
    let env = Env::new(source_model, target_model, config);
    let mut current_model = env.source_model.clone();
    let mut commands_buf: Vec<(Coord, BotCommand)> = Vec::new();
    let mut script: Vec<BotCommand> = Vec::new();
    let mut script_tick: Vec<BotCommand> = Vec::new();
    let mut volatiles: HashSet<Coord> = HashSet::new();
    let mut pending_voids: Vec<Coord> = Vec::new();
    let mut pending_fills: Vec<Coord> = Vec::new();

    let mut void_towers = make_towers(&env.source_model);
    let mut fill_towers = make_towers(&env.target_model);

    let mut nanobots = if env.config.init_bots.is_empty() {
        let (init_bid, init_bot) = Nanobot::init_bot(env.config.max_seeds);
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

    use std::time::Instant;
    let mut now = Instant::now();

    let mut ticks_count = 0;
    let mut work_complete = false;
    let mut harmonics = Harmonics::Low;
    let mut ungrounded_voxel: Option<Coord> = None;
    loop {
        ticks_count += 1;

        let elapsed = now.elapsed();
        if elapsed.as_secs() >= 5 {
            debug!("ticks_count = {}", ticks_count);
            now = Instant::now();
        }

        if ticks_count >= env.config.global_ticks_limit {
            let mut voxels_to_do = 0;
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
            return Err((Error::GlobalTicksLimitExceeded {
                ticks: ticks_count,
                voxels_to_do,
            }, script));
        }
        // check for stop condition
        let work_state = if work_complete || current_model.equals(&env.target_model) {
            if !work_complete {
                info!("target model is successfully printed");
                work_complete = true;
            }
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
        volatiles.extend(nanobots.iter().map(|nanobot| nanobot.bot.pos));

        let prev_nanobots_count = nanobots.len();
        let mut nanobots_count = nanobots.len();
        let mut next_nanobots =
            Vec::with_capacity(nanobots_count);
        for nanobot in nanobots {
            let dim = current_model.dim() as isize;
            let nanobot_pos = nanobot.bot.pos;
            let implement_result =
                nanobot.implement_plan(
                    &env,
                    &current_model,
                    work_state,
                    ungrounded_voxel,
                    nanobots_count,
                    |region| if region.min.x < 0 || region.min.y < 0 || region.min.y < 0 {
                        false
                    } else if region.max.x >= dim || region.max.y >= dim || region.max.z >= dim {
                        false
                    } else if current_model.contains_filled(region) {
                        false
                    } else if region.contents().any(|p| p != nanobot_pos && volatiles.contains(&p)) {
                        false
                    } else {
                        true
                    },
                    &mut commands_buf,
                    &mut void_towers,
                    &mut fill_towers,
                    rng,
                );

            let mut interpret = |nanobot: &mut Nanobot, cmd: &BotCommand| Ok(match cmd {
                &BotCommand::Halt |
                &BotCommand::Wait |
                &BotCommand::FusionP{ .. } |
                &BotCommand::FusionS{ .. } |
                &BotCommand::GFill { .. } |
                &BotCommand::GVoid { .. } |
                &BotCommand::Flip =>
                    (),
                &BotCommand::Fission { .. } =>
                    nanobots_count += 1,
                &BotCommand::SMove { ref long } => {
                    let move_diff = long.to_coord_diff();
                    let move_coord = nanobot.bot.pos.add(move_diff);
                    volatiles.extend(Region::from_corners(
                        &move_coord,
                        &nanobot.bot.pos,
                    ).contents());
                    nanobot.bot.pos = move_coord;
                },
                &BotCommand::LMove { ref short1, ref short2, } => {
                    let move_diff_a = short1.to_coord_diff();
                    let move_coord_a = nanobot.bot.pos.add(move_diff_a);
                    volatiles.extend(Region::from_corners(
                        &move_coord_a,
                        &nanobot.bot.pos,
                    ).contents());
                    let move_diff_b = short2.to_coord_diff();
                    let move_coord_b = move_coord_a.add(move_diff_b);
                    volatiles.extend(Region::from_corners(
                        &move_coord_b,
                        &move_coord_a,
                    ).contents());
                    nanobot.bot.pos = move_coord_b;
                },
                &BotCommand::Fill { near, } => {
                    let fill_coord = nanobot.bot.pos.add(near);
                    pending_fills.push(fill_coord);
                    volatiles.insert(fill_coord);
                },
                &BotCommand::Void{ near, } => {
                    let void_coord = nanobot.bot.pos.add(near);
                    pending_voids.push(void_coord);
                    volatiles.insert(void_coord);
                },
            });

            match implement_result {
                PlanResult::DoAndPerish(cmd) =>
                    script_tick.push(cmd),
                PlanResult::Regular { mut nanobot, cmd, } => {
                    interpret(&mut nanobot, &cmd).map_err(|e| (e, script.clone()))?;
                    script_tick.push(cmd);
                    next_nanobots.push(nanobot);
                },
                PlanResult::Spawn { mut parent, child, cmd, } => {
                    interpret(&mut parent, &cmd).map_err(|e| (e, script.clone()))?;
                    script_tick.push(cmd);
                    next_nanobots.push(parent);
                    next_nanobots.push(child);
                },
                PlanResult::Error(error) =>
                    return Err((error, script)),
            }
        }

        nanobots = next_nanobots;
        nanobots.sort_by_key(|nanobot| nanobot.bid);

        for void_coord in pending_voids.drain(..) {
            current_model.set_void(&void_coord);
        }
        for fill_coord in pending_fills.drain(..) {
            current_model.set_filled(&fill_coord);
        }

        ungrounded_voxel = current_model.first_ungrounded_voxel();
        if ungrounded_voxel.is_some() {
            if let Harmonics::Low = harmonics {
                script.push(BotCommand::Flip);
                script.extend((1 .. prev_nanobots_count).map(|_| BotCommand::Wait));
                script.extend(script_tick.drain(..));
                harmonics = Harmonics::High;
            } else {
                script.extend(script_tick.drain(..));
            }
        } else {
            if let Harmonics::High = harmonics {
                script.extend(script_tick.drain(..));
                script.push(BotCommand::Flip);
                script.extend((1 .. nanobots_count).map(|_| BotCommand::Wait));
                harmonics = Harmonics::Low;
            } else {
                script.extend(script_tick.drain(..));
            }
        }
    }
}

struct Env {
    source_model: Matrix,
    target_model: Matrix,
    config: Config,
}

impl Env {
    fn new(source_model: Matrix, target_model: Matrix, config: Config) -> Env {
        Env {
            source_model,
            target_model,
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
    HeadingFor { target: Coord, attempts: usize, goal: Goal, },
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Goal {
    Park,
    Wander,
    Fill { tower: Region, },
    Void { tower: Region, },
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
    fn init_bot(max_seeds: usize) -> (Bid, Bot) {
        (1, Bot {
            pos: INIT_POS,
            seeds: (2 ..= max_seeds).collect(),
        })
    }

    fn implement_plan<FP, R>(
        mut self,
        env: &Env,
        current_model: &Matrix,
        work_state: WorkState,
        ungrounded_voxel: Option<Coord>,
        nanobots_count: usize,
        is_passable: FP,
        commands_buf: &mut Vec<(Coord, BotCommand)>,
        void_towers: &mut Vec<Region>,
        fill_towers: &mut Vec<Region>,
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
                            self.plan = Plan::HeadingFor {
                                target: if self.bid == 1 {
                                    INIT_POS
                                } else {
                                    let close: Vec<_> = INIT_POS.get_neighbours().collect();
                                    let index = rng.gen_range(0, close.len());
                                    close[index]
                                },
                                attempts: 0,
                                goal: Goal::Park,
                            },
                    };
                },
        }
        loop {
            match self.plan {
                Plan::Init => {
                    // go somewhere
                    let target = pick_random_coord(current_model.dim() as isize, rng);
                    self.plan = Plan::HeadingFor { target, attempts: 0, goal: Goal::Wander, };
                },
                Plan::HeadingFor { target, attempts, goal: Goal::Wander, .. } if attempts > env.config.route_attempts_limit => {
                    debug!("HeadingFor attempts limit exceeded for bid = {} while wandering, deciding to wait", self.bid);
                    debug!("active nanobots count = {}", nanobots_count);
                    debug!("current source {:?} is {}",
                           self.bot.pos, if is_passable(&Region { min: self.bot.pos, max: self.bot.pos, }) { "passable" } else { "blocked" });
                    debug!("current target {:?} is {}",
                           target, if is_passable(&Region { min: target, max: target, }) { "passable" } else { "blocked" });
                    // go somewhere else
                    let target = pick_random_coord(current_model.dim() as isize, rng);
                    self.plan = Plan::HeadingFor { target, attempts: 0, goal: Goal::Wander, };
                    return PlanResult::Regular { nanobot: self, cmd: BotCommand::Wait, };
                },
                Plan::HeadingFor { target, attempts, goal, } if attempts > env.config.route_attempts_limit => {
                    debug!("HeadingFor attempts limit exceeded for bid = {} and  goal = {:?}", self.bid, goal);
                    debug!("active nanobots count = {}", nanobots_count);
                    debug!("current source pos is {}",
                           if is_passable(&Region { min: self.bot.pos, max: self.bot.pos, }) { "passable" } else { "blocked" });
                    debug!("current target pos is {}",
                           if is_passable(&Region { min: target, max: target, }) { "passable" } else { "blocked" });
                    return PlanResult::Error(Error::RouteAttempsLimitExceeded {
                        source: self.bot.pos,
                        target,
                        attempts,
                    });
                },
                Plan::HeadingFor { goal: Goal::Park, target, .. } if target == self.bot.pos => {
                    warn!("target reached while in PARK mode, bot {} target {:?}", self.bid, self.bot.pos);
                    return PlanResult::Regular { nanobot: self, cmd: BotCommand::Wait, };
                },
                Plan::HeadingFor { goal: Goal::Wander, target, .. } if target == self.bot.pos => {
                    // spawn if able to
                    if self.bot.seeds.len() > 0 && nanobots_count < env.config.max_spawns && self.bid == 1 {
                        let dim = current_model.dim() as isize;
                        if let Some(pos) = target.get_neighbours()
                            .filter(|p| p.x < dim && p.y < dim && p.z < dim)
                            .find(|&p| is_passable(&Region { min: p, max: p, }))
                        {
                            let child = Nanobot {
                                bid: self.bot.seeds.remove(0),
                                bot: Bot { pos, seeds: vec![], },
                                plan: Plan::Init,
                            };
                            return PlanResult::Spawn {
                                parent: self,
                                child,
                                cmd: BotCommand::Fission { near: pos.diff(&target), split_m: 0, },
                            };
                        } else {
                            // go somewhere
                            let target = pick_random_coord(current_model.dim() as isize, rng);
                            self.plan = Plan::HeadingFor { target, attempts: 0, goal: Goal::Wander, };
                            continue;
                        }
                    }

                    // take a job if any
                    let dim = current_model.dim() as isize;
                    let maybe_void_index = void_towers.iter()
                        .enumerate()
                        .min_by_key(|(_, region)| {
                            let top_center = Coord { x: dim / 2, y: dim - 1, z: dim / 2, };
                            let flag = ungrounded_voxel.map(|v| !region.contains(&v)).unwrap_or(true);
                            (flag, -region.min.y, top_center.diff(&region.min).l_1_norm() + top_center.diff(&region.max).l_1_norm())
                        })
                        .map(|p| p.0);
                    let maybe_fill_index = fill_towers.iter()
                        .enumerate()
                        .min_by_key(|(_, region)| {
                            let bottom_center = Coord { x: dim / 2, y: 0, z: dim / 2, };
                            let flag = ungrounded_voxel.map(|v| !region.contains(&v)).unwrap_or(true);
                            (flag, region.min.y, bottom_center.diff(&region.min).l_1_norm() + bottom_center.diff(&region.max).l_1_norm())
                        })
                        .map(|p| p.0);
                    self.plan = if let Some(index) = maybe_void_index {
                        let void_region = void_towers.swap_remove(index);
                        Plan::HeadingFor {
                            goal: Goal::Void { tower: void_region, },
                            target: Coord {
                                x: void_region.max.x,
                                y: void_region.max.y + 1,
                                z: void_region.max.z,
                            },
                            attempts: 0,
                        }
                    } else if let Some(index) = maybe_fill_index {
                        let fill_region = fill_towers.swap_remove(index);
                        Plan::HeadingFor {
                            goal: Goal::Fill { tower: fill_region, },
                            target: Coord {
                                x: fill_region.min.x,
                                y: fill_region.min.y + 1,
                                z: fill_region.min.z,
                            },
                            attempts: 0,
                        }
                    } else {
                        // no jobs left, but model isn't fully printed yet: go to left side
                        let mut target = pick_random_coord(current_model.dim() as isize, rng);
                        target.x = 0;
                        Plan::HeadingFor { target, attempts: 0, goal: Goal::Wander, }
                    };
                },
                Plan::HeadingFor { goal: Goal::Void { mut tower, }, mut target, .. } if target == self.bot.pos => {
                    let job_coord = tower.max;
                    let current_filled = current_model.is_filled(&job_coord);
                    let source_filled = env.source_model.is_filled(&job_coord);
                    if current_filled && source_filled {
                        return PlanResult::Regular { nanobot: self, cmd: BotCommand::Void { near: job_coord.diff(&target), }, };
                    } else if tower.min == tower.max {
                        // get next tower
                        self.plan = Plan::HeadingFor { goal: Goal::Wander, target, attempts: 0, };
                    } else {
                        tower.max.y -= 1;
                        target.y -= 1;
                        self.plan = Plan::HeadingFor {
                            goal: Goal::Void { tower, },
                            target,
                            attempts: 0,
                        };
                    }
                },
                Plan::HeadingFor { goal: Goal::Fill { mut tower, }, mut target, .. } if target == self.bot.pos => {
                    let job_coord = tower.min;
                    let current_filled = current_model.is_filled(&job_coord);
                    let target_filled = env.target_model.is_filled(&job_coord);
                    if !current_filled && target_filled {
                        return if is_passable(&Region { min: job_coord, max: job_coord, }) {
                            PlanResult::Regular { nanobot: self, cmd: BotCommand::Fill { near: job_coord.diff(&target), }, }
                        } else {
                            PlanResult::Regular { nanobot: self, cmd: BotCommand::Wait, }
                        };
                    } else if tower.min == tower.max {
                        // get next tower
                        self.plan = Plan::HeadingFor { goal: Goal::Wander, target, attempts: 0, };
                    } else {
                        tower.min.y += 1;
                        target.y += 1;
                        self.plan = Plan::HeadingFor {
                            goal: Goal::Fill { tower, },
                            target,
                            attempts: 0,
                        };
                    }
                },
                Plan::HeadingFor { target, attempts, goal, } => {
                    // still moving to target
                    let rtt_limit = if let Goal::Wander = goal {
                        env.config.rtt_wander_limit
                    } else {
                        env.config.rtt_limit
                    };
                    let route_result =
                        route_and_step(&self.bot.pos, &target, current_model, &is_passable, commands_buf, rtt_limit, rng);
                    match route_result {
                        Ok(Some(moving_cmd)) => {
                            // can continue moving
                            self.plan = Plan::HeadingFor { target, goal, attempts: 0, };
                            return PlanResult::Regular { nanobot: self, cmd: moving_cmd, };
                        },
                        Ok(None) =>
                            // can not move there
                            match goal {
                                Goal::Wander => {
                                    // pick another wandering target
                                    let target = pick_random_coord(current_model.dim() as isize, rng);
                                    self.plan = Plan::HeadingFor { target, attempts: attempts + 1, goal: Goal::Wander, };
                                },
                                Goal::Park => {
                                    // try to find a free position nearby
                                    self.plan = Plan::HeadingFor {
                                        goal: Goal::Park,
                                        target: if self.bid == 1 {
                                            INIT_POS
                                        } else {
                                            let close: Vec<_> = INIT_POS.get_neighbours().collect();
                                            let index = rng.gen_range(0, close.len());
                                            close[index]
                                        },
                                        attempts: attempts + 1,
                                    };
                                    return PlanResult::Regular { nanobot: self, cmd: BotCommand::Wait, }

                                    // let next_attempts = attempts + 1;
                                    // let offset = (next_attempts % current_model.dim()) as isize;
                                    // let axis = next_attempts % 2;
                                    // self.plan = Plan::HeadingFor {
                                    //     goal: Goal::Park,
                                    //     target: Coord {
                                    //         x: if axis == 0 { offset } else { 0 },
                                    //         y: offset,
                                    //         z: if axis == 1 { offset } else { 0 },
                                    //     },
                                    //     attempts: next_attempts,
                                    // };
                                },
                                Goal::Void { tower, } => {
                                    self.plan = Plan::HeadingFor { target, attempts: attempts + 1, goal: Goal::Void { tower, } };
                                    return PlanResult::Regular { nanobot: self, cmd: BotCommand::Wait, };
                                },
                                Goal::Fill { tower, } => {
                                    self.plan = Plan::HeadingFor { target, attempts: attempts + 1, goal: Goal::Fill { tower, } };
                                    return PlanResult::Regular { nanobot: self, cmd: BotCommand::Wait, };
                                },
                            }
                        Err(error) =>
                            return PlanResult::Error(error),
                    }
                },
            }
        }
    }
}

fn make_towers(model: &Matrix) -> Vec<Region> {
    let dim = model.dim() as isize;
    let mut regions = Vec::new();
    for x in 0 .. dim {
        for z in 0 .. dim {
            let mut current_reg: Option<Region> = None;
            for y in 0 .. dim {
                let p = Coord { x, y, z, };
                if model.is_filled(&p) {
                    if let Some(ref mut reg) = current_reg {
                        reg.max = p;
                    } else {
                        current_reg = Some(Region { min: p, max: p, });
                    }
                } else if let Some(reg) = current_reg {
                    regions.push(reg);
                    current_reg = None;
                }
            }
            if let Some(reg) = current_reg {
                regions.push(reg);
            }
        }
    }
    regions
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
            Region,
            CoordDiff,
            Resolution,
            LinearCoordDiff,
        },
        state::Bot,
        cmd::BotCommand,
    };

    #[test]
    fn make_towers_1() {
        let model = Matrix::from_iter(Resolution(3), vec![
            Coord { x: 1, y: 0, z: 1, },
            Coord { x: 1, y: 1, z: 1, },
            Coord { x: 1, y: 2, z: 1, },
        ]);
        assert_eq!(super::make_towers(&model), vec![Region {
            min: Coord { x: 1, y: 0, z: 1, },
            max: Coord { x: 1, y: 2, z: 1, },
        }]);
    }

    #[test]
    fn make_towers_4() {
        let model = Matrix::from_iter(Resolution(3), vec![
            Coord { x: 1, y: 0, z: 1, },
            Coord { x: 1, y: 1, z: 1, },
            Coord { x: 1, y: 2, z: 1, },
            Coord { x: 0, y: 0, z: 0, },
            Coord { x: 0, y: 1, z: 0, },
            Coord { x: 2, y: 2, z: 2, },
            Coord { x: 2, y: 0, z: 2, },
        ]);
        assert_eq!(super::make_towers(&model), vec![
            Region {
                min: Coord { x: 0, y: 0, z: 0, },
                max: Coord { x: 0, y: 1, z: 0, },
            },
            Region {
                min: Coord { x: 1, y: 0, z: 1, },
                max: Coord { x: 1, y: 2, z: 1, },
            },
            Region {
                min: Coord { x: 2, y: 0, z: 2, },
                max: Coord { x: 2, y: 0, z: 2, },
            },
            Region {
                min: Coord { x: 2, y: 2, z: 2, },
                max: Coord { x: 2, y: 2, z: 2, },
            },
        ]);
    }

    #[test]
    fn solve_empty() {
        let source_model = Matrix::from_iter(Resolution(3), vec![]);
        let target_model = Matrix::from_iter(Resolution(3), vec![]);
        let script = super::solve(source_model, target_model, super::Config {
            init_bots: vec![],
            rtt_limit: 64,
            rtt_wander_limit: 64,
            route_attempts_limit: 16,
            global_ticks_limit: 100,
            max_spawns: 1,
            max_seeds: 1,
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
                rtt_wander_limit: 64,
                route_attempts_limit: 16,
                global_ticks_limit: 100,
                max_spawns: 1,
                max_seeds: 1,
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
                rtt_wander_limit: 64,
                route_attempts_limit: 16,
                global_ticks_limit: 100,
                max_spawns: 1,
                max_seeds: 1,
            },
            &mut rng,
        ).unwrap();
        assert_eq!(
            script,
            vec![
                BotCommand::LMove {
                    short1: LinearCoordDiff::Short { axis: Axis::Y, value: -1 },
                    short2: LinearCoordDiff::Short { axis: Axis::Z, value: -1 },
                },
                BotCommand::LMove {
                    short1: LinearCoordDiff::Short { axis: Axis::X, value: -2 },
                    short2: LinearCoordDiff::Short { axis: Axis::Z, value: -2 },
                },

                BotCommand::Wait,
                BotCommand::LMove {
                    short1: LinearCoordDiff::Short { axis: Axis::X, value: 1 },
                    short2: LinearCoordDiff::Short { axis: Axis::Y, value: -2 },
                },

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
                rtt_wander_limit: 64,
                route_attempts_limit: 16,
                global_ticks_limit: 100,
                max_spawns: 1,
                max_seeds: 1,
            },
            &mut rng,
        ).unwrap();
        assert_eq!(
            script,
            vec![
                BotCommand::LMove {
                    short1: LinearCoordDiff::Short { axis: Axis::Y, value: -1 },
                    short2: LinearCoordDiff::Short { axis: Axis::Z, value: -1 },
                },
                BotCommand::LMove {
                    short1: LinearCoordDiff::Short { axis: Axis::Y, value: -1 },
                    short2: LinearCoordDiff::Short { axis: Axis::Z, value: -2 },
                },
                BotCommand::LMove {
                    short1: LinearCoordDiff::Short { axis: Axis::X, value: 1 },
                    short2: LinearCoordDiff::Short { axis: Axis::Y, value: -2 },
                },

                BotCommand::FusionP { near: CoordDiff(Coord { x: 1, y: 0, z: 0 }) },
                BotCommand::Wait,
                BotCommand::FusionS { near: CoordDiff(Coord { x: -1, y: 0, z: 0 }) },

                BotCommand::Wait,
                BotCommand::LMove {
                    short1: LinearCoordDiff::Short { axis: Axis::Y, value: -1 },
                    short2: LinearCoordDiff::Short { axis: Axis::Z, value: 1, },
                },

                BotCommand::Wait,
                BotCommand::LMove {
                    short1: LinearCoordDiff::Short { axis: Axis::Z, value: -1 },
                    short2: LinearCoordDiff::Short { axis: Axis::Y, value: 1 },
                },

                BotCommand::Wait,
                BotCommand::LMove {
                    short1: LinearCoordDiff::Short { axis: Axis::X, value: -2 },
                    short2: LinearCoordDiff::Short { axis: Axis::Z, value: 1 }, },

                BotCommand::FusionP { near: CoordDiff(Coord { x: 0, y: 1, z: 1 }) },
                BotCommand::FusionS { near: CoordDiff(Coord { x: 0, y: -1, z: -1 }) },

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
                rtt_wander_limit: 64,
                route_attempts_limit: 16,
                global_ticks_limit: 100,
                max_spawns: 1,
                max_seeds: 1,
            },
            &mut rng,
        ).unwrap();
        assert_eq!(
            script,
            vec![
                BotCommand::SMove { long: LinearCoordDiff::Long { axis: Axis::Z, value: 2 } },
                BotCommand::LMove {
                    short1: LinearCoordDiff::Short { axis: Axis::Y, value: 1 },
                    short2: LinearCoordDiff::Short { axis: Axis::Z, value: -1 },
                },
                BotCommand::LMove {
                    short1: LinearCoordDiff::Short { axis: Axis::Y, value: -1 },
                    short2: LinearCoordDiff::Short { axis: Axis::Z, value: -1 },
                },
                BotCommand::LMove {
                    short1: LinearCoordDiff::Short { axis: Axis::Z, value: 1 },
                    short2: LinearCoordDiff::Short { axis: Axis::Y, value: 1 },
                },
                BotCommand::SMove { long: LinearCoordDiff::Long { axis: Axis::X, value: 1 } },
                BotCommand::Void { near: CoordDiff(Coord { x: 0, y: -1, z: 0 }) },
                BotCommand::LMove {
                    short1: LinearCoordDiff::Short { axis: Axis::Y, value: -1 },
                    short2: LinearCoordDiff::Short { axis: Axis::X, value: -1 },
                },
                BotCommand::SMove { long: LinearCoordDiff::Long { axis: Axis::Z, value: -1 } },
                BotCommand::Halt
            ],
        );
    }

    #[test]
    fn solve_void_tower_and_halt() {
        use rand::{SeedableRng, prng::XorShiftRng};
        let mut rng: XorShiftRng =
            SeedableRng::from_seed([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
        let source_model = Matrix::from_iter(Resolution(4), vec![
            Coord { x: 1, y: 0, z: 1, },
            Coord { x: 1, y: 1, z: 1, },
            Coord { x: 1, y: 2, z: 1, },
        ]);
        let target_model = Matrix::from_iter(Resolution(4), vec![]);
        let script = super::solve_rng(
            source_model,
            target_model,
            super::Config {
                init_bots: vec![],
                rtt_limit: 64,
                rtt_wander_limit: 64,
                route_attempts_limit: 16,
                global_ticks_limit: 100,
                max_spawns: 1,
                max_seeds: 1,
            },
            &mut rng,
        ).unwrap();
        assert_eq!(
            script,
            vec![
                BotCommand::LMove {
                    short1: LinearCoordDiff::Short { axis: Axis::Y, value: 3 },
                    short2: LinearCoordDiff::Short { axis: Axis::Z, value: 1 },
                },
                BotCommand::SMove { long: LinearCoordDiff::Long { axis: Axis::X, value: 1 } },
                BotCommand::Void { near: CoordDiff(Coord { x: 0, y: -1, z: 0 }) },
                BotCommand::SMove { long: LinearCoordDiff::Long { axis: Axis::Y, value: -1 } },
                BotCommand::Void { near: CoordDiff(Coord { x: 0, y: -1, z: 0 }) },
                BotCommand::LMove {
                    short1: LinearCoordDiff::Short { axis: Axis::Z, value: -1 },
                    short2: LinearCoordDiff::Short { axis: Axis::Y, value: -1 },
                },
                BotCommand::SMove { long: LinearCoordDiff::Long { axis: Axis::Z, value: 1 } },
                BotCommand::Void { near: CoordDiff(Coord { x: 0, y: -1, z: 0 }) },
                BotCommand::LMove {
                    short1: LinearCoordDiff::Short { axis: Axis::X, value: -1 },
                    short2: LinearCoordDiff::Short { axis: Axis::Y, value: -1 },
                },
                BotCommand::SMove { long: LinearCoordDiff::Long { axis: Axis::Z, value: -1 } },
                BotCommand::Halt,
            ],
        );
    }

    #[test]
    fn solve_fill_tower_and_halt() {
        use rand::{SeedableRng, prng::XorShiftRng};
        let mut rng: XorShiftRng =
            SeedableRng::from_seed([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
        let source_model = Matrix::from_iter(Resolution(4), vec![]);
        let target_model = Matrix::from_iter(Resolution(4), vec![
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
                rtt_wander_limit: 64,
                route_attempts_limit: 16,
                global_ticks_limit: 100,
                max_spawns: 1,
                max_seeds: 1,
            },
            &mut rng,
        ).unwrap();
        assert_eq!(
            script,
            vec![
                BotCommand::LMove {
                    short1: LinearCoordDiff::Short { axis: Axis::Y, value: 1 },
                    short2: LinearCoordDiff::Short { axis: Axis::Z, value: 1 },
                },
                BotCommand::LMove {
                    short1: LinearCoordDiff::Short { axis: Axis::Z, value: -1 },
                    short2: LinearCoordDiff::Short { axis: Axis::X, value: 1 },
                },
                BotCommand::SMove { long: LinearCoordDiff::Long { axis: Axis::Z, value: 1 } },
                BotCommand::Fill { near: CoordDiff(Coord { x: 0, y: -1, z: 0 }) },
                BotCommand::SMove { long: LinearCoordDiff::Long { axis: Axis::Y, value: 1 } },
                BotCommand::Fill { near: CoordDiff(Coord { x: 0, y: -1, z: 0 }) },
                BotCommand::SMove { long: LinearCoordDiff::Long { axis: Axis::Y, value: 1 } },
                BotCommand::Fill { near: CoordDiff(Coord { x: 0, y: -1, z: 0 }) },
                BotCommand::LMove {
                    short1: LinearCoordDiff::Short { axis: Axis::Z, value: -1 },
                    short2: LinearCoordDiff::Short { axis: Axis::X, value: -1 },
                },
                BotCommand::SMove { long: LinearCoordDiff::Long { axis: Axis::Y, value: -3 } },
                BotCommand::Halt,
            ],
        );
    }

    #[test]
    fn solve_rebuild_tower_and_halt() {
        use rand::{SeedableRng, prng::XorShiftRng};
        let mut rng: XorShiftRng =
            SeedableRng::from_seed([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
        let source_model = Matrix::from_iter(Resolution(4), vec![
            Coord { x: 1, y: 0, z: 1, },
            Coord { x: 1, y: 1, z: 1, },
            Coord { x: 1, y: 2, z: 1, },
        ]);
        let target_model = Matrix::from_iter(Resolution(4), vec![
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
                rtt_wander_limit: 64,
                route_attempts_limit: 16,
                global_ticks_limit: 100,
                max_spawns: 1,
                max_seeds: 1,
            },
            &mut rng,
        ).unwrap();
        assert_eq!(script.iter().filter(|cmd| if let BotCommand::Fill { .. } = cmd { true } else { false }).count(), 6);
        assert_eq!(script.iter().filter(|cmd| if let BotCommand::Void { .. } = cmd { true } else { false }).count(), 3);
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
    //     let script = super::solve_rng(
    //         source_model,
    //         target_model,
    //         super::Config {
    //             init_bots: vec![],
    //             rtt_limit: 256,
    //             route_attempts_limit: 512,
    //             global_ticks_limit: 4096,
    //             max_spawns: 1,
    //         },
    //         &mut rng,
    //     ).unwrap();
    //     assert_eq!(script.last(), Some(&BotCommand::Halt));
    // }
}
