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
}

pub struct Config {
    rtt_limit: usize,
}

pub fn solve(source_model: Matrix, target_model: Matrix, config: Config) -> Result<Vec<BotCommand>, Error> {
    let source_dim = source_model.dim();
    let target_dim = target_model.dim();
    if source_dim != target_dim {
        return Err(Error::ModelsDimMismatch { source_dim, target_dim, });
    }
    let env = Env::new(source_model, target_model, config);
    let mut current_model = env.source_model.clone();
    let mut rng = rand::thread_rng();
    let mut commands_buf: Vec<(Coord, BotCommand)> = Vec::new();
    let mut script: Vec<BotCommand> = Vec::new();

    let (init_bid, init_bot) = Nanobot::init_bot();
    let mut nanobots = vec![
        Nanobot {
            bid: init_bid,
            bot: init_bot,
            plan: Plan::Init,
        },
    ];

    let mut work_complete = false;
    loop {
        // check for stop condition
        if work_complete || current_model.equals(&env.target_model) {
            work_complete = true;
            if nanobots.is_empty() {
                return Ok(script);
            }
        }




    //         // one nanobot left and it is ready for halt
    //         if nanobots.len() == 1 && nanobots[0].0.bot.pos == init_bot_pos {
    //             nanobots[0].0.plan = Plan::Perish;
    //             nanobots[0].1 = BotCommand::Halt;
    //         } else {
    //             // find master nanobot
    //             let master_index = nanobots.iter()
    //                 .map(|p| &p.0)
    //                 .enumerate()
    //                 .min_by_key(|(_, ref nanobot)| nanobot.bot.pos.diff(&init_bot_pos).l_1_norm())
    //                 .map(|(index, _)| index)
    //                 .unwrap_or(0);
    //             if nanobots[master_index].0.bot.pos == init_bot_pos {
    //                 // master is ready for fusion


    //             } else {
    //                 // master is on the way to start position
    //                 nanobots[master_index].0.plan = Plan::MasterReturnToBase {
    //                     base: init_bot_pos,
    //                 };

    //             }



    //             // find slave nanobot
    //             let maybe_slave_index = {
    //                 if nanobots[master_index].0.bot.pos == init_bot_pos {
    //                     nanobots[master_index].1 = BotCommand::Wait;
    //                     nanobots.iter()
    //                         .position(|p| init_bot_pos.diff(&p.0.bot.pos).is_near());
    //                 } else {


    //                     nanobots[master_index].1 = BotCommand::Wait;
    //                     None
    //                 }
    //             };
    //             // fusion if able to
    //             if let Some(slave_index) = maybe_slave_index {
    //                 nanobots[master_index].1 = BotCommand::FusionP {
    //                     near: init_bot_pos.diff(&nanobots[slave_index].0.bot.pos),
    //                 };
    //                 nanobots[slave_index].0.plan = Plan::Perish;
    //                 nanobots[slave_index].1 = BotCommand::FusionS {
    //                     near: nanobots[slave_index].0.bot.pos.diff(&init_bot_pos),
    //                 };
    //             }
    //         }
    //     }

    //     let mut next_nanobots =
    //         Vec::with_capacity(nanobots.len());
    //     for (mut nanobot, performing_cmd) in nanobots {
    //         // interpret command
    //         match performing_cmd {
    //             BotCommand::Halt |
    //             BotCommand::Wait |
    //             BotCommand::Flip |
    //             BotCommand::Fission { .. } |
    //             BotCommand::FusionP{ .. } |
    //             BotCommand::FusionS{ .. } |
    //             BotCommand::GFill { .. } |
    //             BotCommand::GVoid { .. } =>
    //                 (),
    //             BotCommand::SMove { ref long } => {
    //                 let move_diff = long.to_coord_diff();
    //                 let move_coord = nanobot.bot.pos.add(move_diff);
    //                 nanobot.bot.pos = move_coord;
    //             },
    //             BotCommand::LMove { ref short1, ref short2, } => {
    //                 let move_diff = short1.to_coord_diff();
    //                 let move_coord = nanobot.bot.pos.add(move_diff);
    //                 let move_diff = short2.to_coord_diff();
    //                 let move_coord = move_coord.add(move_diff);
    //                 nanobot.bot.pos = move_coord;
    //             },
    //             BotCommand::Fill { near, } => {
    //                 let fill_coord = nanobot.bot.pos.add(near);
    //                 current_model.set_filled(&fill_coord);
    //             },
    //             BotCommand::Void{ near, } => {
    //                 let void_coord = nanobot.bot.pos.add(near);
    //                 current_model.set_void(&void_coord);
    //             },
    //         }

    //         // record the script
    //         script.push(performing_cmd);

    //         // implement nanobot plan
    //         let next_nanobots_iter =
    //             nanobot.implement_plan(&env, &current_model, None.into_iter(), &mut commands_buf, &mut rng)?;
    //         next_nanobots.extend(next_nanobots_iter);
    //     }
    //     nanobots = next_nanobots;
    //     nanobots.sort_by_key(|&(ref bot, _)| bot.bid);
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

#[derive(Clone, PartialEq, Eq, Debug)]
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

    fn implement_plan<R, VI>(
        mut self,
        env: &Env,
        current_model: &Matrix,
        work_state: WorkState,
        volatiles: VI,
        commands_buf: &mut Vec<(Coord, BotCommand)>,
        rng: &mut R,
    )
        -> PlanResult where
        R: Rng,
        VI: Iterator<Item = Region> + Clone,
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
                            near: self.bot.pos.diff(&slave_coord),
                        };
                        PlanResult::Regular { nanobot: self, cmd: fusion_cmd, }
                    } else {
                        PlanResult::Regular { nanobot: self, cmd: BotCommand::Wait, }
                    };
                } else {
                    match slave_pick {
                        Some(coord) if coord == self.bot.pos =>
                            return PlanResult::DoAndPerish(BotCommand::FusionS {
                                near: self.bot.pos.diff(&INIT_POS),
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
                Plan::HeadingFor { target, attempts, } => {
                    // still moving to target
                    let route_result =
                        route_and_step(&self.bot.pos, &target, current_model, volatiles.clone(), commands_buf, env.config.rtt_limit);
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
                                    let pick_result = pick_wandering_target(
                                        &self.bot.pos,
                                        env,
                                        current_model,
                                        volatiles.clone(),
                                        commands_buf,
                                        rng,
                                    );
                                    match pick_result {
                                        Ok((wandering_cmd, wandering_coord)) => {
                                            self.plan = Plan::HeadingFor { target: wandering_coord, attempts: 0, };
                                        },
                                        Err(error) =>
                                            return PlanResult::Error(error),
                                    }
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

fn pick_wandering_target<R, VI>(
    bot_pos: &Coord,
    env: &Env,
    current_model: &Matrix,
    volatiles: VI,
    commands_buf: &mut Vec<(Coord, BotCommand)>,
    rng: &mut R,
)
    -> Result<(BotCommand, Coord), Error> where
    R: Rng,
    VI: Iterator<Item = Region> + Clone,
{
    let dim = current_model.dim() as isize;
    loop {
        let target = pick_random_coord(dim, rng);
        if let Some(move_command) =
            route_and_step(bot_pos, &target, current_model, volatiles.clone(), commands_buf, env.config.rtt_limit)?
        {
            return Ok((move_command, target));
        }
    }
}

fn route_and_step<VI>(
    start: &Coord,
    finish: &Coord,
    current_model: &Matrix,
    volatiles: VI,
    commands_buf: &mut Vec<(Coord, BotCommand)>,
    rtt_limit: usize,
)
    -> Result<Option<BotCommand>, Error> where
    VI: Iterator<Item = Region> + Clone,
{
    let maybe_route = rtt::plan_route(
        start,
        finish,
        current_model,
        volatiles.clone(),
        rtt_limit,
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
            Coord,
            Matrix,
            Resolution,
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
        // let script = super::solve(source_model, target_model, super::Config {
        //     wandering_rtt_limit: 16,
        // }).unwrap();
        // assert_eq!(script, vec![BotCommand::Halt]);
    }
}
