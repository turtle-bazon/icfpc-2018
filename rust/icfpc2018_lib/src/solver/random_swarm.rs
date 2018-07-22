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

#[derive(Clone, PartialEq, Eq, Debug)]
pub enum Error {
    ModelsDimMismatch { source_dim: usize, target_dim: usize, },
    EmptyCommandsBufferForRoute { route: Vec<Coord>, },
}

pub struct Config {
    wandering_rtt_limit: usize,
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
    let mut commands_buf = Vec::new();

    let (init_bid, init_bot) = Nanobot::init_bot();
    let (wandering_cmd, wandering_plan) = pick_wandering_target(
        &init_bot.pos,
        &env,
        &current_model,
        None.into_iter(),
        &mut commands_buf,
        &mut rng,
    )?;
    let init_plan = Plan::Wandering(wandering_plan);
    let mut nanobots = vec![
        Nanobot {
            bid: init_bid,
            bot: init_bot,
            plan: init_plan,
            cmd: wandering_cmd,
        },
    ];

    // loop {
    //     let next_nanobots: Vec<Nanobot> =
    //         Vec::with_capacity(nanobots.len());
    //     for nanobot in nanobots.iter() {
    //         // bot.implement_plan(&env, &current_model, &config, &mut rng);
    //     }
    //     let nanobots = next_nanobots;

    //     unimplemented!()
    // }

    unimplemented!()
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
    cmd: BotCommand,
}

#[derive(Clone, PartialEq, Eq, Debug)]
enum Plan {
    Wandering(Wandering),
}

impl Nanobot {
    fn init_bot() -> (Bid, Bot) {
        (1, Bot {
            pos: Coord {
                x: 0,
                y: 0,
                z: 0,
            },
            seeds: (2 ..= 40).collect(),
        })
    }
}

fn pick_random_coord<R>(dim: isize, rng: &mut R) -> Coord where R: Rng {
    Coord {
        x: rng.gen_range(0, dim),
        y: rng.gen_range(0, dim),
        z: rng.gen_range(0, dim),
    }
}

#[derive(Clone, PartialEq, Eq, Debug)]
struct Wandering {
    target: Coord,
}

fn pick_wandering_target<R, VI>(
    bot_pos: &Coord,
    env: &Env,
    current_model: &Matrix,
    volatiles: VI,
    commands_buf: &mut Vec<(Coord, BotCommand)>,
    rng: &mut R,
)
    -> Result<(BotCommand, Wandering), Error> where
    R: Rng,
    VI: Iterator<Item = Region> + Clone,
{
    let dim = current_model.dim() as isize;
    loop {
        let target = pick_random_coord(dim, rng);
        let maybe_route = rtt::plan_route(
            bot_pos,
            &target,
            current_model,
            volatiles.clone(),
            env.config.wandering_rtt_limit,
        );
        if let Some(route) = maybe_route {
            rtt::plan_route_commands(&route, commands_buf);
            if commands_buf.is_empty() {
                return Err(Error::EmptyCommandsBufferForRoute { route, });
            }
            let (_move_coord, move_command) =
                commands_buf.swap_remove(0);
            return Ok((move_command, Wandering { target, }));
        }
    }
}

#[cfg(test)]
mod test {
    use super::super::super::{
        coord::Coord,
        state::Bot,
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

}
