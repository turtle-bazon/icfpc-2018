use rand::{self};

use super::super::{
    coord::{
        Coord,
        Matrix,
    },
    cmd::BotCommand,
    state::{
        Bid,
        Bot,
    },
};

#[derive(Clone, PartialEq, Eq, Debug)]
pub enum Error {

}

pub fn solve(_source_model: Matrix, _target_model: Matrix) -> Result<Vec<BotCommand>, Error> {
    let mut _rng = rand::thread_rng();
    let mut _nanobots = Nanobot::init();


    unimplemented!()
}

#[derive(Clone, PartialEq, Eq, Debug)]
struct Nanobot {
    bid: Bid,
    bot: Bot,
}

impl Nanobot {
    fn init() -> Vec<Nanobot> {
        vec![
            Nanobot {
                bid: 1,
                bot: Bot {
                    pos: Coord {
                        x: 0,
                        y: 0,
                        z: 0,
                    },
                    seeds: (2 ..= 40).collect(),
                },
            },
        ]
    }
}

#[cfg(test)]
mod test {
    use super::super::super::{
        coord::Coord,
        state::Bot,
    };
    use super::Nanobot;

    #[test]
    fn nanobot_init() {
        assert_eq!(
            Nanobot::init(),
            vec![
                Nanobot {
                    bid: 1,
                    bot: Bot {
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
                    },
                }
            ],
        );
    }

}
