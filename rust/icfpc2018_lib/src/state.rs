use std::collections::{HashSet, BTreeMap};

use super::{
    coord::{
        Coord,
        Matrix,
        Resolution,
        Region,
    },
    cmd::{
        BotCommand,
    },
};

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub enum Harmonics {
    Low,
    High,
}

pub type Bid = usize;

#[derive(Debug)]
pub struct Bot {
    pub pos: Coord,
    pub seeds: Vec<Bid>,
}

#[derive(Debug)]
pub struct Command;

#[derive(Debug)]
pub struct State {
    pub energy: usize,
    pub harmonics: Harmonics,
    pub matrix: Matrix,
    pub bots: BTreeMap<Bid, Bot>,
    pub trace: Vec<BotCommand>,
}

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub enum WellformedStatus {
    Wellformed,
    NotGroundedWhileLowHarmonics,
    BotInFilledVoxel,
    SeedsAreNotDisjoint,
    SeedIsTheSameAsActiveBot,
}

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub enum Error {
    StateNotWellformed{status: WellformedStatus},
    CommandsInterfere,
    HaltNotAtZeroCoord,
    HaltTooManyBots,
    HaltNotInLow,
    MoveOutOfBounds {c: Coord},
}


impl State {
    pub fn new(matrix: Matrix, trace: Vec<BotCommand>) -> State {
        let mut bots = BTreeMap::new();

        bots.insert(1, Bot {
            pos: Coord { x:0, y:0, z:0 },
            seeds: (2..21).collect(),
        });

        State {
            energy: 0,
            harmonics: Harmonics::Low,
            matrix,
            bots,
            trace,
        }
    }

    pub fn wellformed(&self) -> WellformedStatus {
        if let Harmonics::Low = self.harmonics {
            if !self.matrix.all_voxels_are_grounded() {
                return WellformedStatus::NotGroundedWhileLowHarmonics;
            }
        }
        if !self.bots.values().map(|bot| &bot.pos).all(|pos| !self.matrix.is_filled(pos)) {
            return WellformedStatus::BotInFilledVoxel;
        }

        let mut bids_seen = HashSet::new();
        for &bid in self.bots.values().flat_map(|bot| bot.seeds.iter()) {
            if !bids_seen.insert(bid) {
                return WellformedStatus::SeedsAreNotDisjoint;
            }
            if self.bots.contains_key(&bid) {
                return WellformedStatus::SeedIsTheSameAsActiveBot;
            }
        }

        WellformedStatus::Wellformed
    }

    pub fn do_cmd_mut(&mut self, cmd: BotCommand, bot: &mut Bot) -> Result<HashSet<Coord>, Error> {
        let c = bot.pos;
        let mut volatile: HashSet<Coord> = [c].iter().cloned().collect();

        match cmd {
            BotCommand::Halt => {
                let check_coord = c.x == 0 || c.y == 0 || c.z == 0;
                let bot_ids: Vec<Bid> = self.bots.keys().cloned().collect();
                let check_the_only_bot = bot_ids == [1];
                let check_low = self.harmonics == Harmonics::Low;

                match (check_coord, check_the_only_bot, check_low) {
                    (true, true, true) => {
                        self.bots.remove(&1);
                    },
                    (false, _, _) => return Err(Error::HaltNotAtZeroCoord),
                    (_, false, _) => return Err(Error::HaltTooManyBots),
                    (_, _, false) => return Err(Error::HaltNotInLow),
                }
            },
            BotCommand::Wait => (),
            BotCommand::Flip => {
                match self.harmonics {
                    Harmonics::Low => {
                        self.harmonics = Harmonics::High
                    },
                    Harmonics::High => {
                        self.harmonics = Harmonics::Low
                    },
                };
            },
            BotCommand::SMove{ long } => {
                let d = long.to_coord_diff();
                let cf = c.add(d);

                if !self.matrix.is_valid_coord(&cf) {
                    return Err(Error::MoveOutOfBounds{c: cf})
                }

                bot.pos = cf;
                self.energy += 2 * d.l_1_norm();

                for c in Region::from_corners(&c, &cf).coord_set().iter() {
                    volatile.insert(*c);
                }
            },
            BotCommand::LMove{ short1, short2 } => {
                let d1 = short1.to_coord_diff();
                let d2 = short2.to_coord_diff();

                let cf = c.add(d1);
                if !self.matrix.is_valid_coord(&cf) {
                    return Err(Error::MoveOutOfBounds{c: cf})
                }

                let cff = cf.add(d2);
                if !self.matrix.is_valid_coord(&cff) {
                    return Err(Error::MoveOutOfBounds{c: cff})
                }

                bot.pos = cff;
                self.energy += 2 * (d1.l_1_norm() + 2 + d2.l_1_norm());

                for c in Region::from_corners(&c, &cf).coord_set().iter() {
                    volatile.insert(*c);
                }
                for c in Region::from_corners(&cf, &cff).coord_set().iter() {
                    volatile.insert(*c);
                }
            },
            BotCommand::Fill{ near } => {
                let cf = c.add(near);
                if !self.matrix.is_filled(&cf) {
                    self.matrix.set_filled(&cf);
                    self.energy += 12;
                }
                else {
                    self.energy += 6;
                }
            },
            BotCommand::Fission{ near, split_m } => unimplemented!(),
            BotCommand::FusionP{ near } => unimplemented!(),
            BotCommand::FusionS{ near } => unimplemented!(),

        }
        Ok(volatile)
    }

    pub fn step_mut(&mut self) {
        // checks here
        //...
        let bot_count = self.bots.len();

        // take command sequence for this step (and drop them from the trace)
        let this_step_cmds: Vec<BotCommand> = self.trace.drain(0..bot_count).collect();

        // energy step for the step itself
        match self.harmonics {
            Harmonics::Low =>
                self.energy += 3 * self.matrix.dim() * self.matrix.dim() * self.matrix.dim(),
            Harmonics::High =>
                self.energy += 30 * self.matrix.dim() * self.matrix.dim() * self.matrix.dim(),
        }

        // energy for each nanobot
        self.energy += 20 * bot_count;

        // here run commands
        // ...
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use super::super::{
        coord::{
            Coord,
            Matrix,
            Resolution,
        }
    };

    #[test]
    fn test_init_state() {
        let matrix = Matrix::new(Resolution(10));
        let trace = vec![];
        let st = State::new(matrix, trace.to_vec());

        assert_eq!(st.energy, 0);
        assert_eq!(st.harmonics, Harmonics::Low);
        // assert_eq!(st.matrix, matrix);
        assert_eq!(st.trace, trace);

        // check the first (and the only) bot
        assert_eq!(st.bots.len(), 1);
        let bot = st.bots.get(&1).unwrap();
        assert_eq!(bot.pos, Coord { x:0, y:0, z:0 });
        assert_eq!(bot.seeds, vec![2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20]);
    }
}
