use std::collections::{HashSet, BTreeMap};

use super::{
    coord::{
        Coord,
        Matrix,
        Resolution,
        // LinearCoordDiff,
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

    /* Maybe this method should return either Error or Coord[] of volatile coordinates ? */
    pub fn do_cmd_mut(&mut self, cmd: BotCommand, bot: &mut Bot) -> Result<(), Error> {
        let c = bot.pos;

        match cmd {
            BotCommand::Halt => {
                let check_coord = c.x == 0 || c.y == 0 || c.z == 0;
                let bot_ids: Vec<Bid> = self.bots.keys().cloned().collect();
                let check_the_only_bot = bot_ids == [1];
                let check_low = self.harmonics == Harmonics::Low;
                match (check_coord, check_the_only_bot, check_low) {
                    (true, true, true) => {
                        self.bots.remove(&1);
                        Ok(()) // [c]
                    },
                    (false, _, _) => Err(Error::HaltNotAtZeroCoord),
                    (_, false, _) => Err(Error::HaltTooManyBots),
                    (_, _, false) => Err(Error::HaltNotInLow),
                }
            },
            BotCommand::Wait => Ok(()),
            BotCommand::Flip => {
                match self.harmonics {
                    Harmonics::Low => {
                        self.harmonics = Harmonics::High
                    },
                    Harmonics::High => {
                        self.harmonics = Harmonics::Low
                    },
                };
                Ok(()) //[c]
            },
            BotCommand::SMove{ long } => unimplemented!(),
            BotCommand::LMove{ short1, short2 } => unimplemented!(),
            BotCommand::Fission{ near, split_m } => unimplemented!(),
            BotCommand::Fill{ near } => {
                let cf = c.add(near);
                if (!self.matrix.is_filled(&cf)) {
                    self.matrix.set_filled(&cf);
                    self.energy += 12;
                }
                else {
                    self.energy += 6;
                }
                Ok(()) // [c,cf]
            },
            BotCommand::FusionP{ near } => unimplemented!(),
            BotCommand::FusionS{ near } => unimplemented!(),

        }
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
