use std::collections::{HashSet, BTreeMap};

use super::{
    coord::{
        Coord,
        Matrix,
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


impl State {
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
}
