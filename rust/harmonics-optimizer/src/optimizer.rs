use std;
use std::collections::VecDeque;

use icfpc2018_lib as kernel;

use kernel::{
    coord::{
        Matrix,
    },
    cmd::{
        BotCommand,
    },
    state::{
        State,
        Harmonics,
        WellformedStatus,
    },
};

#[derive(Debug)]
pub enum Error {
    State(kernel::state::Error),
}

#[derive(Debug)]
pub struct HarmonicsOptimizer {

    last_step_commands: Vec<BotCommand>,
}

impl Iterator for HarmonicsOptimizer {
    type Item = BotCommand;

    fn next(&mut self) -> Option<BotCommand> {
        None
    }
}

impl HarmonicsOptimizer {
    pub fn new() -> Self {
        HarmonicsOptimizer {
            last_step_commands: vec![],
        }
    }

    // pub fn next_step(&mut self) {
    // }

    pub fn optimize(trace: Vec<BotCommand>, src_model: Matrix, dst_model: &Matrix) -> Result<Vec<BotCommand>, Error> {
        let mut state = State::new(src_model, vec![]);
        let mut optimizer = HarmonicsOptimizer::new();

        loop {
            match state.harmonics {
                Harmonics::Low => {},
                Harmonics::High => {
                    if state.matrix.all_voxels_are_grounded() {
                        /* Switch to Low */
                    }
                },

            }
            optimizer.last_step_commands = Vec::with_capacity(state.bots.len());

            let res = state.step_mut(&mut optimizer);
            if let Err(kernel::state::Error::StateNotWellformed{status}) = res {
                if status == WellformedStatus::NotGroundedWhileLowHarmonics {

                }
            }

            // match res {
            //     Err(kernel::state::Error::StateNotWellformed{status}) => {

            //     },
            //     Err(e) => {
            //         return Err(Error::State(e));
            //     },
            //     Ok(_) => ()
            // }

            if state.is_halt() {
                unimplemented!();
                // return Ok(())
            }
        }
        unimplemented!()
    }
}

#[cfg(test)]
mod test {
}
