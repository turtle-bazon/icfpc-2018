use std;
use std::collections::VecDeque;

use icfpc2018_lib as kernel;

use kernel::coord::{
    Matrix,
};
use kernel::state::State;
use kernel::cmd::BotCommand;

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
    pub fn optimize(trace: Vec<BotCommand>, src_model: Matrix, dst_model: &Matrix) -> Vec<BotCommand> {
        let mut state = State::new(src_model, vec![]);
        
        loop {
            let res = state.step_mut(&mut cmd_iter);
            match res {
                Err(kernel::state::Error::StateNotWellformed{status}) => (),
                Err(e) => {
                    unimplemented!();
                    return Err(e);
                },
                Ok(_) => ()
            }

            if self.is_halt() {
                unimplemented!();
                return Ok(())
            }
        }
        unimplemented!()
    }
}

#[cfg(test)]
mod test {
}

