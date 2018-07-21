extern crate icfpc2018_lib;
#[macro_use] extern crate clap;

use clap::Arg;
use std::fs::File;
use std::io::{Read,Write};
use std::collections::VecDeque;

use icfpc2018_lib as kernel;
use kernel::cmd::BotCommand;
use kernel::coord::{LinearCoordDiff,Axis,M,Coord,CoordDiff};

#[derive(Debug)]
enum Error {
    Args(clap::Error),
    Io(std::io::Error),
    ModelReadError(kernel::model::Error),
    Cmd(kernel::cmd::Error),
    State(kernel::state::Error),
}



fn main() -> Result<(),Error> {
    let mut app = app_from_crate!()
        .arg(Arg::with_name("model")
             .display_order(1)
             .short("m")
             .long("model")
             .help("Model file (In)",)
             .default_value("../../problems/FA001_tgt.mdl")
             .takes_value(true))
        .arg(Arg::with_name("trace")
             .display_order(1)
             .short("t")
             .long("trace")
             .help("Original trace (In)",)
             .default_value("../../traces/FA001.nbt")
             .takes_value(true));

    let matches = app.get_matches();
    let model_filename = value_t!(matches, "model", String).map_err(Error::Args)?;
    let trace_filename = value_t!(matches, "trace", String).map_err(Error::Args)?;

    let ref_model = kernel::model::read_model_file(model_filename).unwrap();

    let mut f = File::open(&trace_filename).map_err(Error::Io)?;
    let mut buffer = Vec::new();
    f.read_to_end(&mut buffer).map_err(Error::Io)?;
    let mut cmds = kernel::cmd::from_bytes(&buffer).map_err(Error::Cmd)?;


    let mut matrix = kernel::coord::Matrix::new(kernel::coord::Resolution(ref_model.dim() as isize));
    let mut state = kernel::state::State::new(matrix, vec![]);
    let mut step_counter = 0;
    loop {
        step_counter += 1;
        let res = state.step_mut(&mut cmds);
        match res {
            Err(e) => {
                println!("ERROR: {:?}", e);
                return Err(Error::State(e));
            },
            Ok(e) => {}
        }

        if cmds.is_empty() {
            println!("ENERGY {} Steps {} ", state.energy, step_counter);
            return Ok(())
        }
    }

    // TODO: Check ref_model and model correspondance by-voxel

}


#[cfg(test)]
mod test {
    use super::*;
}
