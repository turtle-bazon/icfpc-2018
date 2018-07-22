extern crate icfpc2018_lib;
#[macro_use] extern crate clap;

use clap::Arg;
use std::fs::File;
use std::io::Read;

use icfpc2018_lib as kernel;

#[derive(Debug)]
enum Error {
    Args(clap::Error),
    Io(std::io::Error),
    ModelReadError(kernel::model::Error),
    Cmd(kernel::cmd::Error),
    State(kernel::state::Error),
}



fn main() -> Result<(),Error> {
    let app = app_from_crate!()
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

    let ref_model = match kernel::model::read_model_file(model_filename) {
        Err(e) => return Err(Error::ModelReadError(e)),
        Ok(m) => m,
    };

    let mut f = File::open(&trace_filename).map_err(Error::Io)?;
    let mut buffer = Vec::new();
    f.read_to_end(&mut buffer).map_err(Error::Io)?;

    let matrix = ref_model.new_empty_of_same_size();
    let mut state = kernel::state::State::new(matrix, vec![]);

    let cmds = kernel::cmd::from_bytes(&buffer).map_err(Error::Cmd)?;
    println!("Commands: {}", cmds.len());

    let res = state.run_mut(cmds);
    println!("Steps: {} ", state.steps);
    println!("ENERGY: {}", state.energy);
    match res {
        Err(e) => {
            println!("ERROR: {:?}", e);
            Err(Error::State(e))
        },
        Ok(_) => {
            println!("SUCCESS");
            Ok(())
        }
    }
}


#[cfg(test)]
mod test {
    use super::*;
}
