extern crate icfpc2018_lib;
#[macro_use] extern crate clap;

use clap::Arg;
use std::fs::File;
use std::io::Read;

use icfpc2018_lib as kernel;

use kernel::{
    cmd,
    model,
    state,
};

#[derive(Debug)]
enum Error {
    Args(clap::Error),
    NoSourceOrTargetModelProvided,
    Io(std::io::Error),
    Model(kernel::model::Error),
    Cmd(kernel::cmd::Error),
    State(kernel::state::Error),
    ModelNotMatch,
}



fn main() -> Result<(),Error> {
    let app = app_from_crate!()
        .arg(Arg::with_name("source-model")
             .short("s")
             .long("source-model")
             .value_name("FILE")
             .help("Source model")
             .takes_value(true))
        .arg(Arg::with_name("dst-model")
             .short("d")
             .long("dst-model")
             .value_name("FILE")
             .help("Destination model")
             .takes_value(true))
        .arg(Arg::with_name("trace")
             .display_order(1)
             .short("t")
             .long("trace")
             .help("Trace",)
             .takes_value(true));

    let matches = app.get_matches();
    let (source_model, dst_model) =
        if let Some(source_model_file) = matches.value_of("source-model") {
            if let Some(dst_model_file) = matches.value_of("dst-model") {
                (model::read_model_file(source_model_file).map_err(Error::Model)?,
                 model::read_model_file(dst_model_file).map_err(Error::Model)?)
            } else {
                let source_model = model::read_model_file(source_model_file).map_err(Error::Model)?;
                let dst_model = source_model.new_empty_of_same_size();
                (source_model, dst_model)
            }
        } else {
            if let Some(dst_model_file) = matches.value_of("dst-model") {
                let dst_model = model::read_model_file(dst_model_file).map_err(Error::Model)?;
                let source_model = dst_model.new_empty_of_same_size();
                (source_model, dst_model)
            } else {
                return Err(Error::NoSourceOrTargetModelProvided);
            }
        };

    let trace_filename = value_t!(matches, "trace", String).map_err(Error::Args)?;

    let mut f = File::open(&trace_filename).map_err(Error::Io)?;
    let mut buffer = Vec::new();
    f.read_to_end(&mut buffer).map_err(Error::Io)?;

    let mut state = kernel::state::State::new(source_model, vec![]);

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
            for voxel in state.matrix.filled_voxels() {
                if state.matrix.is_filled(voxel) && !dst_model.is_filled(voxel) {
                    return Err(Error::ModelNotMatch);
                }
            }
            for voxel in dst_model.filled_voxels() {
                if !state.matrix.is_filled(voxel) {
                    return Err(Error::ModelNotMatch);
                }
            }

            println!("SUCCESS");
            Ok(())
        }
    }
}


#[cfg(test)]
mod test {
    use super::*;
}
