extern crate env_logger;
extern crate icfpc2018_lib;
#[macro_use] extern crate log;
#[macro_use] extern crate clap;

use std::process;
use clap::Arg;

use icfpc2018_lib::{
    coord::{
        M,
        Matrix,
        Resolution,
    },
    model,
};

fn main() {
    env_logger::init();
    match run() {
        Ok(()) =>
            info!("graceful shutdown"),
        Err(e) => {
            error!("fatal error: {:?}", e);
            process::exit(1);
        },
    }
}

#[derive(Debug)]
enum Error {
    MissingParameter(&'static str),
    Model(model::Error),
}

fn run() -> Result<(), Error> {
    let matches = app_from_crate!()
        .arg(Arg::with_name("source-model")
             .short("s")
             .long("source-model")
             .value_name("FILE")
             .help("Source model")
             .takes_value(true))
        .arg(Arg::with_name("target-model")
             .short("t")
             .long("target-model")
             .value_name("FILE")
             .help("Target model")
             .default_value("../../problems/FA004_tgt.mdl")
             .takes_value(true))
        .get_matches();

    let target_model_file = matches.value_of("target-model")
        .ok_or(Error::MissingParameter("target-model"))?;
    let target_model = model::read_model_file(target_model_file)
            .map_err(Error::Model)?;
    let source_model = if let Some(source_model_file) = matches.value_of("souce-model") {
        model::read_model_file(source_model_file)
            .map_err(Error::Model)?
    } else {
        Matrix::new(Resolution(target_model.dim() as M))
    };

    unimplemented!()
}
