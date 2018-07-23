extern crate icfpc2018_lib;
#[macro_use] extern crate clap;

use clap::Arg;
use std::fs::File;
use std::io::Read;
use std::io::Write;

use icfpc2018_lib as kernel;
use kernel::cmd::BotCommand;
use kernel::state::Wellness;
use kernel::coord::{Matrix,Resolution};

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
             .default_value("")
             .takes_value(true))
        .arg(Arg::with_name("trace")
             .display_order(1)
             .short("t")
             .long("trace")
             .help("Original trace (In)",)
             .default_value("")
             .takes_value(true))
        .arg(Arg::with_name("fliptrace")
             .display_order(2)
             .short("f")
             .long("flip-trace")
             .help("Flipped trace (Out)",)
             .default_value("tmp.nbt")
             .takes_value(true));

    let matches = app.get_matches();
    let model_filename = value_t!(matches, "model", String).map_err(Error::Args)?;
    let trace_filename = value_t!(matches, "trace", String).map_err(Error::Args)?;
    let out_trace_filename = value_t!(matches, "fliptrace", String).map_err(Error::Args)?;

    let matrix = if model_filename != "" {
        match kernel::model::read_model_file(model_filename) {
            Err(e) => return Err(Error::ModelReadError(e)),
            Ok(m) => m,
        }
    } else {
        Matrix::new(Resolution(250))
    };

    let mut f = File::open(&trace_filename).map_err(Error::Io)?;
    let mut buffer = Vec::new();
    f.read_to_end(&mut buffer).map_err(Error::Io)?;

    //let matrix = ref_model.new_empty_of_same_size();
    //let matrix = Matrix::new(Resolution(250));
    let mut state = kernel::state::State::new(matrix, vec![]);

    let cmds = kernel::cmd::from_bytes(&buffer).map_err(Error::Cmd)?;
    println!("Commands: {}", cmds.len());

    let res = state.stateless_run(&cmds);
    match res {
        Err(e) => {
            println!("ERROR: {:?}", e);
            Err(Error::State(e))
        },
        Ok(script) => {
            println!("SUCCESS");
            let true_threshold = 2;
            let flipper = {
                let mut cur = true;
                let mut flip = Vec::new();
                for (i,s) in script.iter().enumerate() {
                    if s.ok == cur { continue; }
                    //println!("{} {:?}",i,s);
                    flip.push((i,s));
                    cur = s.ok;
                }
                //println!("{} {:?}",script.len()-1,script[script.len()-1]);
                let mut new_flip = Vec::new();
                flip.push((script.len()-1,&script[script.len()-1]));
                
                for (k,(i,s)) in flip[0 .. flip.len()-1].iter().enumerate() {
                    let cnt = flip[k+1].0 - *i;
                    if (cnt<true_threshold) && s.ok {
                        new_flip.push((cnt,Wellness {
                            ok: false,
                            bots: s.bots,
                            offset: s.offset,
                        }));
                    } else {
                        new_flip.push((cnt,**s));
                    }
                }
                let mut idx = 1;
                while idx<new_flip.len() {
                    if new_flip[idx-1].1.ok == new_flip[idx].1.ok {
                        new_flip[idx-1].0 += new_flip[idx].0;
                        new_flip.remove(idx);
                        continue;
                    }
                    idx += 1;
                }
                /*let mut s = 0;
                for cs in &new_flip {
                    s += cs.0;
                    println!("{} {} {:?}",s,cs.0,cs.1);
                }*/
                new_flip
            };
            let mut cur_offset = 0;
            let mut new_cmd = Vec::with_capacity(cmds.len()+flipper.len()*40);
            let mut cmd_iter = cmds.into_iter();
            let mut sfl = 0;
            for s in flipper {
                for _ in 0 .. s.1.offset-cur_offset {   
                    new_cmd.push(cmd_iter.next().unwrap());
                }
                cur_offset = s.1.offset;
                new_cmd.push(BotCommand::Flip);
                for _ in 1 .. s.1.bots {
                    new_cmd.push(BotCommand::Wait);
                }
                sfl+=1;
            }
            new_cmd.extend(cmd_iter);
            if sfl % 2 !=0 {
                panic!("FLIPS: {}",sfl);
            }
            let buffer = kernel::cmd::into_bytes(&new_cmd).unwrap();
            {
                let mut f = File::create(&out_trace_filename).map_err(Error::Io)?;
                f.write_all(&buffer).map_err(Error::Io)?;
            }
            Ok(())
        }
    }
}


