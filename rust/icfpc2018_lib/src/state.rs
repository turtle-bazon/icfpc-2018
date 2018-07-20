
use super::coord::Matrix;

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub enum Harmonics {
    Low,
    High,
}

#[derive(Debug)]
pub struct Bot;

#[derive(Debug)]
pub struct Command;

#[derive(Debug)]
pub struct State {
    pub energy: usize,
    pub harmonics: Harmonics,
    pub matrix: Matrix,
    pub bots: Vec<Bot>,
    pub trace: Vec<Command>,
}
