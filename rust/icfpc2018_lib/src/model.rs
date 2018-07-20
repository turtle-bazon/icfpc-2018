use std::io::{self, Read};

use super::coord::{
    M,
    Coord,
    Matrix,
    Resolution,
};

#[derive(Debug)]
pub enum ModelError {
    ResolutionRead(io::Error),
    VoxelsRead(io::Error),
}

pub fn read_model<R>(mut reader: R) -> Result<Matrix, ModelError> where R: Read {
    let mut buf = [0u8];
    reader.read_exact(&mut buf)
        .map_err(ModelError::ResolutionRead)?;
    let dim = buf[0] as usize;
    let res = Resolution(dim as M);

    let bytes_total = ((dim * dim * dim) + 7) / 8;
    let mut bytes: Vec<u8> = (0 .. bytes_total).map(|_| 0).collect();
    reader.read_exact(&mut bytes)
        .map_err(ModelError::VoxelsRead)?;

    let mut coord = Coord { x: 0, y: 0, z: 0, };
    let coords_iter = bytes
        .into_iter()
        .flat_map(|byte| (0 .. 8).map(move |bit_i| (byte, bit_i)))
        .flat_map(|(byte, bit_i)| {
            let item = if byte & (1 << bit_i) == 0 {
                None
            } else {
                Some(coord)
            };
            coord.z += 1;
            if coord.z as usize >= dim {
                coord.z = 0;
                coord.y += 1;
            }
            if coord.y as usize >= dim {
                coord.y = 0;
                coord.z += 1;
            }
            if coord.z as usize >= dim {
                None
            } else {
                item
            }
        });
    Ok(Matrix::from_iter(res, coords_iter))
}
