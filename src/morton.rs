#[allow(dead_code)]
use crate::{utils::*, *};
use glam::*;

#[cfg(not(feature = "wasm_support"))]
use rayon::prelude::*;

/// Split an unsigned integer such that its bits are spaced by 2 zeros.
/// For instance, morton_split(0b00110010) = 0b000000001001000000001000.
pub fn morton_split(int: u32) -> u32 {
    let log_bits = round_up_log2(std::mem::size_of::<u32>() as u32 * 8, 0);
    let mut x = int as usize;
    let mut mask = !(0 as usize);
    let mut i = log_bits as usize;
    let mut n = 1 << log_bits as usize;
    while i > 0 {
        mask = (mask | (mask << n)) & !(mask << (n / 2));
        x = (x | (x << n)) & mask;

        n >>= 1;
        i -= 1;
    }

    x as u32
}

pub struct MortonEncoder {
    world_to_grid: Vec3A,
    grid_offset: Vec3A,
    grid_dim: usize,
}

impl MortonEncoder {
    pub const MAX_GRID_DIM: usize = 1 << (std::mem::size_of::<u32>() * 8 / 3);

    pub fn new(aabb: &AABB, grid_dim: usize) -> MortonEncoder {
        debug_assert!(grid_dim <= Self::MAX_GRID_DIM);
        let world_to_grid = grid_dim as f32 * (1.0 / aabb.diagonal::<Vec3A>());
        let grid_offset = -Vec3A::from(aabb.min) * world_to_grid;

        Self {
            world_to_grid,
            grid_offset,
            grid_dim,
        }
    }

    pub fn morton_encode(x: u32, y: u32, z: u32) -> u32 {
        morton_split(x) | (morton_split(y) << 1) | (morton_split(z) << 2)
    }

    pub fn encode(&self, point: Vec3A) -> u32 {
        let grid_pos = point * self.world_to_grid + self.grid_offset;
        let min = (self.grid_dim - 1) as i32;

        let x: u32 = min.min((grid_pos[0] as i32).max(0)) as u32;
        let y: u32 = min.min((grid_pos[1] as i32).max(0)) as u32;
        let z: u32 = min.min((grid_pos[2] as i32).max(0)) as u32;

        Self::morton_encode(x, y, z)
    }

    pub fn get_sorted_indices(&self, aabbs: &[AABB], centers: &[Vec3A]) -> (Vec<u32>, Vec<u32>) {
        debug_assert_eq!(aabbs.len(), centers.len());
        let prim_count = aabbs.len();

        let mut indices: Vec<u32> = (0..(prim_count as u32)).collect();

        #[cfg(not(feature = "wasm_support"))]
        let morton_codes: Vec<u32> = (0..prim_count)
            .into_par_iter()
            .map(|i| self.encode(centers[i]))
            .collect();

        #[cfg(feature = "wasm_support")]
        let morton_codes: Vec<u32> = (0..prim_count)
            .into_iter()
            .map(|i| self.encode(centers[i]))
            .collect();

        #[cfg(not(feature = "wasm_support"))]
        indices.par_sort_by(|a, b| {
            let a = (*a) as usize;
            let b = (*b) as usize;

            morton_codes[a].cmp(&morton_codes[b])
        });

        #[cfg(feature = "wasm_support")]
        indices.sort_by(|a, b| {
            let a = (*a) as usize;
            let b = (*b) as usize;

            morton_codes[a].cmp(&morton_codes[b])
        });

        (indices, morton_codes)
    }
}

#[cfg(test)]
mod tests {
    use crate::morton::*;

    #[test]
    fn morton_split_works() {
        assert_eq!(morton_split(2), 8);
        assert_eq!(morton_split(4), 64);
        assert_eq!(morton_split(8), 512);
        assert_eq!(morton_split(32), 32768);
    }
}
