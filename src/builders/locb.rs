use crate::builders::*;
use crate::morton::*;
use crate::utils::{prefix_sum, UnsafeSliceWrapper};
use crate::*;
use glam::Vec3;
use rayon::prelude::*;

pub struct LocallyOrderedClusteringBuilder<'a> {
    aabbs: &'a [AABB],
    centers: &'a [Vec3],
    encoder: MortonEncoder,
    world_bounds: AABB,
    search_radius: usize,
}

impl<'a> LocallyOrderedClusteringBuilder<'a> {
    pub fn new(aabbs: &'a [AABB], centers: &'a [Vec3]) -> Self {
        let world_bounds = AABB::union_of_list(aabbs);
        let encoder = MortonEncoder::new(&world_bounds, MortonEncoder::MAX_GRID_DIM);

        Self {
            aabbs,
            centers,
            encoder,
            world_bounds,
            search_radius: 14,
        }
    }

    pub fn with_search_radius(mut self, radius: usize) -> Self {
        self.search_radius = radius;
        self
    }

    fn search_range(&self, i: usize, begin: usize, end: usize) -> (usize, usize) {
        let begin = if i > (begin + self.search_radius) {
            i - self.search_radius
        } else {
            begin
        };
        let end = (i + self.search_radius + 1).min(end);

        (begin, end)
    }

    fn cluster(
        &self,
        input: &[BVHNode],
        output: &mut [BVHNode],
        neighbours: &mut [u32],
        merged_index: &mut [u32],
        begin: usize,
        end: usize,
        previous_end: usize,
    ) -> (usize, usize) {
        let num_threads = num_cpus::get();
        let chunk_size = (end - begin) / num_threads;

        let par_neighbours = UnsafeSliceWrapper::new(neighbours);

        (0..num_threads).into_iter().for_each(|thread_id| {
            let chunk_begin = begin + thread_id * chunk_size;
            let chunk_end = if thread_id != (num_threads - 1) {
                chunk_begin + chunk_size
            } else {
                end
            };

            let mut distances = vec![0.0 as f32; (self.search_radius + 1) * self.search_radius];
            let mut distance_matrix =
                vec![std::ptr::null_mut() as *mut f32; self.search_radius + 1];

            for i in 0..=self.search_radius {
                distance_matrix[i] = unsafe { distances.as_mut_ptr().add(i * self.search_radius) };
            }

            // Initialize distance matrix
            for i in self.search_range(chunk_begin, begin, end).0..chunk_begin {
                for j in (i + 1)..self.search_range(i, begin, end).1 {
                    unsafe {
                        (*distance_matrix[chunk_begin - i].add(j - i - 1)) =
                            input[i].bounds.union_of(&input[j].bounds).half_area();
                    }
                }
            }

            // Nearest neighbour search
            for i in chunk_begin..chunk_end {
                let (search_begin, search_end) = self.search_range(i, begin, end);
                let mut best_distance = std::f32::MAX;
                let mut best_neighbour = -1;

                // Backward search
                for j in search_begin..i {
                    let distance = unsafe { *distance_matrix[i - j].add(i - j - 1) };
                    debug_assert_ne!(distance, f32::INFINITY);

                    if distance < best_distance {
                        best_distance = distance;
                        best_neighbour = j as i32;
                    }
                }

                // Forward search
                for j in (i + 1)..search_end {
                    let distance = input[i].bounds.union_of(&input[j].bounds).half_area();
                    debug_assert_ne!(
                        distance,
                        f32::INFINITY,
                        "i {}, j {}, bounds1 {}, bound2 {}",
                        i,
                        j,
                        &input[i].bounds,
                        &input[j].bounds
                    );
                    unsafe {
                        (*distance_matrix[0].add(j - i - 1)) = distance;
                    }
                    if distance < best_distance {
                        best_distance = distance;
                        best_neighbour = j as i32;
                    }
                }

                debug_assert_ne!(best_neighbour, -1);
                par_neighbours.set(i, best_neighbour as u32);

                // Rotate the distance matrix columns
                unsafe {
                    let last = distance_matrix[self.search_radius];
                    utils::move_backward(
                        distance_matrix.as_mut_ptr(),
                        distance_matrix.as_mut_ptr().add(self.search_radius),
                        distance_matrix.as_mut_ptr().add(self.search_radius + 1),
                    );
                    distance_matrix[0] = last;
                }
            }
        });

        // Mark nodes that are the closest as merged, but keep
        // the one with lowest index to act as the parent
        #[cfg(feature = "wasm_support")]
        merged_index[begin..end]
            .iter_mut()
            .enumerate()
            .for_each(|(i, m)| {
                let i = begin + i;
                let j = neighbours[i] as usize;
                let is_mergeable = (neighbours[j] as usize) == i;
                *m = if i < j && is_mergeable { 1 } else { 0 };
            });

        #[cfg(not(feature = "wasm_support"))]
        merged_index[begin..end]
            .par_iter_mut()
            .enumerate()
            .for_each(|(i, m)| {
                let i = begin + i;
                let j = neighbours[i] as usize;
                let is_mergeable = (neighbours[j] as usize) == i;
                *m = if i < j && is_mergeable { 1 } else { 0 };
            });

        // Perform a prefix sum to compute the insertion indices
        unsafe {
            let count = end - begin;
            let start = std::slice::from_raw_parts(merged_index.as_ptr().add(begin), count);
            let output =
                std::slice::from_raw_parts_mut(merged_index.as_mut_ptr().add(begin), count);
            prefix_sum(start, count, output);
        }

        let merged_count = merged_index[end - 1] as usize;
        let unmerged_count = end - begin - merged_count;
        let children_count = merged_count * 2;
        let children_begin = end - children_count;
        let unmerged_begin = end - (children_count + unmerged_count);

        let next_begin = unmerged_begin;
        let next_end = children_begin;

        // Finally, merge nodes that are marked for merging and create
        // their parents using the indices computed previously.
        let par_output = UnsafeSliceWrapper::new(output);
        let neighbours: &[u32] = neighbours;
        let merged_index: &[u32] = merged_index;

        #[cfg(not(feature = "wasm_support"))]
        (begin..end).into_iter().for_each(|i| {
            let j = neighbours[i] as usize;
            if neighbours[j] as usize == i {
                if i < j {
                    let unmerged_node = par_output
                        .get_mut(unmerged_begin + j - begin - merged_index[j] as usize)
                        .unwrap();
                    let first_child = children_begin + (merged_index[i] as usize - 1) * 2;
                    unmerged_node.bounds = input[j].bounds.union_of(&input[i].bounds);
                    unmerged_node.count = -1;
                    unmerged_node.left_first = first_child as i32;

                    par_output.set(first_child + 0, input[i].clone());
                    par_output.set(first_child + 1, input[j].clone());
                }
            } else {
                par_output.set(
                    unmerged_begin + i - begin - (merged_index[i] as usize),
                    input[i].clone(),
                );
            }
        });

        #[cfg(feature = "wasm_support")]
        (begin..end).into_par_iter().for_each(|i| {
            let j = neighbours[i] as usize;
            if neighbours[j] as usize == i {
                if i < j {
                    let unmerged_node = par_output
                        .get_mut(unmerged_begin + j - begin - merged_index[j] as usize)
                        .unwrap();
                    let first_child = children_begin + (merged_index[i] as usize - 1) * 2;
                    unmerged_node.bounds = input[j].bounds.union_of(&input[i].bounds);
                    unmerged_node.count = -1;
                    unmerged_node.left_first = first_child as i32;

                    par_output.set(first_child + 0, input[i].clone());
                    par_output.set(first_child + 1, input[j].clone());
                }
            } else {
                par_output.set(
                    unmerged_begin + i - begin - (merged_index[i] as usize),
                    input[i].clone(),
                );
            }
        });

        // Copy the nodes of the previous level into the current array of nodes.
        for i in end..previous_end {
            output[i] = input[i].clone();
        }

        (next_begin, next_end)
    }
}

impl<'a> Builder for LocallyOrderedClusteringBuilder<'a> {
    fn build(self) -> BVH {
        debug_assert!(!self.aabbs.is_empty());

        let prim_count = self.aabbs.len();
        if prim_count <= 2 {
            // Splitting not worth it, just create a single root leaf node
            return BVH {
                nodes: vec![BVHNode {
                    bounds: self.world_bounds,
                    left_first: 0,
                    count: prim_count as i32,
                }],
                prim_indices: (0..prim_count).into_iter().map(|i| i as u32).collect(),
            };
        }

        let (prim_indices, _) = self.encoder.get_sorted_indices(self.aabbs, self.centers);
        let node_count = 2 * prim_count - 1;

        let mut nodes = vec![
            BVHNode {
                bounds: (Vec3::zero(), Vec3::zero()).into(),
                count: -1,
                left_first: 0,
            };
            node_count
        ];
        let mut nodes_copy = nodes.clone();
        let mut auxiliary_data = vec![node_count as u32; node_count * 3];

        let mut begin = node_count - prim_count;
        let mut end = node_count;
        let mut previous_end = end;

        for i in 0..prim_count {
            let node = &mut nodes[begin + i];
            node.bounds = self.aabbs[prim_indices[i] as usize];
            node.count = 1;
            node.left_first = i as i32;
        }

        let (aux_slice0, aux_slice1) = unsafe {
            (
                std::slice::from_raw_parts_mut(auxiliary_data.as_mut_ptr(), node_count * 3),
                std::slice::from_raw_parts_mut(
                    auxiliary_data.as_mut_ptr().add(node_count),
                    node_count * 2,
                ),
            )
        };

        while end - begin > 1 {
            let (next_begin, next_end) = self.cluster(
                nodes.as_slice(),
                nodes_copy.as_mut_slice(),
                aux_slice0,
                aux_slice1,
                begin,
                end,
                previous_end,
            );

            std::mem::swap(&mut nodes, &mut nodes_copy);
            previous_end = end;
            begin = next_begin;
            end = next_end;
        }

        BVH {
            nodes,
            prim_indices,
        }
    }
}
