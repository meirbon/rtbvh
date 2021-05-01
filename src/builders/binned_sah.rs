use crate::{
    builders::{AtomicNodeStack, BuildAlgorithm},
    Primitive,
};
use crate::{utils::*, BuildType};
use crate::{Aabb, Bvh, BvhNode};
use glam::*;
use std::fmt::Debug;
use std::sync::atomic::AtomicUsize;

#[derive(Debug, Copy, Clone)]
struct SahBin {
    pub aabb: Aabb,
    pub prim_count: usize,
    pub right_cost: f32,
}

#[derive(Debug, Copy, Clone)]
struct SahSplit {
    cost: f32,
    count: u32,
}

impl Default for SahSplit {
    fn default() -> Self {
        Self {
            cost: 0.0,
            count: 0,
        }
    }
}

struct BinnedSahBuildTask<'a, T: Primitive<i32>> {
    bins_per_axis: [Vec<SahBin>; 3],
    builder: &'a BinnedSahBuilder<'a, T>,
    allocator: AtomicNodeStack<'a>,
    prim_indices: &'a mut [u32],

    pub node: &'a mut BvhNode,
    pub begin: usize,
    pub end: usize,
    pub depth: usize,
}

impl<'a, T: Primitive<i32>> BinnedSahBuildTask<'a, T> {
    pub fn new(
        builder: &'a BinnedSahBuilder<'a, T>,
        allocator: AtomicNodeStack<'a>,
        prim_indices: &'a mut [u32],
        node: &'a mut BvhNode,
        begin: usize,
        end: usize,
        depth: usize,
    ) -> Self {
        // Setup bins
        let bin_array = vec![
            SahBin {
                aabb: Aabb::empty(),
                prim_count: 0,
                right_cost: std::f32::MAX,
            };
            builder.bin_count
        ];

        let bins_per_axis = [bin_array.clone(), bin_array.clone(), bin_array];

        Self {
            bins_per_axis,
            builder,
            allocator,
            prim_indices,
            node,
            begin,
            end,
            depth,
        }
    }

    pub fn find_split(&mut self, axis: usize) -> SahSplit {
        let bins = self.bins_per_axis[axis].as_mut_slice();

        let mut current_box = Aabb::empty();
        let mut current_count = 0;

        let mut i = self.builder.bin_count - 1;
        while i > 0 {
            current_box.grow_bb(&bins[i].aabb);
            current_count += bins[i].prim_count;
            bins[i].right_cost = current_box.half_area() * current_count as f32;
            i -= 1;
        }

        let mut current_box = Aabb::empty();
        let mut current_count = 0;
        let mut best_split = SahSplit {
            cost: std::f32::MAX,
            count: self.builder.bin_count as u32,
        };

        for i in 0..(self.builder.bin_count - 1) {
            current_box.grow_bb(&bins[i].aabb);
            current_count += bins[i].prim_count;
            let cost = current_box.half_area() * current_count as f32 + bins[i + 1].right_cost;
            if cost < best_split.cost {
                best_split = SahSplit {
                    cost,
                    count: i as u32 + 1,
                }
            }
        }

        best_split
    }
}

impl<'a, T: Primitive<i32>> BinnedSahBuildTask<'a, T> {
    #[inline]
    fn compute_bin_index(
        center: Vec3,
        bin_count: usize,
        bin_offset: Vec3,
        center_to_bin: Vec3,
        axis: usize,
    ) -> usize {
        let bin_index = center[axis] * center_to_bin[axis] + bin_offset[axis];
        (bin_count - 1).min(bin_index.max(0.0) as usize)
    }
}

impl<'a, T: Primitive<i32>> Task for BinnedSahBuildTask<'a, T> {
    fn run(mut self) -> Option<(Self, Self)> {
        self.node.bounds.offset_by(0.0001);
        let make_leaf = |node: &mut BvhNode, begin: usize, end: usize| {
            node.bounds.offset_by(0.0001);
            node.set_left_first(Some(begin as u32));
            node.set_count(Some((end - begin) as u32));
        };

        if self.work_size() <= 1 || self.depth >= self.builder.max_depth {
            make_leaf(self.node, self.begin, self.end);
            return None;
        }

        let bin_count = self.builder.bin_count;
        let center_to_bin = (Vec3::ONE / self.node.bounds.diagonal()) * bin_count as f32;
        let bin_offset: Vec3 = -self.node.bounds.min * center_to_bin;

        self.bins_per_axis.iter_mut().for_each(|bins| {
            bins.iter_mut().for_each(|b| {
                b.aabb = Aabb::empty();
                b.prim_count = 0;
            })
        });

        // Fill bins with primitives
        for i in 0..self.work_size() {
            let p_id = self.prim_indices[i] as usize;
            for axis in 0..3 {
                let bin_index = Self::compute_bin_index(
                    self.builder.primitives[p_id].center(),
                    bin_count,
                    bin_offset,
                    center_to_bin,
                    axis,
                );
                self.bins_per_axis[axis][bin_index].prim_count += 1;
                self.bins_per_axis[axis][bin_index]
                    .aabb
                    .grow_bb(&self.builder.aabbs[p_id]);
            }
        }

        let mut best_splits: [SahSplit; 3] = [SahSplit::default(); 3];
        for (axis, split) in best_splits.iter_mut().enumerate() {
            *split = self.find_split(axis);
        }

        let mut best_axis = 0_usize;
        if best_splits[0].cost > best_splits[1].cost {
            best_axis = 1;
        }
        if best_splits[best_axis].cost > best_splits[2].cost {
            best_axis = 2;
        }

        // Make sure the cost of splitting does not exceed the cost of not splitting
        let mut split_index = best_splits[best_axis].count as usize;
        let max_split_cost =
            self.node.bounds.half_area() * (self.work_size() as f32 - self.builder.traversal_cost);
        if best_splits[best_axis].count as usize == bin_count
            || best_splits[best_axis].cost >= max_split_cost
        {
            if self.work_size() > self.builder.max_leaf_size {
                // Fallback strategy: approximate median split on largest axis
                best_axis = self.node.bounds.longest_axis();
                let mut count = 0;
                for i in 0..(bin_count - 1) {
                    count += self.bins_per_axis[best_axis][i].prim_count as usize;
                    // Split when we reach 0.4 times the number of primitives in the node
                    if count >= (self.work_size() * 2 / 5 + 1) {
                        split_index = i + 1;
                        break;
                    }
                }
            } else {
                make_leaf(self.node, self.begin, self.end);
                return None;
            }
        }

        let primitives = self.builder.primitives;
        let begin_right = self.begin
            + partition(self.prim_indices, 0..(self.end - self.begin), |i| {
                let i = *i;
                let center = primitives[i as usize].center();
                Self::compute_bin_index(center, bin_count, bin_offset, center_to_bin, best_axis)
                    < split_index
            });

        // Check if the split does not leave one side empty
        if begin_right > self.begin && begin_right < self.end {
            // Allocate two nodes
            let new_nodes = self.allocator.allocate().unwrap();
            self.node.set_left_first(Some(new_nodes.left as u32));
            self.node.set_count(None);

            // Compute the bounding boxes
            let mut left_box = Aabb::empty();
            let mut right_box = Aabb::empty();

            let bins = &self.bins_per_axis[best_axis];
            for bin in bins[0..(best_splits[best_axis].count as usize)].iter() {
                left_box.grow_bb(&bin.aabb);
            }

            for bin in bins[split_index..bin_count].iter() {
                right_box.grow_bb(&bin.aabb);
            }

            // Retrieve references to new nodes
            let left_node = new_nodes.left_node;
            let right_node = new_nodes.right_node;

            // Set their bounds
            left_node.bounds = left_box;
            right_node.bounds = right_box;

            // Split indices in a safe way
            let (left_indices, right_indices) =
                self.prim_indices.split_at_mut(begin_right - self.begin);

            // Spawn new split tasks for each node
            let first_item = Self::new(
                self.builder,
                self.allocator.clone(),
                left_indices,
                left_node,
                self.begin,
                begin_right,
                self.depth + 1,
            );

            let second_item = Self::new(
                self.builder,
                self.allocator,
                right_indices,
                right_node,
                begin_right,
                self.end,
                self.depth + 1,
            );

            // Return new tasks
            return Some((first_item, second_item));
        }

        // Make this node a leaf node
        make_leaf(self.node, self.begin, self.end);
        // Node became a leaf node so do not spawn new tasks
        None
    }

    fn work_size(&self) -> usize {
        self.end - self.begin
    }

    fn depth(&self) -> usize {
        self.depth
    }
}

pub struct BinnedSahBuilder<'a, T: Primitive<i32>> {
    aabbs: &'a [Aabb<i32>],
    primitives: &'a [T],
    max_depth: usize,
    bin_count: usize,
    max_leaf_size: usize,
    traversal_cost: f32,
}

impl<'a, T: Primitive<i32>> BinnedSahBuilder<'a, T> {
    pub fn new(aabbs: &'a [Aabb<i32>], primitives: &'a [T]) -> Self {
        debug_assert_eq!(aabbs.len(), primitives.len());
        Self {
            aabbs,
            primitives,
            max_depth: 64,
            bin_count: 16,
            max_leaf_size: 5,
            traversal_cost: 1.0,
        }
    }

    /// Maximum depth of tree
    pub fn with_max_depth(mut self, depth: usize) -> Self {
        self.max_depth = depth;
        self
    }

    /// Maximum number of bins to check for splitting a node
    pub fn with_bin_count(mut self, bin_count: usize) -> Self {
        self.bin_count = bin_count;
        self
    }

    /// Maximum number of primitives inside a node
    pub fn with_max_leaf_size(mut self, max_leaf_size: usize) -> Self {
        self.max_leaf_size = max_leaf_size;
        self
    }

    /// Cost to determine for splitting a node with the SAH algorithm for traversing a node
    pub fn with_traversal_cost(mut self, traversal_cost: f32) -> Self {
        self.traversal_cost = traversal_cost;
        self
    }
}

impl<'a, T: Primitive<i32>> BuildAlgorithm for BinnedSahBuilder<'a, T> {
    fn build(self) -> Bvh {
        let prim_count = self.aabbs.len();
        let mut nodes = vec![BvhNode::new(); prim_count * 2 - 1];
        let mut prim_indices: Vec<u32> = (0..prim_count).into_iter().map(|i| i as u32).collect();

        let task_spawner = TaskSpawner::new();
        let node_count = AtomicUsize::new(1);
        let (node_stack, first_node) = AtomicNodeStack::new(&node_count, nodes.as_mut_slice());
        first_node.bounds = Aabb::union_of_list(self.aabbs);

        // Create first build task
        let root_task = BinnedSahBuildTask::new(
            &self,
            node_stack,
            prim_indices.as_mut(),
            first_node,
            0,
            prim_count,
            0,
        );

        #[cfg(feature = "wasm_support")]
        {
            let mut stack = vec![root_task];
            while let Some(task) = stack.pop() {
                if let Some(left_task, right_task) = task.run() {
                    stack.push(left);
                    stack.push(right);
                }
            }
        }

        // Build bvh
        #[cfg(not(feature = "wasm_support"))]
        task_spawner.run(root_task);

        Bvh {
            nodes,
            prim_indices,
            build_type: BuildType::BinnedSAH,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::spatial_sah::SpatialTriangle;
    use crate::Bounds;

    #[test]
    fn test_binned_sah_build() {
        let (aabbs, primitives) = crate::tests::load_teapot();

        let builder = BinnedSahBuilder::new(aabbs.as_slice(), primitives.as_slice());
        let bvh = builder.build();

        let bounds = bvh.bounds();
        assert!(bounds.is_valid());
        assert!(bvh.validate(primitives.len()));

        for (i, t) in primitives.iter().enumerate() {
            assert!(
                bounds.contains(t.vertex0()),
                "Bvh did not contain vertex 0 of primitive {}, bvh-bounds: {}, vertex: {}",
                i,
                bounds,
                Vec3::from(t.vertex0())
            );
            assert!(
                bounds.contains(t.vertex1()),
                "Bvh did not contain vertex 1 of primitive {}, bvh-bounds: {}, vertex: {}",
                i,
                bounds,
                Vec3::from(t.vertex1())
            );
            assert!(
                bounds.contains(t.vertex2()),
                "Bvh did not contain vertex 2 of primitive {}, bvh-bounds: {}, vertex: {}",
                i,
                bounds,
                Vec3::from(t.vertex2())
            );
        }
    }
}
