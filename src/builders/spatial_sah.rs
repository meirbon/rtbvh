use crate::{
    builders::{AtomicNodeStack, BuildAlgorithm},
    Primitive, Ray,
};
use crate::{utils::*, BuildType};
use crate::{Aabb, Bvh, BvhNode};
use glam::*;
use rayon::prelude::*;
use std::num::NonZeroUsize;
use std::{
    cmp::Ordering,
    fmt::Debug,
    ops::BitAnd,
    sync::atomic::{AtomicUsize, Ordering::SeqCst},
};

#[derive(Debug, Copy, Clone)]
struct SpatialBin {
    pub aabb: Aabb<i32>,
    pub accumulated_aabb: Aabb,
    pub entry: usize,
    pub exit: usize,
}

#[derive(Debug, Copy, Clone)]
struct ObjectSplit {
    pub cost: f32,
    pub index: usize,
    pub axis: usize,
    pub left_box: Aabb<i32>,
    pub right_box: Aabb<i32>,
}

impl Default for ObjectSplit {
    fn default() -> Self {
        Self {
            cost: std::f32::MAX,
            index: 1,
            axis: 0,
            left_box: Aabb::empty(),
            right_box: Aabb::empty(),
        }
    }
}

#[derive(Debug, Copy, Clone)]
struct SpatialSplit {
    pub cost: f32,
    pub position: f32,
    pub axis: usize,
}

impl Default for SpatialSplit {
    fn default() -> Self {
        Self {
            cost: std::f32::MAX,
            position: 0.0,
            axis: 0,
        }
    }
}

#[derive(Debug, Copy, Clone)]
struct SpatialReference {
    aabb: Aabb<i32>,
    center: Vec3,
    prim_id: u32,
}

impl Default for SpatialReference {
    fn default() -> Self {
        Self {
            aabb: Aabb::empty(),
            center: Vec3::ZERO,
            prim_id: 0,
        }
    }
}

pub trait SpatialTriangle {
    fn vertex0(&self) -> Vec3;
    fn vertex1(&self) -> Vec3;
    fn vertex2(&self) -> Vec3;

    fn split(&self, axis: usize, position: f32) -> (Aabb<i32>, Aabb<i32>) {
        let p = [self.vertex0(), self.vertex1(), self.vertex2()];

        let mut left = Aabb::empty();
        let mut right = Aabb::empty();

        let split_edge = |a: Vec3, b: Vec3| -> Vec3 {
            let t = (Vec3::splat(position) - Vec3::splat(a[axis])) / Vec3::splat(b[axis] - a[axis]);
            a * t * (b - a)
        };

        let q0 = p[0][axis] <= position;
        let q1 = p[1][axis] <= position;
        let q2 = p[2][axis] <= position;

        let mut grow_if = |q: bool, pos: Vec3| {
            if q {
                left.grow(pos);
            } else {
                right.grow(pos);
            }
        };

        grow_if(q0, p[0]);
        grow_if(q1, p[1]);
        grow_if(q2, p[2]);

        if q0 ^ q1 {
            let m = split_edge(p[0], p[1]);
            left.grow(m);
            right.grow(m);
        }
        if q1 ^ q2 {
            let m = split_edge(p[1], p[2]);
            left.grow(m);
            right.grow(m);
        }
        if q2 ^ q0 {
            let m = split_edge(p[2], p[0]);
            left.grow(m);
            right.grow(m);
        }

        (left, right)
    }

    #[inline]
    fn intersect(&self, ray: &mut Ray) -> bool {
        let v0 = self.vertex0();
        let v1 = self.vertex1();
        let v2 = self.vertex2();

        let edge1 = v1 - v0;
        let edge2 = v2 - v0;
        let h_val = ray.direction.cross(edge2);
        let a_val = edge1.dot(h_val);
        if a_val > -1e-5 && a_val < 1e-5 {
            return false;
        }
        let f_val = 1.0 / a_val;
        let s_val = ray.origin - v0.xyz();
        let u_val = f_val * s_val.dot(h_val);
        if !(0.0..=1.0).contains(&u_val) {
            return false;
        }
        let q_val = s_val.cross(edge1);
        let v_val = f_val * ray.direction.dot(q_val);
        if v_val < 0. || (u_val + v_val) > 1.0 {
            return false;
        }

        let t: f32 = f_val * edge2.dot(q_val);
        if t > ray.t_min && t < ray.t {
            ray.t = t;
            true
        } else {
            false
        }
    }

    #[inline]
    fn intersect4(&self, packet: &mut crate::RayPacket4, t_min: Vec4) -> Option<[bool; 4]> {
        let v0 = self.vertex0();
        let v1 = self.vertex1();
        let v2 = self.vertex2();

        let zero = Vec4::ZERO;
        let one = Vec4::ONE;

        let p0_x = v0.xxxx();
        let p0_y = v0.yyyy();
        let p0_z = v0.zzzz();

        let p1_x = v1.xxxx();
        let p1_y = v1.yyyy();
        let p1_z = v1.zzzz();

        let p2_x = v2.xxxx();
        let p2_y = v2.yyyy();
        let p2_z = v2.zzzz();

        let edge1_x = p1_x - p0_x;
        let edge1_y = p1_y - p0_y;
        let edge1_z = p1_z - p0_z;

        let edge2_x = p2_x - p0_x;
        let edge2_y = p2_y - p0_y;
        let edge2_z = p2_z - p0_z;

        let h_x = (packet.direction_y * edge2_z) - (packet.direction_z * edge2_y);
        let h_y = (packet.direction_z * edge2_x) - (packet.direction_x * edge2_z);
        let h_z = (packet.direction_x * edge2_y) - (packet.direction_y * edge2_x);

        let a = (edge1_x * h_x) + (edge1_y * h_y) + (edge1_z * h_z);
        let epsilon = Vec4::from([1e-6; 4]);
        let mask = a.cmple(-epsilon) | a.cmpge(epsilon);
        if mask.bitmask() == 0 {
            return None;
        }

        let f = one / a;
        let s_x = packet.origin_x - p0_x;
        let s_y = packet.origin_y - p0_y;
        let s_z = packet.origin_z - p0_z;

        let u = f * ((s_x * h_x) + (s_y * h_y) + (s_z * h_z));
        let mask = mask.bitand(u.cmpge(zero) & u.cmple(one));
        if mask.bitmask() == 0 {
            return None;
        }

        let q_x = s_y * edge1_z - s_z * edge1_y;
        let q_y = s_z * edge1_x - s_x * edge1_z;
        let q_z = s_x * edge1_y - s_y * edge1_x;

        let v = f
            * ((packet.direction_x * q_x)
                + (packet.direction_y * q_y)
                + (packet.direction_z * q_z));
        let mask = mask.bitand(v.cmpge(zero) & (u + v).cmple(one));
        if mask.bitmask() == 0 {
            return None;
        }

        let t_value = f * ((edge2_x * q_x) + (edge2_y * q_y) + (edge2_z * q_z));
        let mask = mask.bitand(t_value.cmpge(t_min) & t_value.cmplt(packet.t));
        let bitmask = mask.bitmask();
        if bitmask == 0 {
            return None;
        }

        packet.t = Vec4::select(mask, t_value, packet.t);

        Some([
            bitmask & 1 != 0,
            bitmask & 2 != 0,
            bitmask & 4 != 0,
            bitmask & 8 != 0,
        ])
    }
}

struct WorkItem {
    pub node: usize,
    pub begin: usize,
    pub end: usize,
    pub split_end: usize,
    pub depth: usize,
    pub is_sorted: bool,
}

impl WorkItem {
    pub(crate) fn new(
        node: usize,
        begin: usize,
        end: usize,
        split_end: usize,
        depth: usize,
        is_sorted: bool,
    ) -> Self {
        Self {
            node,
            begin,
            end,
            split_end,
            depth,
            is_sorted,
        }
    }
    pub(crate) fn work_size(&self) -> usize {
        self.end - self.begin
    }
}

struct SpatialSahBuildTask<'a, T: Primitive<i32> + SpatialTriangle> {
    builder: &'a SpatialSahBuilder<'a, T>,
    allocator: AtomicNodeStack<'a>,
    references: [UnsafeSliceWrapper<'a, SpatialReference>; 3],
    reference_count: &'a AtomicUsize,
    prim_indices: UnsafeSliceWrapper<'a, u32>,
    accumulated_aabbs: UnsafeSliceWrapper<'a, Aabb>,
    work_item: WorkItem,
    spatial_threshold: f32,
}

impl<'a, T: Primitive<i32> + SpatialTriangle> SpatialSahBuildTask<'a, T> {
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn new(
        builder: &'a SpatialSahBuilder<'a, T>,
        allocator: AtomicNodeStack<'a>,
        references: [UnsafeSliceWrapper<'a, SpatialReference>; 3],
        reference_count: &'a AtomicUsize,
        prim_indices: UnsafeSliceWrapper<'a, u32>,
        accumulated_aabbs: UnsafeSliceWrapper<'a, Aabb>,
        work_item: WorkItem,
        spatial_threshold: f32,
    ) -> Self {
        Self {
            builder,
            allocator,
            references,
            reference_count,
            prim_indices,
            accumulated_aabbs,
            work_item,
            spatial_threshold,
        }
    }

    fn find_object_split(&self, begin: usize, end: usize, is_sorted: bool) -> ObjectSplit {
        if !is_sorted {
            // Sort references by the projection of their centers on this axis
            for axis in 0..3 {
                let slice = self.references[axis].range(begin, end);
                slice.sort_by(|a, b| {
                    let a_center = a.center[axis];
                    let b_center = b.center[axis];

                    if a_center < b_center {
                        std::cmp::Ordering::Less
                    } else if (a_center - b_center).abs() < 0.0001 {
                        std::cmp::Ordering::Equal
                    } else {
                        std::cmp::Ordering::Greater
                    }
                });
            }
        }

        let mut best_split = ObjectSplit::default();

        for axis in 0..3 {
            let mut aabb = Aabb::empty();
            let mut i = end - 1;
            while i > begin {
                aabb.grow_bb(&self.references[axis][i].aabb);
                self.accumulated_aabbs.set(i, aabb);
                i -= 1;
            }

            aabb = Aabb::empty();
            for i in begin..(end - 1) {
                aabb.grow_bb(&self.references[axis][i].aabb);
                let cost = aabb.half_area() * (i + 1 - begin) as f32
                    + self.accumulated_aabbs[i + 1].half_area() * (end - (i + 1)) as f32;
                if cost < best_split.cost {
                    best_split = ObjectSplit {
                        cost,
                        axis,
                        index: i + 1,
                        left_box: aabb,
                        right_box: self.accumulated_aabbs[i + 1],
                    }
                }
            }
        }

        best_split
    }

    pub(crate) fn allocate_children(
        &mut self,
        right_begin: usize,
        right_end: usize,
        left_box: Aabb,
        right_box: Aabb,
        is_sorted: bool,
    ) -> (WorkItem, WorkItem) {
        let new_nodes = self.allocator.allocate().unwrap();

        let parent = self.allocator.get_mut(self.work_item.node).unwrap();
        parent.set_left_first(Some(new_nodes.left as u32));
        parent.set_count(None);
        parent.bounds.offset_by(0.0001);

        new_nodes.left_node.bounds = left_box;
        new_nodes.right_node.bounds = right_box;

        // Allocate split space for the two children based on their SAH cost
        // This assumes that reference ranges look like this:
        //  item.begin..right_begin is the range of references on the left,
        //  right_begin..right_end is the range of references on the right,
        //  right_end..work_item.split_end is the free split space
        let remaining_split_count = self.work_item.split_end - right_end;
        let left_cost = left_box.half_area() * (right_begin - self.work_item.begin) as f32;
        let right_cost = right_box.half_area() * (right_end - right_begin) as f32;
        let left_split_count = if remaining_split_count == 0 {
            0
        } else {
            (remaining_split_count as f32 * (left_cost / (left_cost + right_cost))) as usize
        };

        // Move references of the right child to end of the list to leave some split space for the left one
        if left_split_count > 0 {
            unsafe {
                move_backward(
                    self.references[0].as_mut_ptr().add(right_begin),
                    self.references[0].as_mut_ptr().add(right_end),
                    self.references[0]
                        .as_mut_ptr()
                        .add(right_end + left_split_count),
                );
                move_backward(
                    self.references[1].as_mut_ptr().add(right_begin),
                    self.references[1].as_mut_ptr().add(right_end),
                    self.references[1]
                        .as_mut_ptr()
                        .add(right_end + left_split_count),
                );
                move_backward(
                    self.references[2].as_mut_ptr().add(right_begin),
                    self.references[2].as_mut_ptr().add(right_end),
                    self.references[2]
                        .as_mut_ptr()
                        .add(right_end + left_split_count),
                );
            }
        }

        let left_end = right_begin;
        let right_begin = right_begin + left_split_count;
        let right_end = right_end + left_split_count;

        debug_assert!(right_end <= self.work_item.split_end);

        let left_item = WorkItem::new(
            new_nodes.left,
            self.work_item.begin,
            left_end,
            right_begin,
            self.work_item.depth + 1,
            is_sorted,
        );

        let right_item = WorkItem::new(
            new_nodes.right,
            right_begin,
            right_end,
            self.work_item.split_end,
            self.work_item.depth + 1,
            is_sorted,
        );

        (left_item, right_item)
    }

    fn apply_object_split(&mut self, split: ObjectSplit) -> (WorkItem, WorkItem) {
        let other_axis = ((split.axis + 1) % 3, (split.axis + 2) % 3);
        let mut reference_marks = vec![false; self.prim_indices.len()];

        for i in self.work_item.begin..split.index {
            reference_marks[self.references[split.axis][i].prim_id as usize] = true;
        }
        for i in split.index..self.work_item.end {
            reference_marks[self.references[split.axis][i].prim_id as usize] = false;
        }

        let partition_predicate = |a: &SpatialReference, b: &SpatialReference| {
            let a_mark = reference_marks[a.prim_id as usize];
            let b_mark = reference_marks[b.prim_id as usize];
            match (a_mark, b_mark) {
                (false, true) => Ordering::Greater,
                (true, false) => Ordering::Less,
                _ => Ordering::Equal,
            }
        };

        self.references[other_axis.0].as_mut()[self.work_item.begin..self.work_item.end]
            .sort_by(partition_predicate);
        self.references[other_axis.1].as_mut()[self.work_item.begin..self.work_item.end]
            .sort_by(partition_predicate);

        self.allocate_children(
            split.index,
            self.work_item.end,
            split.left_box,
            split.right_box,
            true,
        )
    }

    fn run_binning_pass(
        &mut self,
        split: &mut SpatialSplit,
        axis: usize,
        begin: usize,
        end: usize,
        min: f32,
        max: f32,
    ) -> Option<(f32, f32)> {
        let bin_count = self.builder.bin_count;

        let mut bins = vec![
            SpatialBin {
                aabb: Aabb::empty(),
                accumulated_aabb: Aabb::empty(),
                entry: 0,
                exit: 0,
            };
            bin_count
        ];

        // Split primitives and add the bounding box of the fragments to the bins
        let bin_size = (max - min) / bin_count as f32;
        let inv_size = 1.0 / bin_size;

        for i in begin..end {
            let reference = &self.references[0][i];
            let first_bin = (bin_count - 1)
                .min((inv_size * (reference.aabb.min[axis] - min)).max(0.0) as usize);
            let last_bin = (bin_count - 1)
                .min((inv_size * (reference.aabb.max[axis] - min)).max(0.0) as usize);

            let mut current_aabb = reference.aabb;
            for (j, bin) in bins[first_bin..last_bin].iter_mut().enumerate() {
                let triangle = &self.builder.primitives[reference.prim_id as usize];
                let (mut left_box, right_box) =
                    triangle.split(axis, min + (j + first_bin + 1) as f32 * bin_size);
                left_box.shrink(&current_aabb);
                bin.aabb.grow_bb(&left_box);
                current_aabb.shrink(&right_box);
            }

            bins[last_bin].aabb.grow_bb(&current_aabb);
            bins[first_bin].entry += 1;
            bins[first_bin].exit += 1;
        }

        // Accumulate bounding boxes
        let mut current_aabb = Aabb::empty();
        let mut i = bin_count;
        while i > 0 {
            current_aabb.grow_bb(&bins[i - 1].aabb);
            bins[i - 1].aabb = current_aabb;
            i -= 1;
        }

        // Sweep and compute SAH cost
        let mut left_count = 0;
        let mut right_count = end - begin;
        let mut current_aabb = Aabb::empty();
        let mut found = false;
        for i in 0..(bin_count - 1) {
            left_count += bins[i].entry;
            right_count -= bins[i].exit;
            current_aabb.grow_bb(&bins[i].aabb);

            let cost = left_count as f32 * current_aabb.half_area()
                + right_count as f32 * bins[i + 1].aabb.half_area();
            if cost < split.cost {
                split.cost = cost;
                split.axis = axis;
                split.position = min + (i + 1) as f32 * bin_size;
                found = true;
            }
        }

        if found {
            Some((split.position - bin_size, split.position + bin_size))
        } else {
            None
        }
    }

    fn find_spatial_split(&mut self, begin: usize, end: usize) -> SpatialSplit {
        let node = self.allocator.get_mut(self.work_item.node).unwrap();
        let mut split = SpatialSplit::default();

        for axis in 0..3 {
            let mut min = node.bounds.min[axis];
            let mut max = node.bounds.max[axis];

            for _ in 0..self.builder.binning_pass_count {
                if let Some(next_bounds) =
                    self.run_binning_pass(&mut split, axis, begin, end, min, max)
                {
                    min = next_bounds.0;
                    max = next_bounds.1;
                } else {
                    break;
                }
            }
        }

        split
    }

    fn apply_spatial_split(&mut self, split: SpatialSplit) -> (WorkItem, WorkItem) {
        let mut left_end = self.work_item.begin;
        let mut right_begin = self.work_item.end;
        let mut right_end = self.work_item.end;

        let mut left_box = Aabb::empty();
        let mut right_box = Aabb::empty();

        // Choosing the references that are sorted on the split axis
        // is more efficient than the others, since fewer swaps are
        // necessary for primitives that are completely contained on
        // one side of the partition.
        let references_to_split = self.references[split.axis].as_mut();

        // Partition references such that:
        //  item.begin..left_end is on the left,
        //  left_end..right_begin is in between,
        //  right_begin..work_item.end is on the right
        let mut i = self.work_item.begin;
        while i < right_begin {
            let aabb = &references_to_split[i].aabb;
            if aabb.max[split.axis] <= split.position {
                left_box.grow_bb(aabb);
                references_to_split.swap(i, left_end);
                i += 1;
                left_end += 1;
            } else if aabb.min[split.axis] >= split.position {
                right_box.grow_bb(aabb);
                right_begin -= 1;
                references_to_split.swap(i, right_begin);
            } else {
                i += 1;
            }
        }

        let mut left_count = left_end - self.work_item.begin;
        let mut right_count = right_end - right_begin;
        if (left_count == 0 || right_count == 0) && left_end == right_begin {
            // Sometimes the algorithm will suggest a spatial split is possible, but all
            // references are on a single side. This code block splits the number of primitives
            // equally to either side to prevent every primitive going to one side.
            if left_count > 0 {
                left_end = left_count / 2;
            } else {
                left_end += right_count / 2;
            }
            right_begin = left_end;

            // Recompute the left and right bounding boxes
            left_box = Aabb::empty();
            right_box = Aabb::empty();

            for r in &references_to_split[self.work_item.begin..left_end] {
                left_box.grow_bb(&r.aabb);
            }

            for r in &references_to_split[left_end..self.work_item.end] {
                right_box.grow_bb(&r.aabb);
            }
        }

        // Handle straddling references
        while left_end < right_begin {
            let reference = references_to_split[left_end];
            let (mut left_prim_box, mut right_prim_box) = self.builder.primitives
                [reference.prim_id as usize]
                .split(split.axis, split.position);

            left_prim_box.shrink(&reference.aabb);
            right_prim_box.shrink(&reference.aabb);

            // Make sure there is enough space to split reference
            if self.work_item.split_end - right_end > 0 {
                left_box.grow_bb(&left_prim_box);
                right_box.grow_bb(&right_prim_box);

                references_to_split[right_end] = SpatialReference {
                    aabb: right_prim_box,
                    center: right_prim_box.center(),
                    prim_id: reference.prim_id,
                };

                references_to_split[left_end] = SpatialReference {
                    aabb: left_prim_box,
                    center: left_prim_box.center(),
                    prim_id: reference.prim_id,
                };

                right_end += 1;
                left_end += 1;

                left_count += 1;
                right_count += 1;
            } else if left_count < right_count {
                left_box.grow_bb(&reference.aabb);
                left_end += 1;
                left_count += 1;
            } else {
                right_box.grow_bb(&reference.aabb);
                right_begin -= 1;
                references_to_split.swap(right_begin, left_end);
                right_count += 1;
            }
        }

        self.references[(split.axis + 1) % 3].as_mut()[self.work_item.begin..right_end]
            .copy_from_slice(&references_to_split[self.work_item.begin..right_end]);
        self.references[(split.axis + 2) % 3].as_mut()[self.work_item.begin..right_end]
            .copy_from_slice(&references_to_split[self.work_item.begin..right_end]);

        debug_assert_eq!(left_end, right_begin);
        debug_assert!(right_end <= self.work_item.split_end);

        self.allocate_children(right_begin, right_end, left_box, right_box, false)
    }
}

impl<'a, T: Primitive<i32> + SpatialTriangle> Task for SpatialSahBuildTask<'a, T> {
    fn run(mut self) -> Option<(Self, Self)> {
        let node = self.allocator.get_mut(self.work_item.node).unwrap();
        let spatial_threshold = self.spatial_threshold;
        let reference_count = self.reference_count;
        let prim_indices = self.prim_indices.clone();
        let references = self.references.clone();

        let make_leaf = |node: &mut BvhNode, begin: usize, end: usize| {
            let prim_count = end - begin;
            let first_prim = reference_count.fetch_add(prim_count, SeqCst);

            for i in 0..prim_count {
                prim_indices.set(first_prim + i, references[0][begin + i].prim_id);
            }

            node.set_left_first(Some(first_prim as u32));
            node.set_count(Some(prim_count as u32));
        };

        if self.work_size() <= 1 || self.depth() >= self.builder.max_depth {
            make_leaf(node, self.work_item.begin, self.work_item.end);
            return None;
        }

        let mut best_object_split = self.find_object_split(
            self.work_item.begin,
            self.work_item.end,
            self.work_item.is_sorted,
        );

        let mut best_spatial_split = SpatialSplit::default();
        let mut overlap = best_object_split.left_box;
        overlap.shrink(&best_object_split.right_box);
        let overlap = overlap.area();

        if overlap > spatial_threshold && (self.work_item.split_end - self.work_item.end) > 0 {
            best_spatial_split = self.find_spatial_split(self.work_item.begin, self.work_item.end);
        }

        let best_cost = best_spatial_split.cost.min(best_object_split.cost);
        let mut use_spatial_split = best_cost < best_object_split.cost;

        let max_split_cost = node.bounds.half_area()
            * (self.work_item.work_size() as f32 - self.builder.traversal_cost);
        // Make sure the cost of splitting does not exceed the cost of not splitting
        if best_cost >= max_split_cost {
            // In case splitting results in worse node costs, use median split as fallback strategy
            if self.work_item.work_size() > self.builder.max_leaf_size {
                use_spatial_split = false;
                best_object_split.index = (self.work_item.begin + self.work_item.end) / 2;
                best_object_split.axis = node.bounds.longest_axis();
                best_object_split.left_box = Aabb::empty();
                best_object_split.right_box = Aabb::empty();

                for i in self.work_item.begin..best_object_split.index {
                    best_object_split
                        .left_box
                        .grow_bb(&self.references[best_object_split.axis][i].aabb);
                }
                for i in best_object_split.index..self.work_item.end {
                    best_object_split
                        .right_box
                        .grow_bb(&self.references[best_object_split.axis][i].aabb);
                }
            } else {
                make_leaf(node, self.work_item.begin, self.work_item.end);
                return None;
            }
        }

        let builder = self.builder;
        let allocator = self.allocator.clone();
        let references = self.references.clone();
        let reference_count = self.reference_count;
        let prim_indices = self.prim_indices.clone();
        let accumulated_aabbs = self.accumulated_aabbs.clone();

        let (work_a, work_b) = if use_spatial_split {
            self.apply_spatial_split(best_spatial_split)
        } else {
            self.apply_object_split(best_object_split)
        };

        let task_a = Self::new(
            builder,
            allocator.clone(),
            references.clone(),
            reference_count,
            prim_indices.clone(),
            accumulated_aabbs.clone(),
            work_a,
            spatial_threshold,
        );

        let task_b = Self::new(
            builder,
            allocator,
            references,
            reference_count,
            prim_indices,
            accumulated_aabbs,
            work_b,
            spatial_threshold,
        );

        Some((task_a, task_b))
    }

    fn work_size(&self) -> usize {
        self.work_item.work_size()
    }

    fn depth(&self) -> usize {
        self.work_item.depth
    }
}

pub struct SpatialSahBuilder<'a, T: Primitive<i32> + SpatialTriangle> {
    aabbs: &'a [Aabb<i32>],
    primitives: &'a [T],

    binning_pass_count: usize,
    max_depth: usize,
    bin_count: usize,
    max_leaf_size: usize,

    traversal_cost: f32,
    alpha: f32,
    split_factor: f32,
}

#[allow(dead_code)]
impl<'a, T: Primitive<i32> + SpatialTriangle> SpatialSahBuilder<'a, T> {
    pub(crate) fn new(
        aabbs: &'a [Aabb<i32>],
        primitives: &'a [T],
        primitives_per_leaf: Option<NonZeroUsize>,
    ) -> Self {
        Self {
            aabbs,
            primitives,
            binning_pass_count: 2,
            max_depth: 64,
            bin_count: 16,
            max_leaf_size: primitives_per_leaf.map(|p| p.get()).unwrap_or(1),
            traversal_cost: 1.0,
            alpha: 1e-5,
            split_factor: 0.75,
        }
    }

    /// Number of spatial binning passes that are run in order to
    /// find a spatial split. This increases accuracy without
    /// increasing the number of bins.
    pub(crate) fn with_binning_pass_count(mut self, pass_count: usize) -> Self {
        self.binning_pass_count = pass_count;
        self
    }

    /// Maximum depth of tree
    pub(crate) fn with_max_depth(mut self, depth: usize) -> Self {
        self.max_depth = depth;
        self
    }

    /// Maximum number of bins to check for splitting a node
    pub(crate) fn with_bin_count(mut self, bin_count: usize) -> Self {
        self.bin_count = bin_count;
        self
    }

    /// Maximum number of primitives inside a node
    pub(crate) fn with_max_leaf_size(mut self, max_leaf_size: usize) -> Self {
        self.max_leaf_size = max_leaf_size;
        self
    }

    /// Cost to determine for splitting a node with the SAH algorithm for traversing a node
    pub(crate) fn with_traversal_cost(mut self, traversal_cost: f32) -> Self {
        self.traversal_cost = traversal_cost;
        self
    }

    pub(crate) fn with_split_factor(mut self, split_factor: f32) -> Self {
        self.split_factor = split_factor;
        self
    }

    pub(crate) fn with_alpha(mut self, alpha: f32) -> Self {
        self.alpha = alpha.min(1.0).max(1e-6);
        self
    }
}

impl<'a, T: Primitive<i32> + SpatialTriangle> BuildAlgorithm for SpatialSahBuilder<'a, T> {
    fn build(self) -> Bvh {
        if self.primitives.len() != self.aabbs.len()
            || self.primitives.is_empty()
            || self.aabbs.is_empty()
        {
            return Bvh::default();
        }

        let prim_count = self.aabbs.len();
        let max_reference_count = prim_count + (prim_count as f32 * self.split_factor) as usize;
        let reference_count = AtomicUsize::new(0);

        let mut nodes = vec![BvhNode::new(); 2 * max_reference_count + 1];
        let mut prim_indices = vec![0_u32; max_reference_count];
        let mut accumulated_bboxes = vec![Aabb::empty(); max_reference_count];

        // The references storage containers
        let mut reference_data0 = vec![SpatialReference::default(); max_reference_count];
        let mut reference_data1 = vec![SpatialReference::default(); max_reference_count];
        let mut reference_data2 = vec![SpatialReference::default(); max_reference_count];

        // These wrappers are unsafe, but they allow for must faster builds
        let references = [
            UnsafeSliceWrapper::new(reference_data0.as_mut_slice()),
            UnsafeSliceWrapper::new(reference_data1.as_mut_slice()),
            UnsafeSliceWrapper::new(reference_data2.as_mut_slice()),
        ];

        let root_bounds = Aabb::union_of_list(self.aabbs);

        // Compute the spatial split threshold
        let spatial_threshold = self.alpha * 2.0 * root_bounds.half_area();
        let node_count = AtomicUsize::new(1);
        let (node_stack, first_node) = AtomicNodeStack::new(&node_count, nodes.as_mut_slice());

        #[cfg(feature = "wasm_support")]
        references.iter().for_each(|r| {
            r.as_mut()[0..prim_count]
                .iter_mut()
                .enumerate()
                .for_each(|(i, sref)| {
                    sref.aabb = self.aabbs[i];
                    sref.center = self.primitives.center();
                    sref.prim_id = i as u32;
                });
        });

        #[cfg(not(feature = "wasm_support"))]
        references.iter().par_bridge().for_each(|r| {
            r.as_mut()[0..prim_count]
                .iter_mut()
                .enumerate()
                .for_each(|(i, sref)| {
                    sref.aabb = self.aabbs[i];
                    sref.center = self.primitives[i].center();
                    sref.prim_id = i as u32;
                });
        });

        let prim_wrapper = UnsafeSliceWrapper::new(prim_indices.as_mut());
        let accumulated_aabbs_wrapper = UnsafeSliceWrapper::new(accumulated_bboxes.as_mut());
        let task_spawner = TaskSpawner::new();
        first_node.bounds = Aabb::union_of_list(self.aabbs);

        // Initialize the first task, this task will spawn subsequent tasks
        let root_task = SpatialSahBuildTask::new(
            &self,
            node_stack,
            references,
            &reference_count,
            prim_wrapper,
            accumulated_aabbs_wrapper,
            WorkItem {
                node: 0,
                begin: 0,
                end: prim_count,
                depth: 0,
                split_end: max_reference_count,
                is_sorted: false,
            },
            spatial_threshold,
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

        // Launch build task
        #[cfg(not(feature = "wasm_support"))]
        task_spawner.run(root_task);

        nodes.resize(node_count.load(SeqCst), BvhNode::new());

        // Compose bvh
        Bvh {
            nodes,
            prim_indices,
            build_type: BuildType::Spatial,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tests::Triangle;
    use crate::Bounds;

    #[test]
    fn no_primitives() {
        let (aabbs, primitives) = crate::tests::load_teapot();

        let builder = SpatialSahBuilder::new(&[], primitives.as_slice(), None);
        assert!(builder.build().nodes.is_empty());

        let builder: SpatialSahBuilder<Triangle> = SpatialSahBuilder::new(&aabbs, &[], None);
        assert!(builder.build().nodes.is_empty());

        let builder = SpatialSahBuilder::new(&aabbs, &primitives, None);
        let build = builder.build();
        assert!(!build.nodes.is_empty());
        assert!(build.nodes.len() >= aabbs.len());
        assert!(build.nodes.len() <= (2 * aabbs.len()));
    }

    #[test]
    fn test_spatial_sah_build() {
        let (aabbs, primitives) = crate::tests::load_teapot();

        let builder = SpatialSahBuilder::new(aabbs.as_slice(), primitives.as_slice(), None);
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
                t.vertex0()
            );
            assert!(
                bounds.contains(t.vertex1()),
                "Bvh did not contain vertex 1 of primitive {}, bvh-bounds: {}, vertex: {}",
                i,
                bounds,
                t.vertex1()
            );
            assert!(
                bounds.contains(t.vertex2()),
                "Bvh did not contain vertex 2 of primitive {}, bvh-bounds: {}, vertex: {}",
                i,
                bounds,
                t.vertex2()
            );
        }
    }
}
