use crate::bvh_node::*;
use crate::{RayPacket4, AABB};
use serde::{Deserialize, Serialize};

use glam::*;

struct MBVHHit {
    ids: [u8; 4],
    result: [bool; 4],
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[repr(C)]
pub struct MBVHNode {
    min_x: [f32; 4],
    max_x: [f32; 4],
    min_y: [f32; 4],
    max_y: [f32; 4],
    min_z: [f32; 4],
    max_z: [f32; 4],
    pub children: [i32; 4],
    pub counts: [i32; 4],
}

impl std::default::Default for MBVHNode {
    fn default() -> Self {
        MBVHNode {
            min_x: [0.0; 4],
            max_x: [0.0; 4],
            min_y: [0.0; 4],
            max_y: [0.0; 4],
            min_z: [0.0; 4],
            max_z: [0.0; 4],
            children: [0; 4],
            counts: [0; 4],
        }
    }
}

impl MBVHNode {
    pub fn new() -> MBVHNode {
        let min_x = [1e34; 4];
        let min_y = [1e34; 4];
        let min_z = [1e34; 4];

        let max_x = [-1e34; 4];
        let max_y = [-1e34; 4];
        let max_z = [-1e34; 4];

        let children = [-1; 4];
        let counts = [-1; 4];

        MBVHNode {
            min_x: min_x,
            max_x: max_x,
            min_y: min_y,
            max_y: max_y,
            min_z: min_z,
            max_z: max_z,
            children,
            counts,
        }
    }

    pub fn points(&self) -> (Vec4, Vec4, Vec4, Vec4, Vec4, Vec4) {
        #[cfg(all(
            any(target_arch = "x86_64", target_arch = "x86"),
            not(feature = "wasm_support")
        ))]
        {
            (
                Vec4::from(unsafe { core::arch::x86_64::_mm_load_ps(self.min_x.as_ptr()) }),
                Vec4::from(unsafe { core::arch::x86_64::_mm_load_ps(self.max_x.as_ptr()) }),
                Vec4::from(unsafe { core::arch::x86_64::_mm_load_ps(self.min_y.as_ptr()) }),
                Vec4::from(unsafe { core::arch::x86_64::_mm_load_ps(self.max_y.as_ptr()) }),
                Vec4::from(unsafe { core::arch::x86_64::_mm_load_ps(self.min_z.as_ptr()) }),
                Vec4::from(unsafe { core::arch::x86_64::_mm_load_ps(self.max_z.as_ptr()) }),
            )
        }
        #[cfg(any(
            not(any(target_arch = "x86_64", target_arch = "x86")),
            feature = "wasm_support"
        ))]
        {
            (
                Vec4::from(self.min_x),
                Vec4::from(self.max_x),
                Vec4::from(self.min_y),
                Vec4::from(self.max_y),
                Vec4::from(self.min_z),
                Vec4::from(self.max_z),
            )
        }
    }

    pub fn min_points(&self) -> (Vec4, Vec4, Vec4) {
        #[cfg(all(
            any(target_arch = "x86_64", target_arch = "x86"),
            not(feature = "wasm_support")
        ))]
        {
            (
                Vec4::from(unsafe { core::arch::x86_64::_mm_load_ps(self.min_x.as_ptr()) }),
                Vec4::from(unsafe { core::arch::x86_64::_mm_load_ps(self.min_y.as_ptr()) }),
                Vec4::from(unsafe { core::arch::x86_64::_mm_load_ps(self.min_z.as_ptr()) }),
            )
        }
        #[cfg(any(
            not(any(target_arch = "x86_64", target_arch = "x86")),
            feature = "wasm_support"
        ))]
        {
            (
                Vec4::from(self.min_x),
                Vec4::from(self.min_y),
                Vec4::from(self.min_z),
            )
        }
    }

    pub fn max_points(&self) -> (Vec4, Vec4, Vec4) {
        #[cfg(all(
            any(target_arch = "x86_64", target_arch = "x86"),
            not(feature = "wasm_support")
        ))]
        {
            (
                Vec4::from(unsafe { core::arch::x86_64::_mm_load_ps(self.max_x.as_ptr()) }),
                Vec4::from(unsafe { core::arch::x86_64::_mm_load_ps(self.max_y.as_ptr()) }),
                Vec4::from(unsafe { core::arch::x86_64::_mm_load_ps(self.max_z.as_ptr()) }),
            )
        }
        #[cfg(any(
            not(any(target_arch = "x86_64", target_arch = "x86")),
            feature = "wasm_support"
        ))]
        {
            (
                Vec4::from(self.max_x),
                Vec4::from(self.max_y),
                Vec4::from(self.max_z),
            )
        }
    }

    pub fn set_bounds(&mut self, node_id: usize, min: &[f32; 3], max: &[f32; 3]) {
        debug_assert!(node_id < 4);
        self.min_x[node_id] = min[0];
        self.min_y[node_id] = min[1];
        self.min_z[node_id] = min[2];

        self.max_x[node_id] = max[0];
        self.max_y[node_id] = max[1];
        self.max_z[node_id] = max[2];
    }

    pub fn set_bounds_bb(&mut self, node_id: usize, bounds: &AABB) {
        self.set_bounds(node_id, &bounds.min, &bounds.max);
    }

    #[inline(always)]
    fn intersect(&self, origin: Vec3A, inv_direction: Vec3A, t: f32) -> Option<MBVHHit> {
        let origin_x: Vec4 = Vec4::splat(origin.x());
        let origin_y: Vec4 = Vec4::splat(origin.y());
        let origin_z: Vec4 = Vec4::splat(origin.z());

        let inv_dir_x: Vec4 = Vec4::splat(inv_direction.x());
        let inv_dir_y: Vec4 = Vec4::splat(inv_direction.y());
        let inv_dir_z: Vec4 = Vec4::splat(inv_direction.z());

        let (min_x, max_x, min_y, max_y, min_z, max_z) = self.points();

        let tx0: Vec4 = (min_x - origin_x) * inv_dir_x;
        let tx1: Vec4 = (max_x - origin_x) * inv_dir_x;
        let ty0: Vec4 = (min_y - origin_y) * inv_dir_y;
        let ty1: Vec4 = (max_y - origin_y) * inv_dir_y;
        let tz0: Vec4 = (min_z - origin_z) * inv_dir_z;
        let tz1: Vec4 = (max_z - origin_z) * inv_dir_z;

        let tx_min = tx0.min(tx1);
        let tx_max = tx0.max(tx1);
        let ty_min = ty0.min(ty1);
        let ty_max = ty0.max(ty1);
        let tz_min = tz0.min(tz1);
        let tz_max = tz0.max(tz1);

        let mut t_min = tx_min.max(ty_min.max(tz_min));
        let t_max = tx_max.min(ty_max.min(tz_max));

        let result: Vec4Mask = t_max.cmpge(t_min) & t_min.cmplt(Vec4::splat(t));
        let result = result.bitmask();

        let result = [
            (result & 1) != 0,
            (result & 2) != 0,
            (result & 4) != 0,
            (result & 8) != 0,
        ];

        let mut ids = [0, 1, 2, 3];

        let t_minf = t_min.as_mut();

        if t_minf[0] > t_minf[1] {
            t_minf.swap(0, 1);
            ids.swap(0, 1);
        }
        if t_minf[2] > t_minf[3] {
            t_minf.swap(2, 3);
            ids.swap(2, 3);
        }
        if t_minf[0] > t_minf[2] {
            t_minf.swap(0, 2);
            ids.swap(0, 2);
        }
        if t_minf[1] > t_minf[3] {
            t_minf.swap(1, 3);
            ids.swap(1, 3);
        }
        if t_minf[2] > t_minf[3] {
            ids.swap(2, 3);
        }

        Some(MBVHHit { ids, result })
    }

    #[inline(always)]
    fn intersect4(
        &self,
        packet: &RayPacket4,
        inv_dir_x: Vec4,
        inv_dir_y: Vec4,
        inv_dir_z: Vec4,
    ) -> Option<MBVHHit> {
        let (min_x, max_x, min_y, max_y, min_z, max_z) = self.points();

        let mut result = Vec4Mask::new(false, false, false, false);
        let mut final_t_min = Vec4::splat(1e20);

        for i in 0..4 {
            let org_comp = Vec4::splat(packet.origin_x[i]);
            let dir_comp = Vec4::splat(inv_dir_x[i]);

            let t1 = (min_x - org_comp) * dir_comp;
            let t2 = (max_x - org_comp) * dir_comp;

            let t_min = t1.min(t2);
            let t_max = t1.max(t2);

            let org_comp = Vec4::splat(packet.origin_y[i]);
            let dir_comp = Vec4::splat(inv_dir_y[i]);

            let t1 = (min_y - org_comp) * dir_comp;
            let t2 = (max_y - org_comp) * dir_comp;

            let t_min = t_min.max(t1.min(t2));
            let t_max = t_max.min(t1.max(t2));

            let org_comp = Vec4::splat(packet.origin_z[i]);
            let dir_comp = Vec4::splat(inv_dir_z[i]);

            let t1 = (min_z - org_comp) * dir_comp;
            let t2 = (max_z - org_comp) * dir_comp;

            let t_min: Vec4 = t_min.max(t1.min(t2));
            let t_max: Vec4 = t_max.min(t1.max(t2));

            let greater_than_min = t_max.cmpgt(t_min);
            let less_than_t = t_min.cmplt(Vec4::splat(packet.t[i]));
            result = result | (greater_than_min & less_than_t);

            final_t_min = final_t_min.min(t_min);
        }

        let result = result.bitmask();
        if result == 0 {
            return None;
        }

        let result = [
            (result & 1) != 0,
            (result & 2) != 0,
            (result & 4) != 0,
            (result & 8) != 0,
        ];

        let ids = [0, 1, 2, 3];
        Some(MBVHHit { ids, result })
    }

    #[inline(always)]
    pub fn traverse<I, R>(
        tree: &[MBVHNode],
        prim_indices: &[u32],
        origin: Vec3A,
        dir: Vec3A,
        t_min: f32,
        t_max: f32,
        mut intersection_test: I,
    ) -> Option<R>
    where
        I: FnMut(usize, f32, f32) -> Option<(f32, R)>,
        R: Copy,
    {
        let mut todo = [0; 64];
        let mut stack_ptr = 0;
        let dir_inverse = Vec3A::one() / dir;
        let mut t = t_max;
        let mut hit_record = None;

        while stack_ptr >= 0 {
            let left_first = todo[stack_ptr as usize] as usize;

            #[cfg(all(
                any(target_arch = "x86_64", target_arch = "x86"),
                not(feature = "wasm_support")
            ))]
            unsafe {
                core::arch::x86_64::_mm_prefetch(
                    tree.as_ptr().add(left_first as usize) as *const i8,
                    core::arch::x86_64::_MM_HINT_T0,
                )
            };

            stack_ptr = stack_ptr - 1;

            if let Some(hit) = tree[left_first].intersect(origin, dir_inverse, t) {
                for i in (0..4).rev() {
                    let id = hit.ids[i as usize] as usize;

                    if hit.result[id] {
                        let count = tree[left_first].counts[id];
                        let left_first = tree[left_first].children[id];
                        if count >= 0 {
                            for i in 0..count {
                                let prim_id = prim_indices[(left_first + i) as usize] as usize;
                                if let Some((new_t, new_hit)) =
                                    intersection_test(prim_id as usize, t_min, t)
                                {
                                    t = new_t;
                                    hit_record = Some(new_hit);
                                }
                            }
                        } else if left_first >= 0 {
                            stack_ptr += 1;
                            let stack_ptr = stack_ptr as usize;
                            todo[stack_ptr] = left_first as u32;

                            #[cfg(all(
                                any(target_arch = "x86_64", target_arch = "x86"),
                                not(feature = "wasm_support")
                            ))]
                            unsafe {
                                core::arch::x86_64::_mm_prefetch(
                                    tree.as_ptr().add(left_first as usize) as *const i8,
                                    core::arch::x86_64::_MM_HINT_T0,
                                )
                            };
                        }
                    }
                }
            }
        }

        hit_record
    }

    #[inline(always)]
    pub fn traverse_t<I>(
        tree: &[MBVHNode],
        prim_indices: &[u32],
        origin: Vec3A,
        dir: Vec3A,
        t_min: f32,
        t_max: f32,
        mut intersection_test: I,
    ) -> Option<f32>
    where
        I: FnMut(usize, f32, f32) -> Option<f32>,
    {
        let mut todo = [0; 64];
        let mut stack_ptr = -1;
        let dir_inverse = Vec3A::one() / dir;
        let mut t = t_max;

        while stack_ptr >= 0 {
            let left_first = todo[stack_ptr as usize] as usize;

            #[cfg(all(
                any(target_arch = "x86_64", target_arch = "x86"),
                not(feature = "wasm_support")
            ))]
            unsafe {
                core::arch::x86_64::_mm_prefetch(
                    tree.as_ptr().add(left_first as usize) as *const i8,
                    core::arch::x86_64::_MM_HINT_T0,
                )
            };

            stack_ptr -= 1;

            if let Some(hit) = tree[left_first].intersect(origin, dir_inverse, t) {
                for i in (0..4).rev() {
                    let id = hit.ids[i as usize] as usize;

                    if hit.result[id] {
                        let count = tree[left_first].counts[id];
                        let left_first = tree[left_first].children[id];
                        if count >= 0 {
                            for i in 0..count {
                                let prim_id = prim_indices[(left_first + i) as usize] as usize;
                                if let Some(new_t) = intersection_test(prim_id, t_min, t) {
                                    t = new_t;
                                }
                            }
                        } else if left_first >= 0 {
                            stack_ptr += 1;
                            todo[stack_ptr as usize] = left_first as u32;
                        }
                    }
                }
            }
        }

        if t < t_max {
            Some(t)
        } else {
            None
        }
    }

    #[inline(always)]
    pub fn occludes<I>(
        tree: &[MBVHNode],
        prim_indices: &[u32],
        origin: Vec3A,
        dir: Vec3A,
        t_min: f32,
        t_max: f32,
        mut intersection_test: I,
    ) -> bool
    where
        I: FnMut(usize, f32, f32) -> bool,
    {
        let mut todo = [0; 64];
        let mut stack_ptr = -1;
        let dir_inverse = Vec3A::one() / dir;
        let t = t_max;

        while stack_ptr >= 0 {
            let left_first = todo[stack_ptr as usize] as usize;

            #[cfg(all(
                any(target_arch = "x86_64", target_arch = "x86"),
                not(feature = "wasm_support")
            ))]
            unsafe {
                core::arch::x86_64::_mm_prefetch(
                    tree.as_ptr().add(left_first as usize) as *const i8,
                    core::arch::x86_64::_MM_HINT_T0,
                )
            };

            stack_ptr -= 1;

            if let Some(hit) = tree[left_first].intersect(origin, dir_inverse, t) {
                for i in (0..4).rev() {
                    let id = hit.ids[i as usize] as usize;

                    if hit.result[id] {
                        let count = tree[left_first].counts[id];
                        let left_first = tree[left_first].children[id];
                        if count >= 0 {
                            for i in 0..count {
                                let prim_id = prim_indices[(left_first + i) as usize] as usize;
                                if intersection_test(prim_id, t_min, t) {
                                    return true;
                                }
                            }
                        } else if left_first >= 0 {
                            stack_ptr += 1;
                            todo[stack_ptr as usize] = left_first as u32;
                        }
                    }
                }
            }
        }

        false
    }

    #[inline(always)]
    pub fn depth_test<I>(
        tree: &[MBVHNode],
        prim_indices: &[u32],
        origin: Vec3A,
        dir: Vec3A,
        t_min: f32,
        t_max: f32,
        depth_test: I,
    ) -> (f32, u32)
    where
        I: Fn(usize, f32, f32) -> Option<(f32, u32)>,
    {
        let mut todo = [0; 64];
        let mut stack_ptr: i32 = 0;
        let dir_inverse = Vec3A::one() / dir;
        let mut t = t_max;
        let mut depth: u32 = 0;

        while stack_ptr >= 0 {
            let left_first = todo[stack_ptr as usize] as usize;

            #[cfg(all(
                any(target_arch = "x86_64", target_arch = "x86"),
                not(feature = "wasm_support")
            ))]
            unsafe {
                core::arch::x86_64::_mm_prefetch(
                    tree.as_ptr().add(left_first as usize) as *const i8,
                    core::arch::x86_64::_MM_HINT_T0,
                )
            };

            stack_ptr = stack_ptr - 1;

            if let Some(hit) = tree[left_first].intersect(origin, dir_inverse, t) {
                for i in (0..4).rev() {
                    // let id = unsafe { hit.t_min.t_min_int[i as usize] & 0b11 } as usize;
                    let id = hit.ids[i as usize] as usize;

                    if hit.result[id] {
                        let count = tree[left_first].counts[id];
                        let left_first = tree[left_first].children[id];
                        if count >= 0 {
                            for i in 0..count {
                                let prim_id = prim_indices[(left_first + i) as usize] as usize;
                                if let Some((new_t, d)) = depth_test(prim_id, t_min, t) {
                                    t = new_t;
                                    depth += d;
                                }
                            }
                        } else if left_first >= 0 {
                            stack_ptr += 1;
                            todo[stack_ptr as usize] = left_first as u32;
                        }
                    }
                }

                depth += 1;
            }
        }

        (t, depth)
    }

    #[inline(always)]
    pub fn traverse4<I: FnMut(usize, &mut RayPacket4)>(
        tree: &[MBVHNode],
        prim_indices: &[u32],
        packet: &mut RayPacket4,
        mut intersection_test: I,
    ) {
        let mut todo = [0; 64];
        let mut stack_ptr = 0;

        let one = Vec4::one();
        let inv_dir_x = one / Vec4::from(packet.direction_x);
        let inv_dir_y = one / Vec4::from(packet.direction_y);
        let inv_dir_z = one / Vec4::from(packet.direction_z);

        while stack_ptr >= 0 {
            let left_first = todo[stack_ptr as usize] as usize;

            #[cfg(all(
                any(target_arch = "x86_64", target_arch = "x86"),
                not(feature = "wasm_support")
            ))]
            unsafe {
                core::arch::x86_64::_mm_prefetch(
                    tree.as_ptr().add(left_first as usize) as *const i8,
                    core::arch::x86_64::_MM_HINT_T0,
                )
            };

            stack_ptr -= 1;

            if let Some(hit) = tree[left_first].intersect4(packet, inv_dir_x, inv_dir_y, inv_dir_z)
            {
                for i in (0..4).rev() {
                    let id = hit.ids[i as usize] as usize;

                    if hit.result[id] {
                        let count = tree[left_first].counts[id];
                        let left_first = tree[left_first].children[id];
                        if count >= 0 {
                            for i in 0..count {
                                let prim_id = prim_indices[(left_first + i) as usize] as usize;
                                intersection_test(prim_id, packet);
                            }
                        } else if left_first >= 0 {
                            stack_ptr += 1;
                            todo[stack_ptr as usize] = left_first as u32;

                            #[cfg(all(
                                any(target_arch = "x86_64", target_arch = "x86"),
                                not(feature = "wasm_support")
                            ))]
                            unsafe {
                                core::arch::x86_64::_mm_prefetch(
                                    tree.as_ptr().add(left_first as usize) as *const i8,
                                    core::arch::x86_64::_MM_HINT_T0,
                                )
                            };
                        }
                    }
                }
            }
        }
    }

    #[inline(always)]
    pub fn merge_nodes(
        m_index: usize,
        cur_node: usize,
        bvh_pool: &[BVHNode],
        mbvh_pool: &mut [MBVHNode],
        pool_ptr: &mut usize,
    ) {
        let cur_node = &bvh_pool[cur_node];
        if cur_node.is_leaf() {
            panic!("Leaf nodes should not be attempted to be split!");
        } else if m_index >= mbvh_pool.len() {
            panic!(format!("Index {} is out of bounds!", m_index));
        }

        let num_children = mbvh_pool[m_index].merge_node(cur_node, bvh_pool);

        for idx in 0..num_children {
            if mbvh_pool[m_index].children[idx] < 0 {
                mbvh_pool[m_index].set_bounds(idx, &[1e34; 3], &[-1e34; 3]);
                mbvh_pool[m_index].children[idx] = 0;
                mbvh_pool[m_index].counts[idx] = 0;
                continue;
            }

            if mbvh_pool[m_index].counts[idx] < 0 {
                // Not a leaf node
                let cur_node = mbvh_pool[m_index].children[idx] as usize;
                let new_idx = *pool_ptr;
                *pool_ptr = *pool_ptr + 1;
                mbvh_pool[m_index].children[idx] = new_idx as i32;
                Self::merge_nodes(new_idx, cur_node, bvh_pool, mbvh_pool, pool_ptr);
            }
        }
    }

    fn merge_node(&mut self, node: &BVHNode, pool: &[BVHNode]) -> usize {
        self.children = [-1; 4];
        self.counts = [-1; 4];
        let mut num_children = 0;

        let left_first = node.get_left_first();
        if left_first.is_none() {
            return num_children;
        }

        let left_first = left_first.unwrap();
        let left_node = &pool[left_first as usize];
        let right_node = &pool[(left_first + 1) as usize];

        if left_node.is_leaf() {
            let idx = num_children;
            if let Some(left_first) = left_node.get_left_first() {
                num_children += 1;
                self.children[idx] = left_first as i32;
                self.counts[idx] = left_node.get_count_unchecked();
                self.set_bounds_bb(idx, &left_node.bounds);
            }
        } else {
            // Node has children

            if let Some(left) = left_node.get_left_first() {
                let right = left + 1;

                let left_node = &pool[left as usize];
                let right_node = &pool[right as usize];

                if let Some(left_first) = left_node.get_left_first() {
                    let idx1 = num_children;
                    num_children += 1;
                    self.set_bounds_bb(idx1, &left_node.bounds);
                    if left_node.is_leaf() {
                        self.children[idx1] = left_first as i32;
                        self.counts[idx1] = left_node.get_count_unchecked();
                    } else {
                        self.children[idx1] = left as i32;
                        self.counts[idx1] = -1;
                    }
                }

                if let Some(left_first) = right_node.get_left_first() {
                    let idx2 = num_children;
                    num_children += 1;
                    self.set_bounds_bb(idx2, &right_node.bounds);
                    if right_node.is_leaf() {
                        self.children[idx2] = left_first as i32;
                        self.counts[idx2] = right_node.get_count_unchecked();
                    } else {
                        self.children[idx2] = right as i32;
                        self.counts[idx2] = -1;
                    }
                }
            }
        }

        if let Some(left_first) = right_node.get_left_first() {
            if right_node.is_leaf() {
                // Node only has a single child
                let idx = num_children;
                num_children += 1;
                self.set_bounds_bb(idx, &right_node.bounds);

                self.children[idx] = left_first as i32;
                self.counts[idx] = right_node.get_count_unchecked();
            } else {
                if let Some(left) = right_node.get_left_first() {
                    let right = left + 1;
                    let left_node = &pool[left as usize];
                    let right_node = &pool[right as usize];

                    if let Some(left_first) = left_node.get_left_first() {
                        let idx1 = num_children;
                        num_children += 1;
                        self.set_bounds_bb(idx1, &left_node.bounds);
                        if left_node.is_leaf() {
                            self.children[idx1] = left_first as i32;
                            self.counts[idx1] = left_node.get_count_unchecked();
                        } else {
                            self.children[idx1] = left as i32;
                            self.counts[idx1] = -1;
                        }
                    }

                    if let Some(left_first) = right_node.get_left_first() {
                        let idx2 = num_children;
                        num_children += 1;
                        self.set_bounds_bb(idx2, &right_node.bounds);
                        if right_node.is_leaf() {
                            self.children[idx2] = left_first as i32;
                            self.counts[idx2] = right_node.get_count_unchecked();
                        } else {
                            self.children[idx2] = right as i32;
                            self.counts[idx2] = -1;
                        }
                    }
                }
            }
        }

        // In case this quad node isn't filled & not all nodes are leaf nodes, merge 1 more node
        let mut merged = true;
        while num_children < 4 {
            if !merged {
                break;
            }
            merged = false;

            for i in 0..3 {
                if self.counts[i] >= 0 {
                    continue;
                }

                if self.children[i] < 0 {
                    continue;
                }

                let left = self.children[i];
                let right = left + 1;
                let left_sub_node = &pool[left as usize];
                let right_sub_node = &pool[right as usize];

                if let Some(left_first) = left_sub_node.get_left_first() {
                    // Overwrite current node
                    self.set_bounds_bb(i, &left_sub_node.bounds);
                    if left_sub_node.is_leaf() {
                        self.children[i] = left_first as i32;
                        self.counts[i] = left_sub_node.get_count_unchecked();
                    } else {
                        self.children[i] = left;
                        self.counts[i] = -1;
                    }

                    // Add its right node
                    self.set_bounds_bb(num_children, &right_sub_node.bounds);
                    if right_sub_node.is_leaf() {
                        self.children[num_children] = right_sub_node.get_left_first_unchecked();
                        self.counts[num_children] = right_sub_node.get_count_unchecked();
                    } else {
                        self.children[num_children] = right;
                        self.counts[num_children] = -1;
                    }

                    num_children += 1;
                    merged = true;
                } else if let Some(left_first) = right_sub_node.get_left_first() {
                    if right_sub_node.is_leaf() {
                        self.children[i] = left_first as i32;
                        self.counts[i] = right_sub_node.get_count_unchecked();
                    } else {
                        self.children[i] = right;
                        self.counts[i] = -1;
                    }
                    merged = true;
                }

                break;
            }
        }

        num_children
    }
}
